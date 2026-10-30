//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Include-tree inputs.
///
//===----------------------------------------------------------------------===//

#include "clang/AST/ASTConsumer.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/DynamicRecursiveASTVisitor.h"
#include "clang/CAS/IncludeTree.h"
#include "clang/Frontend/CompileJobCacheResult.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/CompilerInvocation.h"
#include "clang/Frontend/FrontendActions.h"
#include "clang/Frontend/TextDiagnosticBuffer.h"
#include "llvm/ADT/SmallBitVector.h"
#include "llvm/CAS/ActionCache.h"
#include "llvm/CAS/ObjectStore.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/VirtualFileSystem.h"
#include "llvm/Testing/Support/Error.h"
#include "llvm/Testing/Support/SupportHelpers.h"
#include "gtest/gtest.h"

using namespace clang;
using namespace clang::cas;
using llvm::Succeeded;

namespace {

/// Collects the names of the declarations in the main translation unit.
class CollectDeclsAction : public ASTFrontendAction {
public:
  std::vector<std::string> DeclNames;

  std::unique_ptr<ASTConsumer> CreateASTConsumer(CompilerInstance &CI,
                                                 StringRef InFile) override {
    return std::make_unique<Visitor>(DeclNames);
  }

private:
  class Visitor : public ASTConsumer, public DynamicRecursiveASTVisitor {
  public:
    Visitor(std::vector<std::string> &DeclNames) : DeclNames(DeclNames) {}

    void HandleTranslationUnit(ASTContext &Context) override {
      TraverseDecl(Context.getTranslationUnitDecl());
    }

    bool VisitNamedDecl(NamedDecl *D) override {
      DeclNames.push_back(D->getQualifiedNameAsString());
      return true;
    }

  private:
    std::vector<std::string> &DeclNames;
  };
};

class CASIncludeTreeTest : public ::testing::Test {
protected:
  std::shared_ptr<ObjectStore> DB = llvm::cas::createInMemoryCAS();
  std::shared_ptr<llvm::cas::ActionCache> Cache =
      llvm::cas::createInMemoryActionCache();
  SmallVector<IncludeTree::FileList::FileEntry> FileEntries;
  std::unique_ptr<TextDiagnosticBuffer> DiagsStorage;

  /// Store a file in the CAS and, unless it is the predefines buffer, add it to
  /// the file list.
  IncludeTree::File addFile(StringRef Name, StringRef Contents) {
    std::optional<ObjectRef> ContentsRef;
    EXPECT_THAT_ERROR(DB->storeFromString({}, Contents).moveInto(ContentsRef),
                      Succeeded());
    std::optional<IncludeTree::File> File;
    EXPECT_THAT_ERROR(
        IncludeTree::File::create(*DB, Name, *ContentsRef).moveInto(File),
        Succeeded());
    if (Name != "<built-in>")
      FileEntries.push_back(
          {File->getRef(),
           static_cast<IncludeTree::FileList::FileSizeTy>(Contents.size())});
    return *File;
  }

  IncludeTree makeTree(IncludeTree::File File,
                       ArrayRef<IncludeTree::IncludeInfo> Includes = {},
                       llvm::SmallBitVector Checks = {},
                       std::optional<StringRef> Submodule = std::nullopt) {
    std::optional<ObjectRef> SubmoduleName;
    if (Submodule)
      EXPECT_THAT_ERROR(
          DB->storeFromString({}, *Submodule).moveInto(SubmoduleName),
          Succeeded());
    std::optional<IncludeTree> Tree;
    EXPECT_THAT_ERROR(IncludeTree::create(*DB, SrcMgr::C_User, File.getRef(),
                                          Includes, SubmoduleName, Checks)
                          .moveInto(Tree),
                      Succeeded());
    return *Tree;
  }

  static IncludeTree::IncludeInfo include(const IncludeTree &Tree,
                                          uint32_t Offset) {
    return {Tree.getRef(), Offset, IncludeTree::NodeKind::Tree};
  }

  /// Create the root, with \p MainTree's first include being the predefines.
  /// \p FileLists are the file lists of the AST files that are imported.
  std::string makeRoot(const IncludeTree &MainTree,
                       std::optional<ObjectRef> PCH = std::nullopt,
                       ArrayRef<ObjectRef> FileLists = {},
                       std::optional<ObjectRef> ModuleMap = std::nullopt) {
    std::optional<IncludeTree::FileList> List;
    EXPECT_THAT_ERROR(IncludeTree::FileList::create(*DB, FileEntries, FileLists)
                          .moveInto(List),
                      Succeeded());
    FileEntries.clear();
    std::optional<IncludeTreeRoot> Root;
    EXPECT_THAT_ERROR(IncludeTreeRoot::create(*DB, MainTree.getRef(),
                                              List->getRef(), PCH, ModuleMap,
                                              std::nullopt)
                          .moveInto(Root),
                      Succeeded());
    return Root->getID().toString();
  }

  IncludeTree makePredefinesTree() {
    return makeTree(addFile("<built-in>", "#define FROM_PREDEFINES 1\n"));
  }

  /// Run \p Action on a compiler invocation created from \p Args, with the
  /// shared CAS and an empty base file system.
  bool run(ArrayRef<const char *> Args, FrontendAction &Action,
           TextDiagnosticBuffer *&Diags, bool ExpectErrors = false,
           IntrusiveRefCntPtr<llvm::vfs::FileSystem> BaseFS = nullptr,
           llvm::function_ref<void(CompilerInstance &)> Configure = nullptr) {
    auto Invocation = std::make_shared<CompilerInvocation>();
    DiagnosticOptions DiagOpts;
    DiagsStorage = std::make_unique<TextDiagnosticBuffer>();
    Diags = DiagsStorage.get();
    DiagnosticsEngine ParseDiags(DiagnosticIDs::create(), DiagOpts, Diags,
                                 /*ShouldOwnClient=*/false);
    if (!CompilerInvocation::CreateFromArgs(*Invocation, Args, ParseDiags))
      return false;

    CompilerInstance Compiler(std::move(Invocation));
    Compiler.setCASDatabases(DB, Cache);
    if (!BaseFS)
      BaseFS = llvm::makeIntrusiveRefCnt<llvm::vfs::InMemoryFileSystem>();
    Compiler.createVirtualFileSystem(std::move(BaseFS), Diags);
    Compiler.createDiagnostics(Diags, /*ShouldOwnClient=*/false);
    if (Configure)
      Configure(Compiler);
    bool Success = Compiler.ExecuteAction(Action);
    if (!ExpectErrors)
      for (auto I = Diags->err_begin(), E = Diags->err_end(); I != E; ++I)
        ADD_FAILURE() << "unexpected error: " << I->second;
    return Success;
  }
};

TEST_F(CASIncludeTreeTest, SyntaxOnly) {
  IncludeTree Predefines = makePredefinesTree();
  IncludeTree Header =
      makeTree(addFile("t.h", "#define HEADER_VALUE 42\nint in_header;\n"));
  StringRef MainContents = "#include \"t.h\"\n"
                           "#if __has_include(\"not_on_disk.h\")\n"
                           "int replayed_check;\n"
                           "#endif\n"
                           "int in_main = HEADER_VALUE + FROM_PREDEFINES;\n";
  llvm::SmallBitVector Checks(1);
  Checks.set(0);
  IncludeTree Main =
      makeTree(addFile("t.c", MainContents),
               {include(Predefines, 0),
                include(Header, StringRef("#include \"t.h\"\n").size())},
               Checks);
  std::string RootID = makeRoot(Main);

  const char *Args[] = {"-fcas-include-tree", RootID.c_str(), "-x", "c",
                        "-fsyntax-only"};
  CollectDeclsAction Action;
  TextDiagnosticBuffer *Diags;
  ASSERT_TRUE(run(Args, Action, Diags));
  EXPECT_EQ(Diags->getNumErrors(), 0u);
  EXPECT_EQ(Action.DeclNames, (std::vector<std::string>{
                                  "in_header", "replayed_check", "in_main"}));
}

TEST_F(CASIncludeTreeTest, PCH) {
  if (!llvm::cas::isOnDiskCASEnabled())
    GTEST_SKIP() << "needs a temporary directory for the PCH output";

  // Build a PCH from an include-tree.
  IncludeTree Predefines = makePredefinesTree();
  IncludeTree PCHMain =
      makeTree(addFile("p.h", "int from_pch;\n"), {include(Predefines, 0)});
  std::string PCHRootID = makeRoot(PCHMain);

  llvm::unittest::TempDir Dir("include-tree-pch", /*Unique=*/true);
  std::string PCHPath = Dir.path("p.pch").str().str();
  {
    const char *Args[] = {"-fcas-include-tree", PCHRootID.c_str(), "-x",
                          "c-header",           "-emit-pch",       "-o",
                          PCHPath.c_str()};
    GeneratePCHAction Action;
    TextDiagnosticBuffer *Diags;
    ASSERT_TRUE(run(Args, Action, Diags));
    EXPECT_EQ(Diags->getNumErrors(), 0u);
  }

  // The PCH records the include-tree it was built from.
  auto PCHBuffer = llvm::MemoryBuffer::getFile(PCHPath);
  ASSERT_TRUE(PCHBuffer);
  EXPECT_NE((*PCHBuffer)->getBuffer().find(PCHRootID), StringRef::npos);

  // Use the PCH from another include-tree.
  std::optional<ObjectRef> PCHContents;
  ASSERT_THAT_ERROR(
      DB->storeFromString({}, (*PCHBuffer)->getBuffer()).moveInto(PCHContents),
      Succeeded());
  std::optional<IncludeTree::File> PCHFile;
  ASSERT_THAT_ERROR(
      IncludeTree::File::create(*DB, "p.pch", *PCHContents).moveInto(PCHFile),
      Succeeded());

  IncludeTree TUPredefines = makePredefinesTree();
  IncludeTree TUMain =
      makeTree(addFile("t.c", "int use_pch = sizeof(from_pch);\n"),
               {include(TUPredefines, 0)});
  // The PCH refers to its input files, so the file list includes the PCH's.
  std::optional<CASID> PCHRootCASID;
  ASSERT_THAT_ERROR(DB->parseID(PCHRootID).moveInto(PCHRootCASID), Succeeded());
  std::optional<IncludeTreeRoot> PCHRoot;
  ASSERT_THAT_ERROR(IncludeTreeRoot::get(*DB, *DB->getReference(*PCHRootCASID))
                        .moveInto(PCHRoot),
                    Succeeded());
  std::string TURootID =
      makeRoot(TUMain, PCHFile->getRef(), PCHRoot->getFileListRef());

  const char *Args[] = {"-fcas-include-tree", TURootID.c_str(), "-x", "c",
                        "-fsyntax-only"};
  CollectDeclsAction Action;
  TextDiagnosticBuffer *Diags;
  ASSERT_TRUE(run(Args, Action, Diags));
  EXPECT_EQ(Diags->getNumErrors(), 0u);
  EXPECT_TRUE(llvm::is_contained(Action.DeclNames, "use_pch"));
}

TEST_F(CASIncludeTreeTest, Module) {
  if (!llvm::cas::isOnDiskCASEnabled())
    GTEST_SKIP() << "needs a temporary directory for the module output";

  // Build module 'Mod' with submodule 'Mod.Sub' from an include-tree. The
  // include-tree provides the module map and the module's input buffer.
  IncludeTree Predefines = makePredefinesTree();
  IncludeTree SubHeader = makeTree(addFile("Sub.h", "int from_submodule;\n"),
                                   {}, {}, StringRef("Mod.Sub"));
  StringRef ModuleIncludes = "#include \"Sub.h\"\n";
  IncludeTree ModMain = makeTree(
      addFile(Module::getModuleInputBufferName(), ModuleIncludes),
      {include(Predefines, 0), include(SubHeader, ModuleIncludes.size())});

  std::optional<IncludeTree::Module> Sub, Mod;
  ASSERT_THAT_ERROR(IncludeTree::Module::create(
                        *DB, "Sub", "", IncludeTree::Module::ModuleFlags(), {},
                        std::nullopt, std::nullopt, std::nullopt)
                        .moveInto(Sub),
                    Succeeded());
  ASSERT_THAT_ERROR(IncludeTree::Module::create(
                        *DB, "Mod", "", IncludeTree::Module::ModuleFlags(),
                        {Sub->getRef()}, std::nullopt, std::nullopt,
                        std::nullopt)
                        .moveInto(Mod),
                    Succeeded());
  std::optional<IncludeTree::ModuleMap> MM;
  ASSERT_THAT_ERROR(
      IncludeTree::ModuleMap::create(*DB, {Mod->getRef()}).moveInto(MM),
      Succeeded());
  std::string ModRootID = makeRoot(ModMain, std::nullopt, {}, MM->getRef());

  llvm::unittest::TempDir Dir("include-tree-module", /*Unique=*/true);
  std::string PCMPath = Dir.path("Mod.pcm").str().str();
  {
    const char *Args[] = {
        "-fcas-include-tree", ModRootID.c_str(),   "-x",           "c",
        "-fmodules",          "-fmodule-name=Mod", "-emit-module", "-o",
        PCMPath.c_str()};
    GenerateModuleFromModuleMapAction Action;
    TextDiagnosticBuffer *Diags;
    ASSERT_TRUE(run(Args, Action, Diags));
    EXPECT_EQ(Diags->getNumErrors(), 0u);
  }

  // Import the submodule from another include-tree. The include-tree file
  // system provides only the files in the include-tree, so the module file is
  // part of the file list. The module refers to its input files, so the file
  // list also includes the module's.
  auto PCMBuffer = llvm::MemoryBuffer::getFile(PCMPath);
  ASSERT_TRUE(PCMBuffer);
  addFile(PCMPath, (*PCMBuffer)->getBuffer());
  std::optional<CASID> ModRootCASID;
  ASSERT_THAT_ERROR(DB->parseID(ModRootID).moveInto(ModRootCASID), Succeeded());
  std::optional<IncludeTreeRoot> ModRoot;
  ASSERT_THAT_ERROR(IncludeTreeRoot::get(*DB, *DB->getReference(*ModRootCASID))
                        .moveInto(ModRoot),
                    Succeeded());

  std::optional<IncludeTree::ModuleImport> Import;
  ASSERT_THAT_ERROR(IncludeTree::ModuleImport::create(*DB, "Mod.Sub",
                                                      /*VisibilityOnly=*/false)
                        .moveInto(Import),
                    Succeeded());
  IncludeTree TUPredefines = makePredefinesTree();
  StringRef TUIncludes = "#include \"Sub.h\"\n";
  IncludeTree TUMain = makeTree(
      addFile("t.c",
              (TUIncludes + "int use = sizeof(from_submodule);\n").str()),
      {include(TUPredefines, 0),
       {Import->getRef(), static_cast<uint32_t>(TUIncludes.size()),
        IncludeTree::NodeKind::ModuleImport}});
  std::string TURootID =
      makeRoot(TUMain, std::nullopt, ModRoot->getFileListRef());

  std::string ModuleFileArg = "-fmodule-file=Mod=" + PCMPath;
  const char *Args[] = {
      "-fcas-include-tree", TURootID.c_str(),     "-x", "c", "-fmodules",
      "-fsyntax-only",      ModuleFileArg.c_str()};
  CollectDeclsAction Action;
  TextDiagnosticBuffer *Diags;
  ASSERT_TRUE(run(Args, Action, Diags));
  EXPECT_EQ(Diags->getNumErrors(), 0u);
  EXPECT_TRUE(llvm::is_contained(Action.DeclNames, "use"));
}

TEST_F(CASIncludeTreeTest, ModuleFileCacheKey) {
  if (!llvm::cas::isOnDiskCASEnabled())
    GTEST_SKIP() << "needs a temporary directory for the module outputs";

  llvm::unittest::TempDir Dir("include-tree-module-cache-key",
                              /*Unique=*/true);

  auto makeKey = [&](StringRef Name) {
    std::optional<ObjectRef> Ref;
    EXPECT_THAT_ERROR(DB->storeFromString({}, Name).moveInto(Ref), Succeeded());
    return DB->getID(*Ref);
  };

  // Store the module file at \p PCMPath as the main output of a compile job
  // with cache key \p Key, as -fcache-compile-job would have done.
  auto cacheModule = [&](StringRef PCMPath, const CASID &Key) {
    auto Buffer = llvm::MemoryBuffer::getFile(PCMPath);
    ASSERT_TRUE(Buffer);
    std::optional<ObjectRef> PCM;
    ASSERT_THAT_ERROR(
        DB->storeFromString({}, (*Buffer)->getBuffer()).moveInto(PCM),
        Succeeded());
    CompileJobCacheResult::Builder Builder;
    Builder.addOutput(CompileJobCacheResult::OutputKind::MainOutput, *PCM);
    std::optional<ObjectRef> Result;
    ASSERT_THAT_ERROR(Builder.build(*DB).moveInto(Result), Succeeded());
    ASSERT_THAT_ERROR(Cache->put(Key, DB->getID(*Result)), Succeeded());
  };

  auto makeModuleMap = [&](StringRef Name) {
    std::optional<IncludeTree::Module> Mod;
    EXPECT_THAT_ERROR(IncludeTree::Module::create(
                          *DB, Name, "", IncludeTree::Module::ModuleFlags(), {},
                          std::nullopt, std::nullopt, std::nullopt)
                          .moveInto(Mod),
                      Succeeded());
    std::optional<IncludeTree::ModuleMap> MM;
    EXPECT_THAT_ERROR(
        IncludeTree::ModuleMap::create(*DB, {Mod->getRef()}).moveInto(MM),
        Succeeded());
    return MM->getRef();
  };

  auto getFileList = [&](StringRef RootID) {
    std::optional<CASID> ID;
    EXPECT_THAT_ERROR(DB->parseID(RootID).moveInto(ID), Succeeded());
    std::optional<IncludeTreeRoot> Root;
    EXPECT_THAT_ERROR(
        IncludeTreeRoot::get(*DB, *DB->getReference(*ID)).moveInto(Root),
        Succeeded());
    return Root->getFileListRef();
  };

  auto makeImport = [&](StringRef Name, uint32_t Offset) {
    std::optional<IncludeTree::ModuleImport> Import;
    EXPECT_THAT_ERROR(
        IncludeTree::ModuleImport::create(*DB, Name,
                                          /*VisibilityOnly=*/false)
            .moveInto(Import),
        Succeeded());
    return IncludeTree::IncludeInfo{Import->getRef(), Offset,
                                    IncludeTree::NodeKind::ModuleImport};
  };

  // Build module 'A' as a cached compile job. The module file records its
  // cache key.
  CASID KeyA = makeKey("key A");
  std::string PCMPathA = Dir.path("A.pcm").str().str();
  std::string RootA;
  {
    IncludeTree Predefines = makePredefinesTree();
    IncludeTree Header =
        makeTree(addFile("A.h", "int from_a;\n"), {}, {}, StringRef("A"));
    StringRef Includes = "#include \"A.h\"\n";
    IncludeTree Main =
        makeTree(addFile(Module::getModuleInputBufferName(), Includes),
                 {include(Predefines, 0), include(Header, Includes.size())});
    RootA = makeRoot(Main, std::nullopt, {}, makeModuleMap("A"));
    const char *Args[] = {
        "-fcas-include-tree", RootA.c_str(),     "-x",           "c",
        "-fmodules",          "-fmodule-name=A", "-emit-module", "-o",
        PCMPathA.c_str()};
    GenerateModuleFromModuleMapAction Action;
    TextDiagnosticBuffer *Diags;
    ASSERT_TRUE(
        run(Args, Action, Diags, /*ExpectErrors=*/false, nullptr,
            [&](CompilerInstance &CI) { CI.setCompileJobCacheKey(KeyA); }));
  }
  cacheModule(PCMPathA, KeyA);

  // Build module 'B', which imports 'A' through its cache key. The include-tree
  // file system cannot see the module file on disk.
  CASID KeyB = makeKey("key B");
  std::string PCMPathB = Dir.path("B.pcm").str().str();
  std::string RootB;
  {
    IncludeTree Predefines = makePredefinesTree();
    StringRef HeaderIncludes = "#include \"A.h\"\n";
    IncludeTree Header =
        makeTree(addFile("B.h", (HeaderIncludes + "int from_b;\n").str()),
                 {makeImport("A", HeaderIncludes.size())}, {}, StringRef("B"));
    StringRef Includes = "#include \"B.h\"\n";
    IncludeTree Main =
        makeTree(addFile(Module::getModuleInputBufferName(), Includes),
                 {include(Predefines, 0), include(Header, Includes.size())});
    RootB =
        makeRoot(Main, std::nullopt, {getFileList(RootA)}, makeModuleMap("B"));
    std::string KeyAStr = KeyA.toString();
    std::string ModuleFileA = "-fmodule-file=A=" + PCMPathA;
    const char *Args[] = {"-fcas-include-tree",
                          RootB.c_str(),
                          "-x",
                          "c",
                          "-fmodules",
                          "-fmodule-name=B",
                          "-emit-module",
                          "-fmodule-file-cache-key",
                          PCMPathA.c_str(),
                          KeyAStr.c_str(),
                          ModuleFileA.c_str(),
                          "-o",
                          PCMPathB.c_str()};
    GenerateModuleFromModuleMapAction Action;
    TextDiagnosticBuffer *Diags;
    ASSERT_TRUE(
        run(Args, Action, Diags, /*ExpectErrors=*/false, nullptr,
            [&](CompilerInstance &CI) { CI.setCompileJobCacheKey(KeyB); }));
  }
  cacheModule(PCMPathB, KeyB);

  // Import 'B' through its cache key. 'A' is found through the cache key that
  // 'B' recorded for its import.
  IncludeTree Predefines = makePredefinesTree();
  StringRef Includes = "#include \"B.h\"\n";
  IncludeTree Main =
      makeTree(addFile("t.c", (Includes + "int use = sizeof(from_b);\n").str()),
               {include(Predefines, 0), makeImport("B", Includes.size())});
  std::string RootTU = makeRoot(Main, std::nullopt, {getFileList(RootB)});
  std::string KeyBStr = KeyB.toString();
  std::string ModuleFileB = "-fmodule-file=B=" + PCMPathB;
  const char *Args[] = {"-fcas-include-tree",
                        RootTU.c_str(),
                        "-x",
                        "c",
                        "-fmodules",
                        "-fsyntax-only",
                        "-fmodule-file-cache-key",
                        PCMPathB.c_str(),
                        KeyBStr.c_str(),
                        ModuleFileB.c_str()};
  CollectDeclsAction Action;
  TextDiagnosticBuffer *Diags;
  ASSERT_TRUE(run(Args, Action, Diags));
  EXPECT_TRUE(llvm::is_contained(Action.DeclNames, "use"));
}

TEST_F(CASIncludeTreeTest, InvalidRootID) {
  const char *Args[] = {"-fcas-include-tree", "not-a-cas-id", "-x", "c",
                        "-fsyntax-only"};
  CollectDeclsAction Action;
  TextDiagnosticBuffer *Diags;
  EXPECT_FALSE(run(Args, Action, Diags, /*ExpectErrors=*/true));
  ASSERT_GE(Diags->getNumErrors(), 1u);
  EXPECT_NE(Diags->err_begin()->second.find("CAS cannot parse root-id"),
            std::string::npos);
}

TEST_F(CASIncludeTreeTest, IncompatibleArguments) {
  auto parse = [](ArrayRef<const char *> Args) {
    CompilerInvocation Invocation;
    DiagnosticOptions DiagOpts;
    auto *Diags = new TextDiagnosticBuffer();
    DiagnosticsEngine Engine(DiagnosticIDs::create(), DiagOpts, Diags);
    CompilerInvocation::CreateFromArgs(Invocation, Args, Engine);
    std::vector<std::string> Errors;
    for (auto I = Diags->err_begin(), E = Diags->err_end(); I != E; ++I)
      Errors.push_back(I->second);
    return Errors;
  };

  EXPECT_EQ(parse({"-fcas-include-tree", "id", "-x", "c", "t.c"}),
            std::vector<std::string>{
                "passing input files is incompatible with "
                "'-fcas-include-tree' and '-fcas-input-file-cache-key'"});
  EXPECT_EQ(parse({"-fcas-include-tree", "id"}),
            std::vector<std::string>{
                "missing '-x' input kind when using '-fcas-include-tree'"});
  EXPECT_EQ(parse({"-fcas-include-tree", "id", "-x", "c", "-fapinotes"}),
            std::vector<std::string>{"passing incompatible option "
                                     "'-fapinotes' with '-fcas-include-tree'"});
}

} // anonymous namespace
