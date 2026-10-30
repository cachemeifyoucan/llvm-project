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
                       llvm::SmallBitVector Checks = {}) {
    std::optional<IncludeTree> Tree;
    EXPECT_THAT_ERROR(IncludeTree::create(*DB, SrcMgr::C_User, File.getRef(),
                                          Includes, std::nullopt, Checks)
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
                       ArrayRef<ObjectRef> FileLists = {}) {
    std::optional<IncludeTree::FileList> List;
    EXPECT_THAT_ERROR(IncludeTree::FileList::create(*DB, FileEntries, FileLists)
                          .moveInto(List),
                      Succeeded());
    FileEntries.clear();
    std::optional<IncludeTreeRoot> Root;
    EXPECT_THAT_ERROR(IncludeTreeRoot::create(*DB, MainTree.getRef(),
                                              List->getRef(), PCH, std::nullopt,
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
           TextDiagnosticBuffer *&Diags, bool ExpectErrors = false) {
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
    Compiler.createVirtualFileSystem(
        llvm::makeIntrusiveRefCnt<llvm::vfs::InMemoryFileSystem>(), Diags);
    Compiler.createDiagnostics(Diags, /*ShouldOwnClient=*/false);
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
