//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "clang/CAS/IncludeTree.h"
#include "clang/DependencyScanning/ScanAndUpdateArgs.h"
#include "clang/Frontend/TextDiagnosticBuffer.h"
#include "clang/Tooling/DependencyScanningTool.h"
#include "llvm/ADT/SmallBitVector.h"
#include "llvm/CAS/ObjectStore.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/VirtualFileSystem.h"
#include "llvm/Testing/Support/Error.h"
#include "gtest/gtest.h"
#include <system_error>

using namespace clang;
using namespace clang::cas;
using namespace clang::dependencies;
using namespace clang::tooling;

TEST(IncludeTree, IncludeTreeAndRoot) {
  std::shared_ptr<ObjectStore> DB = llvm::cas::createInMemoryCAS();

  auto MakeFile = [&](StringRef Path,
                      StringRef Contents) -> Expected<IncludeTree::File> {
    std::optional<ObjectRef> ContentsRef;
    if (llvm::Error E = DB->storeFromString({}, Contents).moveInto(ContentsRef))
      return std::move(E);
    return IncludeTree::File::create(*DB, Path, *ContentsRef);
  };

  std::optional<IncludeTree::File> MainFile, HeaderFile, PCHFile;
  ASSERT_THAT_ERROR(
      MakeFile("t.c", "#include \"t.h\"\nint x;\n").moveInto(MainFile),
      llvm::Succeeded());
  ASSERT_THAT_ERROR(MakeFile("t.h", "int y;\n").moveInto(HeaderFile),
                    llvm::Succeeded());
  ASSERT_THAT_ERROR(MakeFile("t.pch", "pch").moveInto(PCHFile),
                    llvm::Succeeded());

  std::optional<IncludeTree> HeaderTree;
  ASSERT_THAT_ERROR(IncludeTree::create(*DB, SrcMgr::C_System,
                                        HeaderFile->getRef(), {},
                                        llvm::SmallBitVector())
                        .moveInto(HeaderTree),
                    llvm::Succeeded());
  EXPECT_EQ(HeaderTree->getFileCharacteristic(), SrcMgr::C_System);
  EXPECT_FALSE(HeaderTree->isSubmodule());
  EXPECT_EQ(HeaderTree->getNumIncludes(), 0u);

  // Two __has_include checks, the first false and the second true.
  llvm::SmallBitVector Checks(2);
  Checks.set(1);
  IncludeTree::IncludeInfo Include{HeaderTree->getRef(), /*Offset=*/15,
                                   IncludeTree::NodeKind::Tree};
  std::optional<IncludeTree> MainTree;
  ASSERT_THAT_ERROR(IncludeTree::create(*DB, SrcMgr::C_User, MainFile->getRef(),
                                        Include, Checks)
                        .moveInto(MainTree),
                    llvm::Succeeded());
  EXPECT_EQ(MainTree->getFileCharacteristic(), SrcMgr::C_User);
  EXPECT_EQ(MainTree->getBaseFileRef(), MainFile->getRef());
  ASSERT_EQ(MainTree->getNumIncludes(), 1u);
  EXPECT_EQ(MainTree->getIncludeKind(0), IncludeTree::NodeKind::Tree);
  EXPECT_EQ(MainTree->getIncludeOffset(0), 15u);
  EXPECT_EQ(MainTree->getIncludeRef(0), HeaderTree->getRef());
  EXPECT_FALSE(MainTree->getCheckResult(0));
  EXPECT_TRUE(MainTree->getCheckResult(1));

  std::optional<IncludeTree::FileInfo> Info;
  ASSERT_THAT_ERROR(MainTree->getBaseFileInfo().moveInto(Info),
                    llvm::Succeeded());
  EXPECT_EQ(Info->Filename, "t.c");
  EXPECT_EQ(Info->Contents, "#include \"t.h\"\nint x;\n");

  // Round-trip the tree through its reference.
  std::optional<IncludeTree> Loaded;
  ASSERT_THAT_ERROR(IncludeTree::get(*DB, MainTree->getRef()).moveInto(Loaded),
                    llvm::Succeeded());
  EXPECT_EQ(Loaded->getRef(), MainTree->getRef());
  EXPECT_THAT_ERROR(IncludeTree::get(*DB, MainFile->getRef()).takeError(),
                    llvm::Failed());

  std::optional<IncludeTree::FileList> List;
  IncludeTree::FileList::FileEntry Entries[] = {{MainFile->getRef(), 22},
                                                {HeaderFile->getRef(), 7}};
  ASSERT_THAT_ERROR(
      IncludeTree::FileList::create(*DB, Entries, {}).moveInto(List),
      llvm::Succeeded());

  // A root without a PCH.
  std::optional<IncludeTreeRoot> Root;
  ASSERT_THAT_ERROR(IncludeTreeRoot::create(*DB, MainTree->getRef(),
                                            List->getRef(), std::nullopt)
                        .moveInto(Root),
                    llvm::Succeeded());
  EXPECT_EQ(Root->getMainFileTreeRef(), MainTree->getRef());
  EXPECT_EQ(Root->getFileListRef(), List->getRef());
  EXPECT_EQ(Root->getPCHRef(), std::nullopt);

  // A root with a PCH.
  std::optional<IncludeTreeRoot> PCHRoot;
  ASSERT_THAT_ERROR(IncludeTreeRoot::create(*DB, MainTree->getRef(),
                                            List->getRef(), PCHFile->getRef())
                        .moveInto(PCHRoot),
                    llvm::Succeeded());
  EXPECT_EQ(PCHRoot->getPCHRef(), PCHFile->getRef());
  std::optional<std::optional<IncludeTree::File>> PCH;
  ASSERT_THAT_ERROR(PCHRoot->getPCH().moveInto(PCH), llvm::Succeeded());
  ASSERT_TRUE(*PCH);
  EXPECT_EQ((*PCH)->getRef(), PCHFile->getRef());

  std::optional<IncludeTreeRoot> LoadedRoot;
  ASSERT_THAT_ERROR(
      IncludeTreeRoot::get(*DB, PCHRoot->getRef()).moveInto(LoadedRoot),
      llvm::Succeeded());
  EXPECT_EQ(LoadedRoot->getPCHRef(), PCHFile->getRef());
  EXPECT_THAT_ERROR(IncludeTreeRoot::get(*DB, MainTree->getRef()).takeError(),
                    llvm::Failed());

  std::string Printed;
  llvm::raw_string_ostream OS(Printed);
  ASSERT_THAT_ERROR(PCHRoot->print(OS), llvm::Succeeded());
  StringRef P(Printed);
  EXPECT_TRUE(P.consume_front("(PCH) t.pch llvmcas://")) << P;
  P = P.drop_front(64);
  EXPECT_TRUE(P.consume_front("\nt.c llvmcas://")) << P;
  P = P.drop_front(64);
  EXPECT_TRUE(P.consume_front("\n2:1 t.h llvmcas://")) << P;
  P = P.drop_front(64);
  EXPECT_TRUE(P.consume_front("\nFiles:\nt.c llvmcas://")) << P;
  P = P.drop_front(64);
  EXPECT_TRUE(P.consume_front("\nt.h llvmcas://")) << P;
  P = P.drop_front(64);
  EXPECT_EQ(P, "\n");
}

TEST(IncludeTree, IncludeTreeScan) {
  StringRef PathSep = llvm::sys::path::get_separator();
  CASOptions CASOpts;
  auto [DB, Cache] =
      llvm::cantFail(CASOpts.CASConfiguration::createDatabases());
  auto FS = llvm::makeIntrusiveRefCnt<llvm::vfs::InMemoryFileSystem>();
  FS->setCurrentWorkingDirectory("/root");
  auto add = [&](StringRef Path, StringRef Contents) {
    FS->addFile(Path, 0, llvm::MemoryBuffer::getMemBuffer(Contents));
  };
  StringRef MainContents = R"(
    #include "a1.h"
    #include "sys.h"
    #include "sys_directive.h"
  )";
  StringRef A1Contents = R"(
    #if __has_include("other.h")
      #include "other.h"
    #endif
    #if __has_include("b1.h")
      #include "b1.h"
    #endif
  )";
  StringRef SysDirectiveContents = R"(
    #pragma clang system_header
    #include "sys_indirect.h"
  )";
  add("t.cpp", MainContents);
  add("a1.h", A1Contents);
  add("b1.h", "");
  add("sys/sys.h", "");
  add("sys_directive.h", SysDirectiveContents);
  add("sys_indirect.h", "");

  DependencyScanningServiceOptions Opts;
  Opts.MakeVFS = [FS] { return FS; };
  Opts.Compilation = IncludeTreeCompilation{CASOpts, DB, Cache};
  DependencyScanningService Service(std::move(Opts));
  DependencyScanningTool ScanTool(Service);

  TextDiagnosticBuffer DiagConsumer;

  std::vector<std::string> CommandLine = {"clang",
                                          "-target",
                                          "x86_64-apple-macos11",
                                          "-isystem",
                                          "sys",
                                          "-c",
                                          "t.cpp",
                                          "-o"
                                          "t.cpp.o"};
  std::optional<IncludeTreeRoot> Root;
  Expected<IncludeTreeRoot> ExpectedRoot =
      ScanTool.getIncludeTree(CommandLine, /*CWD*/ "", nullptr, DiagConsumer);
  ASSERT_THAT_ERROR(std::move(ExpectedRoot).moveInto(Root), llvm::Succeeded());

  std::optional<IncludeTree::File> MainFile;
  std::optional<IncludeTree::File> A1File;
  std::optional<IncludeTree::File> B1File;
  std::optional<IncludeTree::File> SysFile;
  std::optional<IncludeTree::File> SysDirectiveFile;
  std::optional<IncludeTree::File> SysIndirectFile;

  std::optional<IncludeTree> Main;
  ASSERT_THAT_ERROR(Root->getMainFileTree().moveInto(Main), llvm::Succeeded());
  {
    ASSERT_THAT_ERROR(Main->getBaseFile().moveInto(MainFile),
                      llvm::Succeeded());
    EXPECT_EQ(Main->getFileCharacteristic(), SrcMgr::C_User);
    IncludeTree::FileInfo FI;
    ASSERT_THAT_ERROR(MainFile->getFileInfo().moveInto(FI), llvm::Succeeded());
    EXPECT_EQ(FI.Filename, "t.cpp");
    EXPECT_EQ(FI.Contents, MainContents);
  }
  ASSERT_EQ(Main->getNumIncludes(), uint32_t(4));

  std::optional<IncludeTree> Predef;
  ASSERT_THAT_ERROR(Main->getIncludeTree(0).moveInto(Predef),
                    llvm::Succeeded());
  EXPECT_EQ(Main->getIncludeOffset(0), uint32_t(0));
  {
    EXPECT_EQ(Predef->getFileCharacteristic(), SrcMgr::C_User);
    IncludeTree::FileInfo FI;
    ASSERT_THAT_ERROR(Predef->getBaseFileInfo().moveInto(FI),
                      llvm::Succeeded());
    EXPECT_EQ(FI.Filename, "<built-in>");
  }

  std::optional<IncludeTree> A1;
  ASSERT_THAT_ERROR(Main->getIncludeTree(1).moveInto(A1), llvm::Succeeded());
  EXPECT_EQ(Main->getIncludeOffset(1), uint32_t(21));
  {
    ASSERT_THAT_ERROR(A1->getBaseFile().moveInto(A1File), llvm::Succeeded());
    EXPECT_EQ(A1->getFileCharacteristic(), SrcMgr::C_User);
    IncludeTree::FileInfo FI;
    ASSERT_THAT_ERROR(A1File->getFileInfo().moveInto(FI), llvm::Succeeded());
    EXPECT_EQ(FI.Filename, "." + PathSep.str() + "a1.h");
    EXPECT_EQ(FI.Contents, A1Contents);
    EXPECT_FALSE(A1->getCheckResult(0));
    EXPECT_TRUE(A1->getCheckResult(1));

    ASSERT_EQ(A1->getNumIncludes(), uint32_t(1));
    std::optional<IncludeTree> B1;
    ASSERT_THAT_ERROR(A1->getIncludeTree(0).moveInto(B1), llvm::Succeeded());
    EXPECT_EQ(A1->getIncludeOffset(0), uint32_t(122));
    {
      ASSERT_THAT_ERROR(B1->getBaseFile().moveInto(B1File), llvm::Succeeded());
      EXPECT_EQ(B1->getFileCharacteristic(), SrcMgr::C_User);
      IncludeTree::FileInfo FI;
      ASSERT_THAT_ERROR(B1->getBaseFileInfo().moveInto(FI), llvm::Succeeded());
      EXPECT_EQ(FI.Filename, "." + PathSep.str() + "b1.h");
      EXPECT_EQ(FI.Contents, "");

      ASSERT_EQ(B1->getNumIncludes(), uint32_t(0));
    }
  }

  std::optional<IncludeTree> Sys;
  ASSERT_THAT_ERROR(Main->getIncludeTree(2).moveInto(Sys), llvm::Succeeded());
  EXPECT_EQ(Main->getIncludeOffset(2), uint32_t(42));
  {
    ASSERT_THAT_ERROR(Sys->getBaseFile().moveInto(SysFile), llvm::Succeeded());
    EXPECT_EQ(Sys->getFileCharacteristic(), SrcMgr::C_System);
    IncludeTree::FileInfo FI;
    ASSERT_THAT_ERROR(Sys->getBaseFileInfo().moveInto(FI), llvm::Succeeded());
    EXPECT_EQ(FI.Filename, "sys" + PathSep.str() + "sys.h");
    EXPECT_EQ(FI.Contents, "");

    ASSERT_EQ(Sys->getNumIncludes(), uint32_t(0));
  }

  std::optional<IncludeTree> SysDirective;
  ASSERT_THAT_ERROR(Main->getIncludeTree(3).moveInto(SysDirective),
                    llvm::Succeeded());
  EXPECT_EQ(Main->getIncludeOffset(3), uint32_t(73));
  {
    ASSERT_THAT_ERROR(SysDirective->getBaseFile().moveInto(SysDirectiveFile),
                      llvm::Succeeded());
    // Note: system_header directive injects a line directive, so C_User is for
    // the start of the file here.
    EXPECT_EQ(SysDirective->getFileCharacteristic(), SrcMgr::C_User);
    ASSERT_EQ(SysDirective->getNumIncludes(), uint32_t(1));
    std::optional<IncludeTree> SysIndirect;
    ASSERT_THAT_ERROR(SysDirective->getIncludeTree(0).moveInto(SysIndirect),
                      llvm::Succeeded());
    {
      ASSERT_THAT_ERROR(SysIndirect->getBaseFile().moveInto(SysIndirectFile),
                        llvm::Succeeded());
      EXPECT_EQ(SysIndirect->getFileCharacteristic(), SrcMgr::C_System);
      ASSERT_EQ(SysIndirect->getNumIncludes(), uint32_t(0));
    }
  }

  std::optional<IncludeTree::FileList> FileList;
  ASSERT_THAT_ERROR(Root->getFileList().moveInto(FileList), llvm::Succeeded());

  SmallVector<std::pair<IncludeTree::File, IncludeTree::FileList::FileSizeTy>>
      Files;
  ASSERT_THAT_ERROR(FileList->forEachFile([&](auto F, auto S) -> llvm::Error {
    Files.push_back({F, S});
    return llvm::Error::success();
  }),
                    llvm::Succeeded());

  ASSERT_EQ(Files.size(), size_t(6));
  EXPECT_EQ(Files[0].first.getRef(), MainFile->getRef());
  EXPECT_EQ(Files[0].second, MainContents.size());
  EXPECT_EQ(Files[1].first.getRef(), A1File->getRef());
  EXPECT_EQ(Files[1].second, A1Contents.size());
  EXPECT_EQ(Files[2].first.getRef(), B1File->getRef());
  EXPECT_EQ(Files[2].second, IncludeTree::FileList::FileSizeTy(0));
  EXPECT_EQ(Files[3].first.getRef(), SysFile->getRef());
  EXPECT_EQ(Files[3].second, IncludeTree::FileList::FileSizeTy(0));
  EXPECT_EQ(Files[4].first.getRef(), SysDirectiveFile->getRef());
  EXPECT_EQ(Files[4].second, SysDirectiveContents.size());
  EXPECT_EQ(Files[5].first.getRef(), SysIndirectFile->getRef());
  EXPECT_EQ(Files[5].second, IncludeTree::FileList::FileSizeTy(0));
}

TEST(IncludeTree, IncludeTreeScanLocalFS) {
  CASOptions CASOpts;
  auto [DB, Cache] =
      llvm::cantFail(CASOpts.CASConfiguration::createDatabases());
  auto FS = llvm::makeIntrusiveRefCnt<llvm::vfs::InMemoryFileSystem>();
  FS->setCurrentWorkingDirectory("/root");
  FS->addFile("t.c", 0, llvm::MemoryBuffer::getMemBuffer("#include \"l.h\"\n"));
  // The header is only visible to this worker.
  auto LocalFS = llvm::makeIntrusiveRefCnt<llvm::vfs::InMemoryFileSystem>();
  LocalFS->setCurrentWorkingDirectory("/root");
  LocalFS->addFile("l.h", 0, llvm::MemoryBuffer::getMemBuffer("int l;\n"));

  DependencyScanningServiceOptions Opts;
  Opts.MakeVFS = [FS] { return FS; };
  Opts.Compilation = IncludeTreeCompilation{CASOpts, DB, Cache};
  DependencyScanningService Service(std::move(Opts));
  DependencyScanningTool ScanTool(Service, LocalFS);

  TextDiagnosticBuffer DiagConsumer;
  std::vector<std::string> CommandLine = {
      "clang", "-target", "x86_64-apple-macos11", "-fsyntax-only", "t.c"};
  std::optional<IncludeTreeRoot> Root;
  ASSERT_THAT_ERROR(
      ScanTool
          .getIncludeTree(CommandLine, /*CWD*/ "/root", nullptr, DiagConsumer)
          .moveInto(Root),
      llvm::Succeeded());

  std::optional<IncludeTree> Main;
  ASSERT_THAT_ERROR(Root->getMainFileTree().moveInto(Main), llvm::Succeeded());
  // The predefines buffer and l.h.
  ASSERT_EQ(Main->getNumIncludes(), uint32_t(2));
  std::optional<IncludeTree> Local;
  ASSERT_THAT_ERROR(Main->getIncludeTree(1).moveInto(Local), llvm::Succeeded());
  IncludeTree::FileInfo FI;
  ASSERT_THAT_ERROR(Local->getBaseFileInfo().moveInto(FI), llvm::Succeeded());
  EXPECT_EQ(FI.Contents, "int l;\n");
}

TEST(IncludeTree, IncludeTreeFileList) {
  std::shared_ptr<ObjectStore> DB = llvm::cas::createInMemoryCAS();
  SmallVector<IncludeTree::File> Files;
  for (unsigned I = 0; I < 10; ++I) {
    std::optional<IncludeTree::File> File;
    std::string Path = "/file" + std::to_string(I);
    static constexpr StringRef Bytes = "123456789";
    std::optional<ObjectRef> Content;
    ASSERT_THAT_ERROR(
        DB->storeFromString({}, Bytes.substr(0, I)).moveInto(Content),
        llvm::Succeeded());
    ASSERT_THAT_ERROR(
        IncludeTree::File::create(*DB, Path, *Content).moveInto(File),
        llvm::Succeeded());
    Files.push_back(std::move(*File));
  }

  auto MakeFileList = [&](unsigned Begin, unsigned End,
                          ArrayRef<ObjectRef> Lists) {
    SmallVector<IncludeTree::FileList::FileEntry> Entries;
    for (; Begin != End; ++Begin)
      Entries.push_back({Files[Begin].getRef(), Begin});
    return IncludeTree::FileList::create(*DB, Entries, Lists);
  };

  std::optional<IncludeTree::FileList> L89, L7, L29, L;
  ASSERT_THAT_ERROR(MakeFileList(8, 10, {}).moveInto(L89), llvm::Succeeded());
  EXPECT_EQ(L89->getNumReferences(), 2u);
  ASSERT_THAT_ERROR(MakeFileList(7, 8, {}).moveInto(L7), llvm::Succeeded());
  EXPECT_EQ(L7->getNumReferences(), 1u);
  ASSERT_THAT_ERROR(
      MakeFileList(2, 7, {L7->getRef(), L89->getRef()}).moveInto(L29),
      llvm::Succeeded());
  EXPECT_EQ(L29->getNumReferences(), 7u); // 2,3,4,5,6, {7}, {8, 9}
  ASSERT_THAT_ERROR(MakeFileList(0, 2, {L29->getRef()}).moveInto(L),
                    llvm::Succeeded());
  EXPECT_EQ(L->getNumReferences(), 3u); // 0, 1, {2, ...}

  size_t I = 0;
  ASSERT_THAT_ERROR(
      L->forEachFile([&](IncludeTree::File F, auto Size) -> llvm::Error {
        if (I >= Files.size())
          return llvm::Error::success(); // diagnosed later.
        EXPECT_EQ(F.getFilenameRef(), Files[I].getFilenameRef())
            << "filename mismatch at " << I;
        EXPECT_EQ(F.getContentsRef(), Files[I].getContentsRef())
            << "contents mismatch at " << I;
        EXPECT_EQ(Size, I) << "size mismatch at " << I;
        I += 1;
        return llvm::Error::success();
      }),
      llvm::Succeeded());
  EXPECT_EQ(I, Files.size());
}

TEST(IncludeTree, IncludeTreeFileListDuplicates) {
  std::shared_ptr<ObjectStore> DB = llvm::cas::createInMemoryCAS();
  SmallVector<IncludeTree::File> Files;
  for (unsigned I = 0; I < 10; ++I) {
    std::optional<IncludeTree::File> File;
    std::string Path = "/file" + std::to_string(I);
    static constexpr StringRef Bytes = "123456789";
    std::optional<ObjectRef> Content;
    ASSERT_THAT_ERROR(
        DB->storeFromString({}, Bytes.substr(0, I)).moveInto(Content),
        llvm::Succeeded());
    ASSERT_THAT_ERROR(
        IncludeTree::File::create(*DB, Path, *Content).moveInto(File),
        llvm::Succeeded());
    Files.push_back(std::move(*File));
  }

  auto MakeFileList = [&](unsigned Begin, unsigned End,
                          ArrayRef<ObjectRef> Lists) {
    SmallVector<IncludeTree::FileList::FileEntry> Entries;
    for (; Begin != End; ++Begin)
      Entries.push_back({Files[Begin].getRef(), Begin});
    return IncludeTree::FileList::create(*DB, Entries, Lists);
  };

  std::optional<IncludeTree::FileList> L89, L;
  ASSERT_THAT_ERROR(MakeFileList(8, 10, {}).moveInto(L89), llvm::Succeeded());
  EXPECT_EQ(L89->getNumReferences(), 2u);
  ASSERT_THAT_ERROR(
      MakeFileList(0, 9, {L89->getRef(), L89->getRef()}).moveInto(L),
      llvm::Succeeded());
  EXPECT_EQ(L->getNumReferences(), 11u); // 0, 1, ..., 8, {8, 9}, {8, 9}

  size_t I = 0;
  ASSERT_THAT_ERROR(
      L->forEachFile([&](IncludeTree::File F, auto Size) -> llvm::Error {
        if (I >= Files.size())
          return llvm::Error::success(); // diagnosed later.
        EXPECT_EQ(F.getFilenameRef(), Files[I].getFilenameRef())
            << "filename mismatch at " << I;
        EXPECT_EQ(F.getContentsRef(), Files[I].getContentsRef())
            << "contents mismatch at " << I;
        EXPECT_EQ(Size, I) << "size mismatch at " << I;
        I += 1;
        return llvm::Error::success();
      }),
      llvm::Succeeded());
  EXPECT_EQ(I, Files.size());
}

static Expected<IntrusiveRefCntPtr<llvm::vfs::FileSystem>>
createTestInclueTreeFS(ObjectStore &DB, unsigned NumFiles = 10) {
  SmallVector<IncludeTree::FileList::FileEntry> Files;
  for (unsigned I = 0; I < NumFiles; ++I) {
    std::optional<IncludeTree::File> File;
    std::string Path = "/file" + std::to_string(I);
    static constexpr StringRef Bytes = "123456789";
    std::optional<ObjectRef> Content;
    if (auto E = DB.storeFromString({}, Bytes.substr(0, I)).moveInto(Content))
      return std::move(E);
    if (auto E = IncludeTree::File::create(DB, Path, *Content).moveInto(File))
      return std::move(E);
    Files.push_back({File->getRef(), I});
  }
  return createIncludeTreeFileSystem(DB, Files);
}

TEST(IncludeTree, IncludeTreeFileSystemOverlay) {
  StringRef PathSep = llvm::sys::path::get_separator();
  std::shared_ptr<ObjectStore> DB = llvm::cas::createInMemoryCAS();
  IntrusiveRefCntPtr<llvm::vfs::FileSystem> IncludeTreeFS;
  ASSERT_THAT_ERROR(createTestInclueTreeFS(*DB).moveInto(IncludeTreeFS),
                    llvm::Succeeded());

  auto FS = llvm::makeIntrusiveRefCnt<llvm::vfs::InMemoryFileSystem>();
  FS->setCurrentWorkingDirectory("/dir");
  FS->addFile("file1", 0, llvm::MemoryBuffer::getMemBuffer("str"));
  FS->addFile("file2", 0, llvm::MemoryBuffer::getMemBuffer("other"));

  llvm::vfs::OverlayFileSystem OverlayFS(std::move(FS));
  OverlayFS.pushOverlay(IncludeTreeFS);

  std::error_code EC;
  int NumFile = 0;
  for (auto I = OverlayFS.dir_begin(PathSep.str() + "dir", EC);
       !EC && I != llvm::vfs::directory_iterator(); I.increment(EC)) {
    ASSERT_FALSE(EC);
    ++NumFile;
    std::string Path = PathSep.str() + "dir" + PathSep.str() + "file" +
                       std::to_string(NumFile);
    ASSERT_EQ(I->path(), Path);
  }
  ASSERT_EQ(NumFile, 2);
}

TEST(IncludeTree, IncludeTreeFileSystemPrint) {
  std::shared_ptr<ObjectStore> DB = llvm::cas::createInMemoryCAS();
  IntrusiveRefCntPtr<llvm::vfs::FileSystem> IncludeTreeFS;
  ASSERT_THAT_ERROR(
      createTestInclueTreeFS(*DB, /*NumFiles=*/3).moveInto(IncludeTreeFS),
      llvm::Succeeded());

  {
    std::string FSStr;
    llvm::raw_string_ostream OS(FSStr);
    IncludeTreeFS->print(OS, llvm::vfs::FileSystem::PrintType::Summary);
    EXPECT_EQ(FSStr, "IncludeTreeFileSystem\n");
  }
  {
    std::string FSStr;
    llvm::raw_string_ostream OS(FSStr);
    IncludeTreeFS->print(OS, llvm::vfs::FileSystem::PrintType::Contents);
    StringRef Printed(FSStr);
    EXPECT_TRUE(Printed.consume_front("IncludeTreeFileSystem")) << Printed;
    EXPECT_TRUE(Printed.consume_front("\n  /file0 llvmcas://")) << Printed;
    Printed = Printed.drop_front(64); // 32 hash bytes in hex
    EXPECT_TRUE(Printed.consume_front("\n  /file1 llvmcas://")) << Printed;
    Printed = Printed.drop_front(64); // 32 hash bytes in hex
    EXPECT_TRUE(Printed.consume_front("\n  /file2 llvmcas://")) << Printed;
    Printed = Printed.drop_front(64); // 32 hash bytes in hex
    EXPECT_EQ(Printed, "\n");         // Final newline.
  }
}
