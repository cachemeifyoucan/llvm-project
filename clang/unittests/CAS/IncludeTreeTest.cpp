//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "clang/CAS/IncludeTree.h"
#include "llvm/ADT/SmallBitVector.h"
#include "llvm/CAS/ObjectStore.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/VirtualFileSystem.h"
#include "llvm/Testing/Support/Error.h"
#include "gtest/gtest.h"
#include <system_error>

using namespace clang;
using namespace clang::cas;

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
