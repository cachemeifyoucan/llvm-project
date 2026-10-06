//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/CAS/CASFileSystem.h"
#include "llvm/CAS/ObjectStore.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/VirtualFileSystem.h"
#include "llvm/Testing/Support/Error.h"
#include "gtest/gtest.h"

using namespace llvm;
using namespace llvm::cas;

namespace {

class CASFileSystemTest : public ::testing::Test {
protected:
  void SetUp() override { CAS = createInMemoryCAS(); }

  /// Make a native absolute path from a POSIX-style one, so the test also
  /// runs on Windows.
  static std::string path(StringRef Posix) {
    SmallString<128> Path;
#ifdef _WIN32
    Path = "C:";
#endif
    Path += Posix;
    sys::path::native(Path);
    return std::string(Path);
  }

  ObjectRef blob(StringRef Content) {
    return cantFail(CAS->storeFromString({}, Content));
  }

  FileSystemProxy buildSnapshot() {
    FileSystemSchema::Builder Builder(*CAS);
    Builder.add(path("/root/a.td"), blob("a"));
    Builder.add(path("/root/./inc/b.td"), blob("b"));
    Builder.add(path("/root/inc/deeper/c.td"), blob("c"));
    Builder.add(path("/other/a.td"), blob("a"));
    return cantFail(Builder.build());
  }

  std::unique_ptr<ObjectStore> CAS;
};

TEST_F(CASFileSystemTest, SchemaRoundTrip) {
  FileSystemProxy Snapshot = buildSnapshot();
  FileSystemSchema Schema = cantFail(FileSystemSchema::create(*CAS));
  EXPECT_TRUE(Schema.isNode(Snapshot));
  EXPECT_TRUE(Schema.isRootNode(Snapshot));

  // A blob, or a NamedValuesSchema node on its own, is not a snapshot.
  ObjectProxy Blob = cantFail(CAS->getProxy(blob("a")));
  EXPECT_FALSE(Schema.isNode(Blob));
  EXPECT_FALSE(Schema.isNode(cantFail(CAS->getProxy(
      Snapshot.getFiles().getRef()))));
  EXPECT_THAT_EXPECTED(Schema.load(Blob), Failed());

  std::optional<FileSystemProxy> Loaded;
  ASSERT_THAT_ERROR(Schema.load(Snapshot.getRef()).moveInto(Loaded),
                    Succeeded());
  EXPECT_EQ(Loaded->getID(), Snapshot.getID());
  ASSERT_EQ(Loaded->getFiles().size(), 4u);

  // "." components are dropped when the path is added.
  std::optional<ObjectRef> B = Loaded->lookupFile(path("/root/inc/b.td"));
  ASSERT_TRUE(B);
  EXPECT_EQ(cantFail(CAS->getProxy(*B)).getData(), "b");
  EXPECT_FALSE(Loaded->lookupFile(path("/root/b.td")));

  // Identical contents produce an identical snapshot, regardless of the order
  // files are added in.
  FileSystemSchema::Builder Reversed(*CAS);
  Reversed.add(path("/other/a.td"), blob("a"));
  Reversed.add(path("/root/inc/deeper/c.td"), blob("c"));
  Reversed.add(path("/root/inc/b.td"), blob("b"));
  Reversed.add(path("/root/a.td"), blob("a"));
  EXPECT_EQ(cantFail(Reversed.build()).getID(), Snapshot.getID());
}

TEST_F(CASFileSystemTest, BuilderErrors) {
  FileSystemSchema::Builder Relative(*CAS);
  Relative.add("relative.td", blob("a"));
  EXPECT_THAT_EXPECTED(Relative.build(), Failed());

  FileSystemSchema::Builder Duplicate(*CAS);
  Duplicate.add(path("/a.td"), blob("a"));
  Duplicate.add(path("/./a.td"), blob("b"));
  EXPECT_THAT_EXPECTED(Duplicate.build(), Failed());
}

TEST_F(CASFileSystemTest, SnapshotFiles) {
  auto Real = makeIntrusiveRefCnt<vfs::InMemoryFileSystem>();
  Real->addFile(path("/root/a.td"), 0, MemoryBuffer::getMemBuffer("a"));
  Real->addFile(path("/root/inc/b.td"), 0, MemoryBuffer::getMemBuffer("b"));
  Real->addFile(path("/root/unused.td"), 0, MemoryBuffer::getMemBuffer("u"));
  ASSERT_FALSE(Real->setCurrentWorkingDirectory(path("/root")));

  // Relative and absolute paths to the same file are stored once, and only
  // the named files are captured.
  std::optional<FileSystemProxy> Snapshot;
  ASSERT_THAT_ERROR(
      snapshotFiles(*CAS, *Real, {"a.td", path("/root/a.td"), "./inc/b.td"})
          .moveInto(Snapshot),
      Succeeded());
  ASSERT_EQ(Snapshot->getFiles().size(), 2u);

  IntrusiveRefCntPtr<vfs::FileSystem> FS =
      createCASFileSystem(*Snapshot, path("/root"));
  ErrorOr<std::unique_ptr<MemoryBuffer>> B = FS->getBufferForFile("inc/b.td");
  ASSERT_TRUE(B);
  EXPECT_EQ((*B)->getBuffer(), "b");
  EXPECT_FALSE(FS->exists("unused.td"));

  EXPECT_THAT_EXPECTED(snapshotFiles(*CAS, *Real, {"missing.td"}), Failed());
}

TEST_F(CASFileSystemTest, FileSystem) {
  IntrusiveRefCntPtr<vfs::FileSystem> FS =
      createCASFileSystem(buildSnapshot(), path("/root"));

  auto read = [&](const Twine &Path) -> std::string {
    ErrorOr<std::unique_ptr<MemoryBuffer>> Buffer = FS->getBufferForFile(Path);
    if (!Buffer)
      return "<" + Buffer.getError().message() + ">";
    return (*Buffer)->getBuffer().str();
  };

  // Absolute, relative, and "."-containing paths all resolve.
  EXPECT_EQ(read(path("/root/a.td")), "a");
  EXPECT_EQ(read("a.td"), "a");
  EXPECT_EQ(read("./inc/b.td"), "b");
  EXPECT_EQ(read(path("/other/a.td")), "a");

  ErrorOr<std::unique_ptr<MemoryBuffer>> Missing =
      FS->getBufferForFile("missing.td");
  EXPECT_EQ(Missing.getError(), std::errc::no_such_file_or_directory);
  // Directories are not represented.
  EXPECT_FALSE(FS->getBufferForFile("inc"));
  EXPECT_FALSE(FS->status("inc"));

  ErrorOr<vfs::Status> File = FS->status("inc/deeper/c.td");
  ASSERT_TRUE(File);
  EXPECT_TRUE(File->isRegularFile());
  EXPECT_EQ(File->getSize(), 1u);
  EXPECT_FALSE(FS->exists("missing.td"));

  // Relative paths follow the working directory.
  ASSERT_FALSE(FS->setCurrentWorkingDirectory("inc"));
  EXPECT_EQ(cantFail(errorOrToExpected(FS->getCurrentWorkingDirectory())),
            path("/root/inc"));
  EXPECT_EQ(read("b.td"), "b");
  EXPECT_EQ(read("a.td"), "<" +
                              std::make_error_code(
                                  std::errc::no_such_file_or_directory)
                                  .message() +
                              ">");
}

} // namespace
