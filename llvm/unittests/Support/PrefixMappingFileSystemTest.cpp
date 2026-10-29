//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/Support/PrefixMappingFileSystem.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/PrefixMapper.h"
#include "llvm/Support/VirtualFileSystem.h"
#include "gtest/gtest.h"

using namespace llvm;

namespace {
IntrusiveRefCntPtr<vfs::FileSystem> createMappedFS() {
  auto MemFS = makeIntrusiveRefCnt<vfs::InMemoryFileSystem>();
  MemFS->addFile("/real/dir/file", 0, MemoryBuffer::getMemBuffer("content"));
  MemFS->addFile("/other/file", 0, MemoryBuffer::getMemBuffer("other"));

  PrefixMapper Mapper(sys::path::Style::posix);
  Mapper.add(MappedPrefix("/virtual", "/real"));
  return vfs::createPrefixMappingFileSystem(std::move(Mapper),
                                            std::move(MemFS));
}
} // namespace

TEST(PrefixMappingFileSystemTest, Status) {
  auto FS = createMappedFS();

  // A remapped path exposes the underlying path.
  ErrorOr<vfs::Status> S = FS->status("/virtual/dir/file");
  ASSERT_TRUE(S);
  EXPECT_TRUE(S->ExposesExternalVFSPath);
  EXPECT_EQ(S->getName(), "/real/dir/file");

  // Paths that are not remapped pass through unchanged.
  S = FS->status("/other/file");
  ASSERT_TRUE(S);
  EXPECT_FALSE(S->ExposesExternalVFSPath);
  EXPECT_EQ(S->getName(), "/other/file");

  // The original path is not visible through the mapped prefix.
  EXPECT_FALSE(FS->status("/virtual/missing"));
  EXPECT_FALSE(FS->status("/virtualdir/file"));
}

TEST(PrefixMappingFileSystemTest, OpenFile) {
  auto FS = createMappedFS();

  auto F = FS->openFileForRead("/virtual/dir/file");
  ASSERT_TRUE(F);
  ErrorOr<vfs::Status> S = (*F)->status();
  ASSERT_TRUE(S);
  EXPECT_TRUE(S->ExposesExternalVFSPath);
  auto Buf = (*F)->getBuffer("file");
  ASSERT_TRUE(Buf);
  EXPECT_EQ((*Buf)->getBuffer(), "content");

  F = FS->openFileForRead("/other/file");
  ASSERT_TRUE(F);
  S = (*F)->status();
  ASSERT_TRUE(S);
  EXPECT_FALSE(S->ExposesExternalVFSPath);
}

TEST(PrefixMappingFileSystemTest, DirectoryIteration) {
  auto FS = createMappedFS();

  std::error_code EC;
  vfs::directory_iterator I = FS->dir_begin("/virtual/dir", EC);
  ASSERT_FALSE(EC);
  ASSERT_NE(I, vfs::directory_iterator());
  EXPECT_EQ(I->path(), "/real/dir/file");
  I.increment(EC);
  ASSERT_FALSE(EC);
  EXPECT_EQ(I, vfs::directory_iterator());
}

TEST(PrefixMappingFileSystemTest, WorkingDirectory) {
  auto FS = createMappedFS();

  ASSERT_FALSE(FS->setCurrentWorkingDirectory("/virtual/dir"));
  ErrorOr<std::string> CWD = FS->getCurrentWorkingDirectory();
  ASSERT_TRUE(CWD);
  EXPECT_EQ(*CWD, "/real/dir");

  // Relative paths are resolved against the remapped working directory.
  ErrorOr<vfs::Status> S = FS->status("file");
  ASSERT_TRUE(S);
  ErrorOr<vfs::Status> Real = FS->status("/real/dir/file");
  ASSERT_TRUE(Real);
  EXPECT_EQ(S->getUniqueID(), Real->getUniqueID());
}
