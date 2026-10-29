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
#include "llvm/Testing/Support/Error.h"
#include "gtest/gtest.h"

using namespace llvm;
using namespace llvm::cas;

namespace {
/// A file whose contents are a CAS object.
class TestCASFile final : public RTTIExtends<TestCASFile, CASBackedFile> {
public:
  static const char ID;

  TestCASFile(ObjectProxy Content, vfs::Status S)
      : Content(std::move(Content)), S(std::move(S)) {}

  ErrorOr<vfs::Status> status() override { return S; }
  ErrorOr<std::unique_ptr<MemoryBuffer>> getBuffer(const Twine &Name, int64_t,
                                                   bool, bool) override {
    return Content.getMemoryBuffer(Name.str());
  }
  std::error_code close() override { return {}; }
  ObjectRef getObjectRefForContent() override { return Content.getRef(); }

private:
  ObjectProxy Content;
  vfs::Status S;
};
const char TestCASFile::ID = 0;

/// A file system that serves files stored in a CAS, by path.
class TestCASFileSystem final
    : public RTTIExtends<TestCASFileSystem, CASBackedFileSystem> {
public:
  static const char ID;

  explicit TestCASFileSystem(std::shared_ptr<ObjectStore> CAS)
      : CAS(std::move(CAS)) {}

  Error addFile(StringRef Path, StringRef Content) {
    auto Proxy = CAS->createProxy({}, Content);
    if (!Proxy)
      return Proxy.takeError();
    Files.insert({Path, Proxy->getRef()});
    return Error::success();
  }

  Expected<std::unique_ptr<CASBackedFile>>
  openCASBackedFileForRead(const Twine &Path) override {
    auto I = Files.find(Path.str());
    if (I == Files.end())
      return createFileError(
          Path, make_error_code(std::errc::no_such_file_or_directory));
    auto Proxy = CAS->getProxy(I->second);
    if (!Proxy)
      return Proxy.takeError();
    vfs::Status S(Path.str(), sys::fs::UniqueID(0, 0), sys::TimePoint<>(), 0, 0,
                  Proxy->getData().size(), sys::fs::file_type::regular_file,
                  sys::fs::all_all);
    return std::make_unique<TestCASFile>(std::move(*Proxy), std::move(S));
  }

  IntrusiveRefCntPtr<CASBackedFileSystem> createThreadSafeProxyFS() override {
    return makeIntrusiveRefCnt<TestCASFileSystem>(*this);
  }

  ErrorOr<vfs::Status> status(const Twine &Path) override {
    auto F = openCASBackedFileForRead(Path);
    if (!F)
      return errorToErrorCode(F.takeError());
    return (*F)->status();
  }
  vfs::directory_iterator dir_begin(const Twine &Dir,
                                    std::error_code &EC) override {
    EC = make_error_code(std::errc::not_supported);
    return {};
  }
  ErrorOr<std::string> getCurrentWorkingDirectory() const override {
    return std::string("/");
  }
  std::error_code setCurrentWorkingDirectory(const Twine &) override {
    return make_error_code(std::errc::not_supported);
  }

private:
  std::shared_ptr<ObjectStore> CAS;
  StringMap<ObjectRef> Files;
};
const char TestCASFileSystem::ID = 0;
} // namespace

TEST(CASBackedFileSystemTest, OpenFile) {
  std::shared_ptr<ObjectStore> CAS = createInMemoryCAS();
  auto FS = makeIntrusiveRefCnt<TestCASFileSystem>(CAS);
  ASSERT_THAT_ERROR(FS->addFile("/a", "content"), Succeeded());

  // Files opened through the generic vfs::FileSystem API are CAS-backed.
  auto F = FS->openFileForRead("/a");
  ASSERT_TRUE(F);
  auto *CASFile = dyn_cast<CASBackedFile>(F->get());
  ASSERT_TRUE(CASFile);

  std::optional<ObjectProxy> Expected;
  ASSERT_THAT_ERROR(CAS->createProxy({}, "content").moveInto(Expected),
                    Succeeded());
  EXPECT_EQ(CASFile->getObjectRefForContent(), Expected->getRef());

  // The file system is CAS-backed when used as a plain vfs::FileSystem.
  IntrusiveRefCntPtr<vfs::FileSystem> Base = FS;
  EXPECT_TRUE(isa<CASBackedFileSystem>(Base.get()));
  EXPECT_FALSE(isa<CASBackedFileSystem>(vfs::getRealFileSystem().get()));

  EXPECT_FALSE(FS->openFileForRead("/missing"));
}

TEST(CASBackedFileSystemTest, GetObjectRef) {
  std::shared_ptr<ObjectStore> CAS = createInMemoryCAS();
  auto FS = makeIntrusiveRefCnt<TestCASFileSystem>(CAS);
  ASSERT_THAT_ERROR(FS->addFile("/a", "content"), Succeeded());

  std::optional<ObjectProxy> Expected;
  ASSERT_THAT_ERROR(CAS->createProxy({}, "content").moveInto(Expected),
                    Succeeded());

  std::optional<ObjectRef> Ref;
  ASSERT_THAT_ERROR(FS->getObjectRefForFileContent("/a").moveInto(Ref),
                    Succeeded());
  EXPECT_EQ(*Ref, Expected->getRef());

  std::optional<std::pair<std::unique_ptr<MemoryBuffer>, ObjectRef>> BufAndRef;
  ASSERT_THAT_ERROR(FS->getBufferAndObjectRefForFile("/a").moveInto(BufAndRef),
                    Succeeded());
  EXPECT_EQ(BufAndRef->first->getBuffer(), "content");
  EXPECT_EQ(BufAndRef->second, Expected->getRef());

  EXPECT_THAT_ERROR(FS->getObjectRefForFileContent("/missing").takeError(),
                    Failed());
  EXPECT_THAT_ERROR(FS->getBufferAndObjectRefForFile("/missing").takeError(),
                    Failed());
}
