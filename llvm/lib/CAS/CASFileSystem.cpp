//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/CAS/CASFileSystem.h"
#include "llvm/ADT/StringSet.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/StringSaver.h"
#include "llvm/Support/VirtualFileSystem.h"

using namespace llvm;
using namespace llvm::cas;

char FileSystemSchema::ID = 0;
constexpr StringLiteral FileSystemSchema::SchemaName;

void FileSystemSchema::anchor() {}

/// Make \p Path absolute against \p WorkingDirectory and drop "." components.
/// ".." components are kept, since resolving them lexically is wrong in the
/// presence of symlinks. The paths in a snapshot and the paths looked up in it
/// both go through this.
static void normalizePath(StringRef WorkingDirectory, const Twine &Path,
                          SmallVectorImpl<char> &Result) {
  Result.clear();
  Path.toVector(Result);
  if (!WorkingDirectory.empty())
    sys::path::make_absolute(WorkingDirectory, Result);
  sys::path::remove_dots(Result, /*remove_dot_dot=*/false);
}

FileSystemSchema::FileSystemSchema(ObjectStore &CAS, Error &E)
    : FileSystemSchema::RTTIExtends(CAS) {
  ErrorAsOutParameter EAOP(E);
  auto Kind = CAS.storeFromString({}, SchemaName);
  if (!Kind) {
    E = Kind.takeError();
    return;
  }
  FileSystemKindRef = *Kind;
}

Expected<FileSystemSchema> FileSystemSchema::create(ObjectStore &CAS) {
  Error E = Error::success();
  FileSystemSchema S(CAS, E);
  if (E)
    return std::move(E);
  return S;
}

bool FileSystemSchema::isNode(const ObjectProxy &Node) const {
  return Node.getNumReferences() == 2 && Node.getData().empty() &&
         Node.getReference(0) == *FileSystemKindRef;
}

bool FileSystemSchema::isRootNode(const ObjectProxy &Node) const {
  if (!isNode(Node))
    return false;
  Expected<NamedValuesSchema> Files = NamedValuesSchema::create(CAS);
  if (!Files) {
    consumeError(Files.takeError());
    return false;
  }
  Expected<NamedValuesProxy> Entries = Files->load(Node.getReference(1));
  if (!Entries) {
    consumeError(Entries.takeError());
    return false;
  }
  return true;
}

Expected<FileSystemProxy> FileSystemSchema::load(ObjectRef Object) const {
  auto Node = CAS.getProxy(Object);
  if (!Node)
    return Node.takeError();
  return load(*Node);
}

Expected<FileSystemProxy> FileSystemSchema::load(ObjectProxy Object) const {
  if (!isNode(Object))
    return createStringError("object does not conform to FileSystemSchema");

  Expected<NamedValuesSchema> FilesSchema = NamedValuesSchema::create(CAS);
  if (!FilesSchema)
    return FilesSchema.takeError();
  Expected<NamedValuesProxy> Files = FilesSchema->load(Object.getReference(1));
  if (!Files)
    return Files.takeError();
  return FileSystemProxy(Object, *Files);
}

void FileSystemSchema::Builder::add(StringRef Path, ObjectRef Contents) {
  SmallString<256> Normalized;
  normalizePath(/*WorkingDirectory=*/"", Path, Normalized);
  Files.emplace_back(StringSaver(Alloc).save(Normalized.str()), Contents);
}

Expected<FileSystemProxy> FileSystemSchema::Builder::build() {
  for (const NamedValuesEntry &File : Files)
    if (!sys::path::is_absolute(File.Name))
      return createStringError("path in a file system snapshot must be "
                               "absolute: '" +
                               File.Name + "'");

  // NamedValuesSchema only rejects entries that match in both name and
  // reference; a path mapped to two different contents is also an error.
  llvm::sort(Files);
  auto Dup = llvm::adjacent_find(
      Files, [](const NamedValuesEntry &LHS, const NamedValuesEntry &RHS) {
        return LHS.Name == RHS.Name;
      });
  if (Dup != Files.end())
    return createStringError("path added twice to a file system snapshot: '" +
                             Dup->Name + "'");

  Expected<FileSystemSchema> Schema = FileSystemSchema::create(CAS);
  if (!Schema)
    return Schema.takeError();
  Expected<NamedValuesSchema> FilesSchema = NamedValuesSchema::create(CAS);
  if (!FilesSchema)
    return FilesSchema.takeError();
  Expected<NamedValuesProxy> Entries = FilesSchema->construct(Files);
  if (!Entries)
    return Entries.takeError();

  Expected<ObjectProxy> Node =
      CAS.createProxy({*Schema->FileSystemKindRef, Entries->getRef()}, "");
  if (!Node)
    return Node.takeError();
  return FileSystemProxy(*Node, *Entries);
}

Expected<FileSystemProxy> cas::snapshotFiles(ObjectStore &CAS,
                                             vfs::FileSystem &FS,
                                             ArrayRef<StringRef> Paths) {
  ErrorOr<std::string> WorkingDirectory = FS.getCurrentWorkingDirectory();
  if (!WorkingDirectory)
    return createStringError(WorkingDirectory.getError(),
                             "could not get the working directory");

  FileSystemSchema::Builder Builder(CAS);
  StringSet<> Seen;
  for (StringRef Path : Paths) {
    SmallString<256> Normalized;
    normalizePath(*WorkingDirectory, Path, Normalized);
    if (!Seen.insert(Normalized).second)
      continue;
    ErrorOr<std::unique_ptr<MemoryBuffer>> Buffer =
        FS.getBufferForFile(Normalized);
    if (!Buffer)
      return createFileError(Path, Buffer.getError());
    Expected<ObjectRef> Contents =
        CAS.storeFromString({}, (*Buffer)->getBuffer());
    if (!Contents)
      return Contents.takeError();
    Builder.add(Normalized, *Contents);
  }
  return Builder.build();
}

namespace {

class CASFile final : public vfs::File {
public:
  CASFile(ObjectProxy Contents, vfs::Status Stat)
      : Contents(Contents), Stat(std::move(Stat)) {}

  ErrorOr<vfs::Status> status() override { return Stat; }

  ErrorOr<std::unique_ptr<MemoryBuffer>>
  getBuffer(const Twine &Name, int64_t FileSize, bool RequiresNullTerminator,
            bool IsVolatile) override {
    return Contents.getMemoryBuffer(Name.str(), RequiresNullTerminator);
  }

  std::error_code close() override { return {}; }

private:
  ObjectProxy Contents;
  vfs::Status Stat;
};

class CASFileSystem final : public vfs::FileSystem {
public:
  CASFileSystem(const FileSystemProxy &Snapshot, StringRef WorkingDirectory)
      : Snapshot(Snapshot), WorkingDirectory(WorkingDirectory) {}

  ErrorOr<vfs::Status> status(const Twine &Path) override {
    SmallString<256> Normalized;
    normalizePath(WorkingDirectory, Path, Normalized);
    ErrorOr<ObjectProxy> Blob = lookup(Normalized);
    if (!Blob)
      return Blob.getError();
    return makeStatus(Path, Blob->getData().size());
  }

  ErrorOr<std::unique_ptr<vfs::File>>
  openFileForRead(const Twine &Path) override {
    SmallString<256> Normalized;
    normalizePath(WorkingDirectory, Path, Normalized);
    ErrorOr<ObjectProxy> Blob = lookup(Normalized);
    if (!Blob)
      return Blob.getError();
    vfs::Status Stat = makeStatus(Path, Blob->getData().size());
    return std::make_unique<CASFile>(*Blob, std::move(Stat));
  }

  vfs::directory_iterator dir_begin(const Twine &Dir,
                                    std::error_code &EC) override {
    EC = std::make_error_code(std::errc::operation_not_supported);
    return {};
  }

  std::error_code setCurrentWorkingDirectory(const Twine &Path) override {
    SmallString<256> Normalized;
    normalizePath(WorkingDirectory, Path, Normalized);
    WorkingDirectory = Normalized.str();
    return {};
  }

  ErrorOr<std::string> getCurrentWorkingDirectory() const override {
    return WorkingDirectory;
  }

private:
  ErrorOr<ObjectProxy> lookup(StringRef Path) const {
    std::optional<ObjectRef> Contents = Snapshot.lookupFile(Path);
    if (!Contents)
      return std::make_error_code(std::errc::no_such_file_or_directory);
    Expected<ObjectProxy> Blob = Snapshot.getCAS().getProxy(*Contents);
    if (!Blob)
      return errorToErrorCode(Blob.takeError());
    return *Blob;
  }

  static vfs::Status makeStatus(const Twine &Path, uint64_t Size) {
    return vfs::Status(Path, vfs::getNextVirtualUniqueID(),
                       sys::TimePoint<>(), /*User=*/0, /*Group=*/0, Size,
                       sys::fs::file_type::regular_file, sys::fs::all_read);
  }

  FileSystemProxy Snapshot;
  std::string WorkingDirectory;
};

} // namespace

IntrusiveRefCntPtr<vfs::FileSystem>
cas::createCASFileSystem(const FileSystemProxy &Snapshot,
                         StringRef WorkingDirectory) {
  assert(sys::path::is_absolute(WorkingDirectory) &&
         "working directory must be absolute");
  return makeIntrusiveRefCnt<CASFileSystem>(Snapshot, WorkingDirectory);
}
