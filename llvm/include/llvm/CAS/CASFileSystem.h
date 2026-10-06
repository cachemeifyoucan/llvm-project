//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains the declarations for FileSystemSchema, a minimal schema
/// for a read-only snapshot of a set of files stored in a CAS, and for a
/// \c vfs::FileSystem that serves it.
///
/// The snapshot is flat: it maps absolute paths to file contents. Directories,
/// symlinks, permissions, and timestamps are not represented. That is enough
/// to replay a tool whose inputs are regular files that it reads by path, like
/// `llvm-tblgen`.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_CAS_CASFILESYSTEM_H
#define LLVM_CAS_CASFILESYSTEM_H

#include "llvm/ADT/IntrusiveRefCntPtr.h"
#include "llvm/CAS/CASNodeSchema.h"
#include "llvm/CAS/NamedValuesSchema.h"
#include "llvm/CAS/ObjectStore.h"
#include "llvm/Support/Compiler.h"

namespace llvm {
namespace vfs {
class FileSystem;
} // namespace vfs

namespace cas {

class FileSystemProxy;

/// A schema for a flat snapshot of files in a CAS.
///
/// A root node has two references: the schema kind, and a \c NamedValuesSchema
/// node mapping each absolute path to the blob holding the file's contents.
/// Its data is empty.
class LLVM_ABI FileSystemSchema
    : public RTTIExtends<FileSystemSchema, NodeSchema> {
  void anchor() override;

public:
  static char ID;

  /// Check that \p Node is a node of this schema and that its entries are a
  /// valid \c NamedValuesSchema node.
  bool isRootNode(const ObjectProxy &Node) const final;

  /// Check if \p Node has the shape of a node of this schema.
  bool isNode(const ObjectProxy &Node) const final;

  /// Create a FileSystemSchema.
  static Expected<FileSystemSchema> create(ObjectStore &CAS);

  /// Load a FileSystemProxy from an ObjectRef.
  Expected<FileSystemProxy> load(ObjectRef Object) const;

  /// Load a FileSystemProxy from an ObjectProxy.
  Expected<FileSystemProxy> load(ObjectProxy Object) const;

  /// A builder class for creating nodes in FileSystemSchema.
  class Builder {
  public:
    Builder(ObjectStore &CAS) : CAS(CAS) {}

    /// Add the file at absolute path \p Path with contents \p Contents. "."
    /// components are dropped from \p Path, the same way paths looked up in the
    /// snapshot are normalized.
    LLVM_ABI void add(StringRef Path, ObjectRef Contents);

    /// Build the node from added files. Fails if a path is relative or was
    /// added twice.
    LLVM_ABI Expected<FileSystemProxy> build();

  private:
    ObjectStore &CAS;
    SmallVector<NamedValuesEntry> Files;
    BumpPtrAllocator Alloc;
  };

private:
  FileSystemSchema(ObjectStore &CAS, Error &E);

  /// Name for the schema.
  static constexpr StringLiteral SchemaName =
      "llvm::cas::schema::filesystem::v1";
  std::optional<ObjectRef> FileSystemKindRef;
};

/// A proxy for a loaded CAS object in FileSystemSchema.
class FileSystemProxy : public ObjectProxy {
public:
  /// Get the files in the snapshot, sorted by path.
  const NamedValuesProxy &getFiles() const { return Files; }

  /// Look up the contents of the file at absolute path \p Path.
  std::optional<ObjectRef> lookupFile(StringRef Path) const {
    if (std::optional<NamedValuesEntry> Entry = Files.lookup(Path))
      return Entry->Ref;
    return std::nullopt;
  }

private:
  FileSystemProxy(const ObjectProxy &Node, const NamedValuesProxy &Files)
      : ObjectProxy(Node), Files(Files) {}

  friend class FileSystemSchema;
  NamedValuesProxy Files;
};

/// Snapshot the files at \p Paths in \p FS. Relative paths are resolved
/// against the working directory of \p FS; a file named more than once is
/// stored once.
LLVM_ABI Expected<FileSystemProxy>
snapshotFiles(ObjectStore &CAS, vfs::FileSystem &FS, ArrayRef<StringRef> Paths);

/// Create a read-only \c vfs::FileSystem serving the files in \p Snapshot.
/// Relative paths are resolved against \p WorkingDirectory, which must be
/// absolute. Only files can be opened or stat'ed; directory iteration is not
/// supported.
LLVM_ABI IntrusiveRefCntPtr<vfs::FileSystem>
createCASFileSystem(const FileSystemProxy &Snapshot,
                    StringRef WorkingDirectory);

} // namespace cas
} // namespace llvm

#endif // LLVM_CAS_CASFILESYSTEM_H
