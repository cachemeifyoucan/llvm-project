//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CAS_CASFILESYSTEM_H
#define LLVM_CAS_CASFILESYSTEM_H

#include "llvm/CAS/CASReference.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/ExtensibleRTTI.h"
#include "llvm/Support/VirtualFileSystem.h"

namespace llvm::cas {

/// Abstract class represents an open file backed by a CAS.
class CASBackedFile : public RTTIExtends<CASBackedFile, vfs::File> {
public:
  LLVM_ABI static const char ID;
  /// Get the CAS reference for the contents of the file.
  virtual cas::ObjectRef getObjectRefForContent() = 0;

protected:
  /// Keep the file CAS-backed when it is given a fixed status, e.g. when it
  /// is read through a \c vfs::RedirectingFileSystem.
  LLVM_ABI std::unique_ptr<vfs::File>
  createWithFixedStatus(std::unique_ptr<vfs::File> Self,
                        vfs::Status S) override;
};

/// Abstract class represents a CAS backed file system.
class CASBackedFileSystem
    : public llvm::RTTIExtends<CASBackedFileSystem, vfs::FileSystem> {
public:
  LLVM_ABI static const char ID;

  /// This is a convenience method that opens a file, gets its content and then
  /// closes the file. It returns MemoryBuffer and ObjectRef in one call to
  /// avoid open the file twice.
  LLVM_ABI llvm::Expected<
      std::pair<std::unique_ptr<llvm::MemoryBuffer>, cas::ObjectRef>>
  getBufferAndObjectRefForFile(const Twine &Name, int64_t FileSize = -1,
                               bool RequiresNullTerminator = true,
                               bool IsVolatile = false);

  /// Get ObjectRef of a file from its path.
  LLVM_ABI llvm::Expected<cas::ObjectRef>
  getObjectRefForFileContent(const Twine &Name);

  /// Implementation for openFileForRead using CASBackedFile.
  ErrorOr<std::unique_ptr<vfs::File>>
  openFileForRead(const Twine &Path) override {
    auto F = openCASBackedFileForRead(Path);
    if (!F)
      return errorToErrorCode(F.takeError());
    return std::move(*F);
  }

  /// Get CASBackedFile for read.
  virtual llvm::Expected<std::unique_ptr<CASBackedFile>>
  openCASBackedFileForRead(const Twine &Path) = 0;

  /// Get a proxy FS that has an independent working directory.
  virtual IntrusiveRefCntPtr<CASBackedFileSystem> createThreadSafeProxyFS() = 0;
};

} // namespace llvm::cas

#endif // LLVM_CAS_CASFILESYSTEM_H
