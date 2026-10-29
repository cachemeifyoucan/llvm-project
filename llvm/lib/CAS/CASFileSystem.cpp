//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/CAS/CASFileSystem.h"
#include "llvm/CAS/ObjectStore.h"

using namespace llvm;
using namespace llvm::cas;

const char CASBackedFile::ID = 0;
const char CASBackedFileSystem::ID = 0;

llvm::Expected<std::pair<std::unique_ptr<llvm::MemoryBuffer>, cas::ObjectRef>>
CASBackedFileSystem::getBufferAndObjectRefForFile(const Twine &Name,
                                                  int64_t FileSize,
                                                  bool RequiresNullTerminator,
                                                  bool IsVolatile,
                                                  bool IsText) {
  auto F = openCASBackedFileForRead(Name);
  if (!F)
    return F.takeError();

  auto Buf =
      (*F)->getBuffer(Name, FileSize, RequiresNullTerminator, IsVolatile);
  if (!Buf)
    return errorCodeToError(Buf.getError());

  auto CASRef = (*F)->getObjectRefForContent();
  return std::pair{std::move(*Buf), CASRef};
}

llvm::Expected<cas::ObjectRef>
CASBackedFileSystem::getObjectRefForFileContent(const Twine &Name) {
  auto F = openCASBackedFileForRead(Name);
  if (!F)
    return F.takeError();
  return (*F)->getObjectRefForContent();
}
