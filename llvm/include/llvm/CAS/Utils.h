//===- llvm/CAS/Utils.h -----------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CAS_UTILS_H
#define LLVM_CAS_UTILS_H

#include "llvm/Support/Error.h"

namespace llvm {
class MemoryBufferRef;

namespace cas {

class CASDB;
class CASID;
class NamedTreeEntry;
class NodeProxy;
class ObjectHandle;

Expected<CASID> readCASIDBuffer(cas::CASDB &CAS, llvm::MemoryBufferRef Buffer);

void writeCASIDBuffer(const CASID &ID, llvm::raw_ostream &OS);

/// Visit each file entry in order, returning an error from \p Callback to stop
/// early.
///
/// The \p NamedTreeEntry, that the \p Callback receives, points to a name
/// string that may not live beyond the return of the callback function.
///
/// Passes the \p TreeProxy if the entry is a \p TreeEntry::Tree, otherwise
/// passes \p None.
Error walkFileTreeRecursively(
    CASDB &CAS, const ObjectHandle &Root,
    function_ref<Error(const NamedTreeEntry &, Optional<NodeProxy>)> Callback);

} // namespace cas
} // namespace llvm

#endif // LLVM_CAS_UTILS_H
