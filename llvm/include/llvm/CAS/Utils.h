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
class TreeProxy;
class TreeHandle;

Expected<CASID> readCASIDBuffer(cas::CASDB &CAS, llvm::MemoryBufferRef Buffer);

void writeCASIDBuffer(const CASID &ID, llvm::raw_ostream &OS);

Error walkFileTreeRecursively(
    CASDB &CAS, const TreeHandle &Root,
    function_ref<Error(const NamedTreeEntry &, Optional<TreeProxy>)> Callback);

} // namespace cas
} // namespace llvm

#endif // LLVM_CAS_UTILS_H
