//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Util for CC1 Dependency Scanning.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_DEPENDENCYSCANNING_SCANANDUPDATEARGS_H
#define LLVM_CLANG_DEPENDENCYSCANNING_SCANANDUPDATEARGS_H

#include "clang/Basic/LLVM.h"
#include "llvm/Support/Error.h"

namespace llvm {
namespace cas {
class ObjectStore;
class CASID;
} // namespace cas
} // namespace llvm

namespace clang {

class CASOptions;
class CompilerInvocation;
class CowCompilerInvocation;

namespace dependencies {

/// Apply CAS inputs for compilation caching to the given invocation, if
/// enabled.
void configureInvocationForCaching(CowCompilerInvocation &CI,
                                   CASOptions CASOpts, std::string InputID);

} // namespace dependencies
} // end namespace clang

#endif // LLVM_CLANG_DEPENDENCYSCANNING_SCANANDUPDATEARGS_H
