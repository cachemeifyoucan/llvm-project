//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_DEPENDENCYSCANNING_CACHINGACTIONS_H
#define LLVM_CLANG_DEPENDENCYSCANNING_CACHINGACTIONS_H

#include "clang/DependencyScanning/DependencyScanningUtils.h"
#include "clang/DependencyScanning/ScanAndUpdateArgs.h"

namespace clang::dependencies {

std::unique_ptr<DependencyActionController>
createIncludeTreeActionController(LookupModuleOutputCallback LookupModuleOutput,
                                  CASOptions CASOpts, cas::ObjectStore &DB);

} // namespace clang::dependencies
#endif // LLVM_CLANG_DEPENDENCYSCANNING_CACHINGACTIONS_H
