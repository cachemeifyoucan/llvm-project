//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Defines the llvm::cas::CASConfiguration interface.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_CAS_CASCONFIGURATION_H
#define LLVM_CAS_CASCONFIGURATION_H

#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/Error.h"
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace llvm {
namespace cas {

class ActionCache;
class ObjectStore;

/// Base class for options configuring which CAS to use.
class CASConfiguration {
public:
  /// Path to a persistent backing store on-disk.
  ///
  /// - "" means there is none; falls back to in-memory.
  /// - "auto" is an alias for an automatically chosen location in the user's
  ///   system cache.
  std::string CASPath;
  /// Path to the CAS plugin library.
  std::string PluginPath;
  /// Each entry is a (<option-name>, <value>) pair.
  std::vector<std::pair<std::string, std::string>> PluginOptions;

  friend bool operator==(const CASConfiguration &LHS,
                         const CASConfiguration &RHS) {
    return LHS.CASPath == RHS.CASPath && LHS.PluginPath == RHS.PluginPath &&
           LHS.PluginOptions == RHS.PluginOptions;
  }
  friend bool operator!=(const CASConfiguration &LHS,
                         const CASConfiguration &RHS) {
    return !(LHS == RHS);
  }

  // Create CASDatabase from the CASConfiguration.
  LLVM_ABI llvm::Expected<std::pair<std::shared_ptr<llvm::cas::ObjectStore>,
                                    std::shared_ptr<llvm::cas::ActionCache>>>
  createDatabases() const;

  /// Get resolved CASPath.
  LLVM_ABI Error getResolvedCASPath(llvm::SmallVectorImpl<char> &Result) const;

  operator bool() const { return !CASPath.empty() || !PluginPath.empty(); }
};

} // namespace cas
} // namespace llvm

#endif // LLVM_CAS_CASCONFIGURATION_H
