//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/CAS/CASConfiguration.h"
#include "llvm/CAS/ActionCache.h"
#include "llvm/CAS/BuiltinUnifiedCASDatabases.h"
#include "llvm/CAS/ObjectStore.h"

using namespace llvm;
using namespace llvm::cas;

Error CASConfiguration::getResolvedCASPath(
    llvm::SmallVectorImpl<char> &Result) const {
  if (CASPath == "auto")
    return getDefaultOnDiskCASPath(Result);

  Result.assign(CASPath.begin(), CASPath.end());
  return Error::success();
}

Expected<std::pair<std::shared_ptr<ObjectStore>, std::shared_ptr<ActionCache>>>
CASConfiguration::createDatabases() const {
  if (CASPath.empty()) {
    return std::pair(createInMemoryCAS(), createInMemoryActionCache());
  }

  SmallString<128> PathBuf;
  if (auto E = getResolvedCASPath(PathBuf))
    return std::move(E);

  if (!PluginPath.empty())
    return createPluginCASDatabases(PluginPath, PathBuf, PluginOptions);

  return createOnDiskUnifiedCASDatabases(PathBuf);
}
