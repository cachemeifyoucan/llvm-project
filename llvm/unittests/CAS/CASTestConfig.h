//===- CASTestConfig.h ----------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/CAS/ObjectStore.h"
#include "llvm/Config/llvm-config.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Testing/Support/Error.h"
#include "llvm/Testing/Support/SupportHelpers.h"
#include "gtest/gtest.h"

#ifndef LLVM_UNITTESTS_CASTESTCONFIG_H
#define LLVM_UNITTESTS_CASTESTCONFIG_H

struct CASTestingEnv {
  std::unique_ptr<llvm::cas::ObjectStore> CAS;
};

class CASTest
    : public testing::TestWithParam<std::function<CASTestingEnv(int)>> {
protected:
  llvm::Optional<int> NextCASIndex;

  std::unique_ptr<llvm::cas::ObjectStore> createObjectStore() {
    auto TD = GetParam()(++(*NextCASIndex));
    return std::move(TD.CAS);
  }
  void SetUp() { NextCASIndex = 0; }
  void TearDown() { NextCASIndex = llvm::None; }
};

#endif
