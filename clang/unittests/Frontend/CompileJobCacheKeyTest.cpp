//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "clang/Frontend/CompileJobCacheKey.h"
#include "FaultingCAS.h"
#include "clang/Basic/DiagnosticOptions.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/CompilerInvocation.h"
#include "clang/Frontend/TextDiagnosticBuffer.h"
#include "llvm/Support/VirtualFileSystem.h"
#include "llvm/Testing/Support/Error.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using namespace clang;
using namespace llvm::cas;

namespace {
struct CompileJobCacheKeyStoreFailureTest
    : public ::testing::TestWithParam<unsigned> {};
} // end anonymous namespace

TEST_P(CompileJobCacheKeyStoreFailureTest, EmitsDiagAndReturnsNullopt) {
  auto Inner = createInMemoryCAS();
  // Create the fake include-tree input.
  ObjectRef InputRef = llvm::cantFail(Inner->storeFromString({}, "input"));
  std::string InputCASID = Inner->getID(InputRef).toString();

  // Inject an error at the store indexed by the test param.
  FaultingCAS CAS(std::move(Inner), GetParam());

  DiagnosticOptions DiagOpts;
  auto *DiagBuffer = new TextDiagnosticBuffer();
  llvm::IntrusiveRefCntPtr<DiagnosticsEngine> Diags =
      CompilerInstance::createDiagnostics(*llvm::vfs::getRealFileSystem(),
                                          DiagOpts, DiagBuffer);

  CompilerInvocation Invocation;
  ASSERT_TRUE(CompilerInvocation::CreateFromArgs(Invocation, {}, *Diags));
  Invocation.getFrontendOpts().CASIncludeTreeID = InputCASID;

  auto Key = createCompileJobCacheKey(CAS, *Diags, Invocation);
  EXPECT_FALSE(Key.has_value());

  EXPECT_EQ(DiagBuffer->getNumErrors(), 1u);
  if (DiagBuffer->getNumErrors())
    EXPECT_THAT(DiagBuffer->err_begin()->second,
                ::testing::HasSubstr("failed to store to CAS"));
}

// There are 5 store calls along the success path of createCompileJobCacheKey
// (command-line, "-cc1", version, schema kind-ref inside Builder::build, and
// the final NamedValues object). Injecting a failure at any of them must
// produce an err_cas_store diagnostic instead of the previous cantFail crash.
INSTANTIATE_TEST_SUITE_P(AllStoreCalls, CompileJobCacheKeyStoreFailureTest,
                         ::testing::Range(0u, 5u));

namespace {
class CompileJobCacheKeyTest : public ::testing::Test {
protected:
  std::unique_ptr<ObjectStore> CAS = createInMemoryCAS();
  DiagnosticOptions DiagOpts;
  TextDiagnosticBuffer *DiagBuffer = new TextDiagnosticBuffer();
  llvm::IntrusiveRefCntPtr<DiagnosticsEngine> Diags =
      CompilerInstance::createDiagnostics(*llvm::vfs::getRealFileSystem(),
                                          DiagOpts, DiagBuffer);

  CompilerInvocation createInvocation(ArrayRef<const char *> Args) {
    CompilerInvocation Invocation;
    EXPECT_TRUE(CompilerInvocation::CreateFromArgs(Invocation, Args, *Diags));
    ObjectRef InputRef = llvm::cantFail(CAS->storeFromString({}, "input"));
    Invocation.getFrontendOpts().CASIncludeTreeID =
        CAS->getID(InputRef).toString();
    return Invocation;
  }

  std::string getKey(const CompilerInvocation &Invocation) {
    std::optional<CASID> Key =
        createCompileJobCacheKey(*CAS, *Diags, Invocation);
    EXPECT_TRUE(Key);
    return Key ? Key->toString() : std::string();
  }
};
} // end anonymous namespace

TEST_F(CompileJobCacheKeyTest, Deterministic) {
  CompilerInvocation Invocation = createInvocation({"-emit-obj"});
  EXPECT_EQ(getKey(Invocation), getKey(Invocation));
  EXPECT_EQ(getKey(Invocation), getKey(createInvocation({"-emit-obj"})));
}

TEST_F(CompileJobCacheKeyTest, IndependentOfOutputsAndRendering) {
  std::string Key = getKey(createInvocation({"-emit-obj", "-o", "a.o"}));
  EXPECT_EQ(Key, getKey(createInvocation({"-emit-obj", "-o", "b.o"})));
  EXPECT_EQ(Key, getKey(createInvocation(
                     {"-emit-obj", "-o", "b.o", "-fcolor-diagnostics"})));
  EXPECT_EQ(Key, getKey(createInvocation(
                     {"-emit-obj", "-o", "b.o", "-fcas-path", "/some/cas"})));

  // Whether a dependency file is produced is part of the key, but not its path
  // or targets.
  std::string DepKey = getKey(createInvocation(
      {"-emit-obj", "-o", "a.o", "-dependency-file", "a.d", "-MT", "a.o"}));
  EXPECT_NE(Key, DepKey);
  EXPECT_EQ(DepKey, getKey(createInvocation({"-emit-obj", "-o", "b.o",
                                             "-dependency-file", "b.d", "-MT",
                                             "b.o"})));
}

TEST_F(CompileJobCacheKeyTest, DependsOnCompilation) {
  std::string Key = getKey(createInvocation({"-emit-obj"}));
  EXPECT_NE(Key, getKey(createInvocation({"-emit-llvm-bc"})));
  EXPECT_NE(Key, getKey(createInvocation({"-emit-obj", "-O2"})));

  CompilerInvocation OtherInput = createInvocation({"-emit-obj"});
  ObjectRef OtherRef = llvm::cantFail(CAS->storeFromString({}, "other"));
  OtherInput.getFrontendOpts().CASIncludeTreeID =
      CAS->getID(OtherRef).toString();
  EXPECT_NE(Key, getKey(OtherInput));
}

TEST_F(CompileJobCacheKeyTest, MissingInput) {
  CompilerInvocation Invocation;
  ASSERT_TRUE(CompilerInvocation::CreateFromArgs(Invocation, {}, *Diags));
  EXPECT_FALSE(createCompileJobCacheKey(*CAS, *Diags, Invocation));
  EXPECT_EQ(DiagBuffer->getNumErrors(), 1u);
}
