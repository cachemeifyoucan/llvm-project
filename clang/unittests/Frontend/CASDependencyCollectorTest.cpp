//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "clang/Frontend/CASDependencyCollector.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/CompilerInvocation.h"
#include "clang/Frontend/FrontendActions.h"
#include "clang/Frontend/TextDiagnosticBuffer.h"
#include "llvm/CAS/CASOutputBackend.h"
#include "llvm/CAS/ObjectStore.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/VirtualFileSystem.h"
#include "llvm/Testing/Support/Error.h"
#include "gtest/gtest.h"

using namespace clang;
using llvm::Succeeded;

namespace {

/// Preprocess \p MainFile from \p FS with a \c CASDependencyCollector attached
/// and return the resulting dependency list.
std::optional<llvm::cas::ObjectRef>
collectDependencies(llvm::cas::ObjectStore &CAS,
                    IntrusiveRefCntPtr<llvm::vfs::FileSystem> FS,
                    StringRef MainFile, const DependencyOutputOptions &Opts) {
  auto Invocation = std::make_shared<CompilerInvocation>();
  Invocation->getFrontendOpts().Inputs.emplace_back(MainFile, Language::C);
  Invocation->getFrontendOpts().ProgramAction = frontend::RunPreprocessorOnly;
  Invocation->getTargetOpts().Triple = "x86_64-apple-macos11";

  CompilerInstance CI(std::move(Invocation));
  CI.createVirtualFileSystem(FS);
  CI.createDiagnostics(new TextDiagnosticBuffer(), /*ShouldOwnClient=*/true);

  std::optional<llvm::cas::ObjectRef> Result;
  bool CallbackCalled = false;
  CI.addDependencyCollector(std::make_shared<CASDependencyCollector>(
      Opts, CAS, [&](std::optional<llvm::cas::ObjectRef> Deps) {
        CallbackCalled = true;
        Result = Deps;
      }));

  PreprocessOnlyAction Action;
  EXPECT_TRUE(CI.ExecuteAction(Action));
  EXPECT_TRUE(CallbackCalled);
  return Result;
}

std::string replay(llvm::cas::ObjectStore &CAS, llvm::cas::ObjectRef Deps,
                   const DependencyOutputOptions &Opts) {
  std::optional<llvm::cas::ObjectProxy> Proxy;
  EXPECT_THAT_ERROR(CAS.getProxy(Deps).moveInto(Proxy), Succeeded());
  std::string Output;
  llvm::raw_string_ostream OS(Output);
  EXPECT_THAT_ERROR(CASDependencyCollector::replay(Opts, CAS, *Proxy, OS),
                    Succeeded());
  return Output;
}

TEST(CASDependencyCollectorTest, StoreAndReplay) {
  std::unique_ptr<llvm::cas::ObjectStore> CAS = llvm::cas::createInMemoryCAS();
  auto FS = llvm::makeIntrusiveRefCnt<llvm::vfs::InMemoryFileSystem>();
  FS->addFile("/t/main.c", 0,
              llvm::MemoryBuffer::getMemBuffer("#include \"a.h\"\n"));
  FS->addFile("/t/a.h", 0, llvm::MemoryBuffer::getMemBuffer("int a;\n"));

  DependencyOutputOptions Opts;
  Opts.OutputFile = "/t/main.d";
  Opts.Targets = {"main.o"};
  Opts.ExtraDeps = {{"/t/extra.txt", EDK_DepFileEntry}};

  std::optional<llvm::cas::ObjectRef> Deps =
      collectDependencies(*CAS, FS, "/t/main.c", Opts);
  ASSERT_TRUE(Deps);

  // The extra dependencies are not stored in the CAS; they are added back
  // during replay from the options passed to it.
  std::optional<llvm::cas::ObjectProxy> Proxy;
  ASSERT_THAT_ERROR(CAS->getProxy(*Deps).moveInto(Proxy), Succeeded());
  std::vector<std::string> Stored;
  ASSERT_THAT_ERROR(
      Proxy->forEachReference([&](llvm::cas::ObjectRef Ref) -> llvm::Error {
        auto Path = CAS->getProxy(Ref);
        if (!Path)
          return Path.takeError();
        Stored.push_back(Path->getData().str());
        return llvm::Error::success();
      }),
      Succeeded());
  EXPECT_EQ(Stored, (std::vector<std::string>{"/t/main.c", "/t/a.h"}));

  EXPECT_EQ(replay(*CAS, *Deps, Opts),
            "main.o: /t/extra.txt /t/main.c /t/a.h\n");

  // Replay with different output options.
  DependencyOutputOptions ReplayOpts;
  ReplayOpts.Targets = {"other.o"};
  ReplayOpts.UsePhonyTargets = true;
  EXPECT_EQ(replay(*CAS, *Deps, ReplayOpts),
            "other.o: /t/main.c /t/a.h\n/t/a.h:\n");
}

TEST(DependencyFileGeneratorTest, WritesThroughOutputBackend) {
  std::unique_ptr<llvm::cas::ObjectStore> CAS = llvm::cas::createInMemoryCAS();
  auto FS = llvm::makeIntrusiveRefCnt<llvm::vfs::InMemoryFileSystem>();
  FS->addFile("/t/main.c", 0,
              llvm::MemoryBuffer::getMemBuffer("#include \"a.h\"\n"));
  FS->addFile("/t/a.h", 0, llvm::MemoryBuffer::getMemBuffer("int a;\n"));

  auto Invocation = std::make_shared<CompilerInvocation>();
  Invocation->getFrontendOpts().Inputs.emplace_back("/t/main.c", Language::C);
  Invocation->getFrontendOpts().ProgramAction = frontend::RunPreprocessorOnly;
  Invocation->getTargetOpts().Triple = "x86_64-apple-macos11";
  Invocation->getDependencyOutputOpts().OutputFile = "/t/main.d";
  Invocation->getDependencyOutputOpts().Targets = {"main.o"};

  CompilerInstance CI(std::move(Invocation));
  CI.createVirtualFileSystem(FS);
  CI.createDiagnostics(new TextDiagnosticBuffer(), /*ShouldOwnClient=*/true);
  auto Outputs = llvm::makeIntrusiveRefCnt<llvm::cas::CASOutputBackend>(*CAS);
  CI.setOutputManager(Outputs);

  PreprocessOnlyAction Action;
  ASSERT_TRUE(CI.ExecuteAction(Action));

  // The dependency file went through the instance's output manager rather
  // than directly to disk.
  SmallVector<llvm::cas::CASOutputBackend::OutputFile> Files =
      Outputs->takeOutputs();
  ASSERT_EQ(Files.size(), 1u);
  EXPECT_EQ(Files[0].Path, "/t/main.d");
  std::optional<llvm::cas::ObjectProxy> Contents;
  ASSERT_THAT_ERROR(CAS->getProxy(Files[0].Object).moveInto(Contents),
                    Succeeded());
  EXPECT_EQ(Contents->getData(), "main.o: /t/main.c /t/a.h\n");
}

} // namespace
