//===- unittests/Frontend/CompilerInstanceTest.cpp - CI tests -------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "clang/Frontend/CompilerInstance.h"
#include "clang/Basic/FileManager.h"
#include "clang/Driver/CreateInvocationFromArgs.h"
#include "clang/Frontend/CompilerInvocation.h"
#include "clang/Frontend/FrontendActions.h"
#include "clang/Frontend/TextDiagnosticBuffer.h"
#include "clang/Frontend/TextDiagnosticPrinter.h"
#include "clang/Lex/PPCallbacks.h"
#include "clang/Lex/Preprocessor.h"
#include "clang/Lex/PreprocessorOptions.h"
#include "llvm/ADT/IntrusiveRefCntPtr.h"
#include "llvm/CAS/ActionCache.h"
#include "llvm/CAS/ObjectStore.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/Format.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/ToolOutputFile.h"
#include "llvm/Support/VirtualFileSystem.h"
#include "llvm/Testing/Support/SupportHelpers.h"
#include "gtest/gtest.h"

using namespace llvm;
using namespace clang;

namespace {

TEST(CompilerInstance, DefaultVFSOverlayFromInvocation) {
  // Create a temporary VFS overlay yaml file.
  int FD;
  SmallString<256> FileName;
  ASSERT_FALSE(sys::fs::createTemporaryFile("vfs", "yaml", FD, FileName));
  ToolOutputFile File(FileName, FD);

  SmallString<256> CurrentPath;
  sys::fs::current_path(CurrentPath);
  sys::path::make_absolute(CurrentPath, FileName);

  // Mount the VFS file itself on the path 'virtual.file'. Makes this test
  // a bit shorter than creating a new dummy file just for this purpose.
  const std::string CurrentPathStr = std::string(CurrentPath.str());
  const std::string FileNameStr = std::string(FileName.str());
  const char *VFSYaml = "{ 'version': 0, 'roots': [\n"
                        "  { 'name': '%s',\n"
                        "    'type': 'directory',\n"
                        "    'contents': [\n"
                        "      { 'name': 'vfs-virtual.file', 'type': 'file',\n"
                        "        'external-contents': '%s'\n"
                        "      }\n"
                        "    ]\n"
                        "  }\n"
                        "]}\n";
  File.os() << format(VFSYaml, CurrentPathStr.c_str(), FileName.c_str());
  File.os().flush();

  // Create a CompilerInvocation that uses this overlay file.
  const std::string VFSArg = "-ivfsoverlay" + FileNameStr;
  const char *Args[] = {"clang", VFSArg.c_str(), "-xc++", "-"};

  DiagnosticOptions DiagOpts;
  IntrusiveRefCntPtr<DiagnosticsEngine> Diags =
      CompilerInstance::createDiagnostics(*llvm::vfs::getRealFileSystem(),
                                          DiagOpts);

  CreateInvocationOptions CIOpts;
  CIOpts.Diags = Diags;
  std::shared_ptr<CompilerInvocation> CInvok =
      createInvocation(Args, std::move(CIOpts));

  if (!CInvok)
    FAIL() << "could not create compiler invocation";
  // Create a minimal CompilerInstance which should use the VFS we specified
  // in the CompilerInvocation (as we don't explicitly set our own).
  CompilerInstance Instance(std::move(CInvok));
  Instance.setDiagnostics(Diags);
  Instance.createVirtualFileSystem();
  Instance.createFileManager();

  // Check if the virtual file exists which means that our VFS is used by the
  // CompilerInstance.
  ASSERT_TRUE(Instance.getFileManager().getOptionalFileRef("vfs-virtual.file"));
}

TEST(CompilerInstance, CreateVFSWithoutDiagnosticConsumer) {
  auto Invocation = std::make_shared<CompilerInvocation>();
  Invocation->getHeaderSearchOpts().VFSOverlayFiles.push_back("/missing.yaml");
  auto BaseFS = llvm::makeIntrusiveRefCnt<llvm::vfs::InMemoryFileSystem>();
  CompilerInstance Instance(std::move(Invocation));
  // Check that omitting the DiagnosticConsumer doesn't crash (e.g. by
  // dereferencing the null pointer).
  ASSERT_NO_FATAL_FAILURE(
      Instance.createVirtualFileSystem(std::move(BaseFS), /*DC=*/nullptr));
}

TEST(CompilerInstance, AllowDiagnosticLogWithUnownedDiagnosticConsumer) {
  DiagnosticOptions DiagOpts;
  // Tell the diagnostics engine to emit the diagnostic log to STDERR. This
  // ensures that a chained diagnostic consumer is created so that the test can
  // exercise the unowned diagnostic consumer in a chained consumer.
  DiagOpts.DiagnosticLogFile = "-";

  // Create the diagnostic engine with unowned consumer.
  std::string DiagnosticOutput;
  llvm::raw_string_ostream DiagnosticsOS(DiagnosticOutput);
  auto DiagPrinter =
      std::make_unique<TextDiagnosticPrinter>(DiagnosticsOS, DiagOpts);
  IntrusiveRefCntPtr<DiagnosticsEngine> Diags =
      CompilerInstance::createDiagnostics(*llvm::vfs::getRealFileSystem(),
                                          DiagOpts, DiagPrinter.get(),
                                          /*ShouldOwnClient=*/false);

  Diags->Report(diag::err_expected) << "no crash";
  ASSERT_EQ(DiagnosticOutput, "error: expected no crash\n");
}

TEST(CompilerInstance, CASDatabases) {
  auto Invocation = std::make_shared<CompilerInvocation>();
  CompilerInstance Instance(std::move(Invocation));
  Instance.createDiagnostics(new IgnoringDiagConsumer());

  // Nothing is created until requested.
  EXPECT_EQ(Instance.getCASDatabases().first, nullptr);
  EXPECT_EQ(Instance.getCASDatabases().second, nullptr);

  // The default configuration is an in-memory CAS, and it is created once.
  auto [CAS, Cache] = Instance.getOrCreateCASDatabases();
  ASSERT_NE(CAS, nullptr);
  ASSERT_NE(Cache, nullptr);
  EXPECT_EQ(&Instance.getOrCreateObjectStore(), CAS.get());
  EXPECT_EQ(&Instance.getOrCreateActionCache(), Cache.get());
  EXPECT_EQ(Instance.getCASDatabases(), std::make_pair(CAS, Cache));
  EXPECT_FALSE(Instance.getDiagnostics().hasErrorOccurred());
}

TEST(CompilerInstance, SetCASDatabases) {
  auto Invocation = std::make_shared<CompilerInvocation>();
  CompilerInstance Instance(std::move(Invocation));
  Instance.createDiagnostics(new IgnoringDiagConsumer());

  std::shared_ptr<llvm::cas::ObjectStore> CAS = llvm::cas::createInMemoryCAS();
  std::shared_ptr<llvm::cas::ActionCache> Cache =
      llvm::cas::createInMemoryActionCache();
  Instance.setCASDatabases(CAS, Cache);
  EXPECT_EQ(Instance.getOrCreateCASDatabases(), std::make_pair(CAS, Cache));
}

TEST(CompilerInstance, CASDatabasesInvalidPath) {
  if (!llvm::cas::isOnDiskCASEnabled())
    GTEST_SKIP() << "on-disk CAS is not enabled";

  // Put a regular file where the CAS should be.
  unittest::TempDir Dir("cas-invalid", /*Unique=*/true);
  unittest::TempFile File(Dir.path("cas"), /*Suffix=*/"",
                          /*Contents=*/"contents");

  auto Invocation = std::make_shared<CompilerInvocation>();
  Invocation->getCASOpts().CASPath = File.path().str();
  CompilerInstance Instance(std::move(Invocation));
  auto *Diags = new TextDiagnosticBuffer();
  Instance.createDiagnostics(Diags);

  // Opening the CAS fails with a diagnostic, and falls back to empty in-memory
  // databases so that callers do not need to check for null.
  auto [CAS, Cache] = Instance.getOrCreateCASDatabases();
  EXPECT_NE(CAS, nullptr);
  EXPECT_NE(Cache, nullptr);
  ASSERT_EQ(Diags->getNumErrors(), 1u);
  EXPECT_NE(Diags->err_begin()->second.find("CAS cannot be initialized"),
            std::string::npos);
}

TEST(CompilerInstance, MultipleInputsCleansFileIDs) {
  auto VFS = makeIntrusiveRefCnt<llvm::vfs::InMemoryFileSystem>();
  VFS->addFile("a.cc", /*ModificationTime=*/{},
               MemoryBuffer::getMemBuffer(R"cpp(
      #include "a.h"
      )cpp"));
  // Paddings of `void foo();` in the sources below are "important". We're
  // testing against source locations from previous compilations colliding.
  // Hence the `unused` variable in `b.h` needs to be within `#pragma clang
  // diagnostic` block from `a.h`.
  VFS->addFile("a.h", /*ModificationTime=*/{}, MemoryBuffer::getMemBuffer(R"cpp(
      #include "b.h"
      #pragma clang diagnostic push
      #pragma clang diagnostic warning "-Wunused"
      void foo();
      #pragma clang diagnostic pop
      )cpp"));
  VFS->addFile("b.h", /*ModificationTime=*/{}, MemoryBuffer::getMemBuffer(R"cpp(
      void foo(); void foo(); void foo(); void foo();
      inline void foo() { int unused = 2; }
      )cpp"));

  DiagnosticOptions DiagOpts;
  IntrusiveRefCntPtr<DiagnosticsEngine> Diags =
      CompilerInstance::createDiagnostics(*VFS, DiagOpts);

  CreateInvocationOptions CIOpts;
  CIOpts.Diags = Diags;

  const char *Args[] = {"clang", "-xc++", "a.cc"};
  std::shared_ptr<CompilerInvocation> CInvok =
      createInvocation(Args, std::move(CIOpts));
  ASSERT_TRUE(CInvok) << "could not create compiler invocation";

  CompilerInstance Instance(std::move(CInvok));
  Instance.setVirtualFileSystem(VFS);
  Instance.setDiagnostics(Diags);
  Instance.createFileManager();

  // Run once for `a.cc` and then for `a.h`. This makes sure we get the same
  // file ID for `b.h` in the second run as `a.h` from first run.
  const auto &OrigInputKind = Instance.getFrontendOpts().Inputs[0].getKind();
  Instance.getFrontendOpts().Inputs.emplace_back("a.h", OrigInputKind);

  SyntaxOnlyAction Act;
  EXPECT_TRUE(Instance.ExecuteAction(Act)) << "Failed to execute action";
  EXPECT_FALSE(Diags->hasErrorOccurred());
  EXPECT_EQ(Diags->getNumWarnings(), 0u);
}

TEST(CompilerInstance, SingleModuleParseModeCallback) {
  auto VFS = makeIntrusiveRefCnt<vfs::InMemoryFileSystem>();
  VFS->addFile("module.modulemap", 0,
               MemoryBuffer::getMemBuffer(R"(module m { header "m.h" })"));
  VFS->addFile("m.h", 0, MemoryBuffer::getMemBuffer(""));
  VFS->addFile("tu.c", 0, MemoryBuffer::getMemBuffer(R"(#include "m.h")"));

  auto Invocation = createInvocation(
      {"clang", "tu.c", "-fmodules", "-fmodules-cache-path=/"});
  Invocation->getPreprocessorOpts().SingleModuleParseMode = true;

  CompilerInstance Instance(std::move(Invocation));
  Instance.createVirtualFileSystem(VFS);
  Instance.createFileManager();
  Instance.createDiagnostics();

  struct ModuleLoadSkippedCallback : PPCallbacks {
    std::vector<std::string> &SkippedModules;
    ModuleLoadSkippedCallback(std::vector<std::string> &SkippedModules)
        : SkippedModules(SkippedModules) {}
    void moduleLoadSkipped(clang::Module *Skipped) override {
      SkippedModules.emplace_back(Skipped->getFullModuleName());
    }
  };

  struct MyAction : SyntaxOnlyAction {
    std::vector<std::string> &SkippedModules;
    MyAction(std::vector<std::string> &SkippedModules)
        : SkippedModules(SkippedModules) {}
    bool BeginSourceFileAction(CompilerInstance &CI) override {
      auto Cb = std::make_unique<ModuleLoadSkippedCallback>(SkippedModules);
      CI.getPreprocessor().addPPCallbacks(std::move(Cb));
      return SyntaxOnlyAction::BeginInvocation(CI);
    }
  };

  std::vector<std::string> SkippedModules;
  MyAction Action(SkippedModules);
  bool Success = Instance.ExecuteAction(Action);

  ASSERT_TRUE(Success);
  ASSERT_EQ(SkippedModules.size(), 1u);
  EXPECT_EQ(SkippedModules[0], "m");
}
} // anonymous namespace
