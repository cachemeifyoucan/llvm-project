//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// PPCachedActions tests.
///
//===----------------------------------------------------------------------===//

#include "clang/Lex/PPCachedActions.h"
#include "clang/Basic/Diagnostic.h"
#include "clang/Basic/DiagnosticOptions.h"
#include "clang/Basic/FileManager.h"
#include "clang/Basic/LangOptions.h"
#include "clang/Basic/SourceManager.h"
#include "clang/Basic/TargetInfo.h"
#include "clang/Basic/TargetOptions.h"
#include "clang/Lex/HeaderSearch.h"
#include "clang/Lex/HeaderSearchOptions.h"
#include "clang/Lex/ModuleLoader.h"
#include "clang/Lex/Preprocessor.h"
#include "clang/Lex/PreprocessorOptions.h"
#include "gtest/gtest.h"
#include <functional>

using namespace clang;

namespace {

/// Replays a fixed set of answers to the preprocessor, without looking up any
/// file.
class MockCachedActions : public PPCachedActions {
public:
  StringRef PredefinesSource = "";
  /// The results of __has_include checks, in order.
  SmallVector<bool> HasIncludeResults;
  /// The files to enter for #include directives, in order. An invalid FileID
  /// means the directive is skipped.
  SmallVector<FileID> Includes;

  // Recorded calls.
  unsigned NumPredefines = 0;
  SmallVector<bool> HasIncludeIsNext;
  SmallVector<FileID> ExitedFiles;

  FileID handlePredefines(Preprocessor &PP) override {
    ++NumPredefines;
    return PP.getSourceManager().createFileID(
        llvm::MemoryBuffer::getMemBuffer(PredefinesSource, "<built-in>"));
  }

  bool evaluateHasInclude(Preprocessor &PP, SourceLocation Loc,
                          bool IsIncludeNext) override {
    HasIncludeIsNext.push_back(IsIncludeNext);
    EXPECT_FALSE(HasIncludeResults.empty());
    if (HasIncludeResults.empty())
      return false;
    return HasIncludeResults.pop_back_val();
  }

  std::variant<std::monostate, IncludeFile>
  handleIncludeDirective(Preprocessor &PP, SourceLocation IncludeLoc,
                         SourceLocation AfterDirectiveLoc) override {
    EXPECT_FALSE(Includes.empty());
    if (Includes.empty())
      return std::monostate();
    FileID FID = Includes.pop_back_val();
    if (FID.isInvalid())
      return std::monostate();
    return IncludeFile{FID};
  }

  void exitedFile(Preprocessor &PP, FileID FID) override {
    ExitedFiles.push_back(FID);
  }
};

class PPCachedActionsTest : public ::testing::Test {
protected:
  PPCachedActionsTest()
      : FileMgr(FileMgrOpts),
        Diags(DiagnosticIDs::create(), DiagOpts, new IgnoringDiagConsumer()),
        SourceMgr(Diags, FileMgr), TargetOpts(new TargetOptions) {
    TargetOpts->Triple = "x86_64-apple-darwin11.1.0";
    Target = TargetInfo::CreateTargetInfo(Diags, *TargetOpts);
  }

  /// Preprocess \p Source with \p Actions and return the spelling of each
  /// token.
  std::vector<std::string> preprocess(StringRef Source,
                                      MockCachedActions *&Actions) {
    SourceMgr.setMainFileID(
        SourceMgr.createFileID(llvm::MemoryBuffer::getMemBuffer(Source)));

    HeaderInfo = std::make_unique<HeaderSearch>(HSOpts, SourceMgr, Diags,
                                                LangOpts, Target.get());
    PP = std::make_unique<Preprocessor>(PPOpts, Diags, LangOpts, SourceMgr,
                                        *HeaderInfo, ModLoader,
                                        /*IILookup=*/nullptr,
                                        /*OwnsHeaderSearch=*/false);
    PP->Initialize(*Target);
    auto OwnedActions = std::make_unique<MockCachedActions>();
    Actions = OwnedActions.get();
    Setup(*Actions);
    PP->setPPCachedActions(std::move(OwnedActions));
    PP->EnterMainSourceFile();

    std::vector<std::string> Tokens;
    Token Tok;
    for (PP->Lex(Tok); Tok.isNot(tok::eof); PP->Lex(Tok))
      Tokens.push_back(PP->getSpelling(Tok));
    return Tokens;
  }

  FileID createFile(StringRef Contents, StringRef Name) {
    return SourceMgr.createFileID(
        llvm::MemoryBuffer::getMemBuffer(Contents, Name));
  }

  std::function<void(MockCachedActions &)> Setup = [](MockCachedActions &) {};

  FileSystemOptions FileMgrOpts;
  FileManager FileMgr;
  DiagnosticOptions DiagOpts;
  DiagnosticsEngine Diags;
  SourceManager SourceMgr;
  LangOptions LangOpts;
  std::shared_ptr<TargetOptions> TargetOpts;
  IntrusiveRefCntPtr<TargetInfo> Target;
  HeaderSearchOptions HSOpts;
  PreprocessorOptions PPOpts;
  TrivialModuleLoader ModLoader;
  std::unique_ptr<HeaderSearch> HeaderInfo;
  std::unique_ptr<Preprocessor> PP;
};

TEST_F(PPCachedActionsTest, Predefines) {
  Setup = [](MockCachedActions &A) {
    A.PredefinesSource = "#define FROM_CACHED_PREDEFINES 42\n";
  };
  MockCachedActions *Actions;
  std::vector<std::string> Tokens =
      preprocess("int x = FROM_CACHED_PREDEFINES;\n", Actions);
  EXPECT_EQ(Actions->NumPredefines, 1u);
  EXPECT_EQ(Tokens, (std::vector<std::string>{"int", "x", "=", "42", ";"}));
  EXPECT_FALSE(Diags.hasErrorOccurred());
}

TEST_F(PPCachedActionsTest, HasInclude) {
  // The results are the opposite of what a real file lookup would give, since
  // none of these files exist. The results are popped from the back.
  Setup = [](MockCachedActions &A) {
    A.HasIncludeResults = {/*third=*/false, /*second=*/true, /*first=*/false};
  };
  MockCachedActions *Actions;
  std::vector<std::string> Tokens = preprocess(R"(
#if __has_include("first.h")
int first;
#endif
#if __has_include(<second.h>)
int second;
#endif
#if __has_include_next("third.h")
int third;
#endif
)",
                                               Actions);
  EXPECT_EQ(Tokens, (std::vector<std::string>{"int", "second", ";"}));
  EXPECT_EQ(Actions->HasIncludeIsNext, (SmallVector<bool>{false, false, true}));
  // __has_include_next in the main file would normally warn; the cached
  // actions answer for it.
  EXPECT_EQ(Diags.getNumWarnings(), 0u);
  EXPECT_FALSE(Diags.hasErrorOccurred());
}

TEST_F(PPCachedActionsTest, IncludeDirective) {
  FileID Header;
  Setup = [&](MockCachedActions &A) {
    Header = createFile("int in_header;\n", "header.h");
    // The second #include is skipped. The results are popped from the back.
    A.Includes = {FileID(), Header};
  };
  MockCachedActions *Actions;
  std::vector<std::string> Tokens = preprocess(R"(
#include "header.h"
#include "skipped.h"
int in_main;
)",
                                               Actions);
  EXPECT_EQ(Tokens, (std::vector<std::string>{"int", "in_header", ";", "int",
                                              "in_main", ";"}));
  ASSERT_GE(Actions->ExitedFiles.size(), 1u);
  EXPECT_TRUE(llvm::is_contained(Actions->ExitedFiles, Header));
  // Neither file exists, so a lookup would have been a fatal error.
  EXPECT_FALSE(Diags.hasErrorOccurred());
}

} // anonymous namespace
