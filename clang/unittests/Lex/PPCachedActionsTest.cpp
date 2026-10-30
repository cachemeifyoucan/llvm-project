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
#include "clang/Lex/ModuleMap.h"
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
  using IncludeResult =
      std::variant<std::monostate, IncludeFile, IncludeModule, SpuriousImport>;

  StringRef PredefinesSource = "";
  /// The results of __has_include checks, in order.
  SmallVector<bool> HasIncludeResults;
  /// The results for #include directives, popped from the back.
  /// \c std::monostate means the directive is skipped.
  SmallVector<IncludeResult> Includes;

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

  IncludeResult
  handleIncludeDirective(Preprocessor &PP, SourceLocation IncludeLoc,
                         SourceLocation AfterDirectiveLoc) override {
    EXPECT_FALSE(Includes.empty());
    if (Includes.empty())
      return std::monostate();
    return Includes.pop_back_val();
  }

  void exitedFile(Preprocessor &PP, FileID FID) override {
    ExitedFiles.push_back(FID);
  }
};

/// Returns modules from the module map, and records the requested imports.
class RecordingModuleLoader : public TrivialModuleLoader {
public:
  std::function<Module *(StringRef)> FindModule;
  SmallVector<std::pair<std::string, bool>> Loads;

  ModuleLoadResult loadModule(SourceLocation ImportLoc, ModuleIdPath Path,
                              Module::NameVisibilityKind Visibility,
                              bool IsInclusionDirective) override {
    std::string Name;
    for (const IdentifierLoc &Component : Path) {
      if (!Name.empty())
        Name += '.';
      Name += Component.getIdentifierInfo()->getName();
    }
    Loads.emplace_back(Name, IsInclusionDirective);
    return FindModule ? FindModule(Name) : nullptr;
  }
};

/// Collects the formatted text of every diagnostic.
class CollectingDiagConsumer : public DiagnosticConsumer {
public:
  SmallVector<std::string> Messages;

  void HandleDiagnostic(DiagnosticsEngine::Level Level,
                        const Diagnostic &Info) override {
    DiagnosticConsumer::HandleDiagnostic(Level, Info);
    SmallString<64> Message;
    Info.FormatDiagnostic(Message);
    Messages.push_back(std::string(Message));
  }
};

class PPCachedActionsTest : public ::testing::Test {
protected:
  PPCachedActionsTest()
      : FileMgr(FileMgrOpts),
        Diags(DiagnosticIDs::create(), DiagOpts, &DiagConsumer,
              /*ShouldOwnClient=*/false),
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
    for (PP->Lex(Tok); Tok.isNot(tok::eof); PP->Lex(Tok)) {
      if (Tok.isAnnotation())
        Tokens.push_back(std::string("<") + tok::getTokenName(Tok.getKind()) +
                         ">");
      else
        Tokens.push_back(PP->getSpelling(Tok));
    }
    return Tokens;
  }

  FileID createFile(StringRef Contents, StringRef Name) {
    return SourceMgr.createFileID(
        llvm::MemoryBuffer::getMemBuffer(Contents, Name));
  }

  std::function<void(MockCachedActions &)> Setup = [](MockCachedActions &) {};

  /// Creates a module (and its parents) in the preprocessor's module map.
  Module *createModule(StringRef Name) {
    ModuleMap &MMap = PP->getHeaderSearchInfo().getModuleMap();
    Module *M = nullptr;
    SmallVector<StringRef> Components;
    Name.split(Components, '.');
    for (StringRef Component : Components)
      M = MMap.findOrCreateModuleFirst(Component, M, /*IsFramework=*/false,
                                       /*IsExplicit=*/false);
    return M;
  }

  IdentifierLoc ident(StringRef Name) {
    return IdentifierLoc(SourceLocation(), PP->getIdentifierInfo(Name));
  }

  CollectingDiagConsumer DiagConsumer;
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
  RecordingModuleLoader ModLoader;
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
    A.Includes = {std::monostate(),
                  PPCachedActions::IncludeFile{Header, nullptr}};
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

TEST_F(PPCachedActionsTest, IncludeModularHeader) {
  FileID Header;
  Module *Mod = nullptr;
  Setup = [&](MockCachedActions &A) {
    Header = createFile("int in_header;\n", "header.h");
    Mod = createModule("Mod.Sub");
    A.Includes = {PPCachedActions::IncludeFile{Header, Mod}};
  };
  MockCachedActions *Actions;
  std::vector<std::string> Tokens = preprocess(R"(
#include "header.h"
int in_main;
)",
                                               Actions);
  // The header is entered as part of its submodule.
  EXPECT_EQ(Tokens, (std::vector<std::string>{
                        "<annot_module_begin>", "int", "in_header", ";",
                        "<annot_module_end>", "int", "in_main", ";"}));
  EXPECT_TRUE(ModLoader.Loads.empty());
  EXPECT_FALSE(Diags.hasErrorOccurred());
}

TEST_F(PPCachedActionsTest, ImportModule) {
  Module *Mod = nullptr;
  Setup = [&](MockCachedActions &A) {
    Mod = createModule("Top.Sub");
    Mod->getTopLevelModule()->setASTFileNameAndKey(
        ModuleFileName::makeExplicit(std::string("Top.pcm")),
        ModuleFileKey(this));
    ModLoader.FindModule = [&](StringRef Name) -> Module * {
      return Name == "Top.Sub" ? Mod : nullptr;
    };
    A.Includes = {PPCachedActions::IncludeModule{{ident("Top"), ident("Sub")},
                                                 /*VisibilityOnly=*/false}};
  };
  MockCachedActions *Actions;
  std::vector<std::string> Tokens = preprocess(R"(
#include "Top/Sub.h"
int in_main;
)",
                                               Actions);
  EXPECT_EQ(Tokens, (std::vector<std::string>{"<annot_module_include>", "int",
                                              "in_main", ";"}));
  ASSERT_EQ(ModLoader.Loads.size(), 1u);
  EXPECT_EQ(ModLoader.Loads[0],
            std::make_pair(std::string("Top.Sub"), /*IsInclusion=*/true));
  EXPECT_FALSE(Diags.hasErrorOccurred());
}

TEST_F(PPCachedActionsTest, ImportModuleVisibilityOnly) {
  Setup = [&](MockCachedActions &A) {
    createModule("Top.Sub");
    A.Includes = {PPCachedActions::IncludeModule{{ident("Top"), ident("Sub")},
                                                 /*VisibilityOnly=*/true}};
  };
  MockCachedActions *Actions;
  std::vector<std::string> Tokens = preprocess(R"(
#include "Top/Sub.h"
int in_main;
)",
                                               Actions);
  // A module that is only made visible is looked up in the module map, not
  // loaded.
  EXPECT_EQ(Tokens, (std::vector<std::string>{"<annot_module_include>", "int",
                                              "in_main", ";"}));
  EXPECT_TRUE(ModLoader.Loads.empty());
  EXPECT_FALSE(Diags.hasErrorOccurred());
}

TEST_F(PPCachedActionsTest, ImportMissingModuleVisibilityOnly) {
  Setup = [&](MockCachedActions &A) {
    createModule("Top");
    A.Includes = {PPCachedActions::IncludeModule{
        {ident("Top"), ident("Missing")}, /*VisibilityOnly=*/true}};
  };
  MockCachedActions *Actions;
  preprocess(R"(
#include "Top/Missing.h"
)",
             Actions);
  EXPECT_TRUE(Diags.hasErrorOccurred());
  ASSERT_EQ(DiagConsumer.Messages.size(), 1u);
  EXPECT_NE(
      DiagConsumer.Messages[0].find("declared in include-tree module map"),
      std::string::npos)
      << DiagConsumer.Messages[0];
}

TEST_F(PPCachedActionsTest, SpuriousImport) {
  FileID Header;
  Module *Top = nullptr;
  Setup = [&](MockCachedActions &A) {
    Header = createFile("int in_header;\n", "header.h");
    Top = createModule("Top");
    ModLoader.FindModule = [&](StringRef Name) -> Module * {
      return Name == "Top" ? Top : nullptr;
    };
    A.Includes = {PPCachedActions::SpuriousImport{
        PPCachedActions::IncludeModule{{ident("Top"), ident("Missing")},
                                       /*VisibilityOnly=*/false},
        PPCachedActions::IncludeFile{Header, nullptr}}};
  };
  MockCachedActions *Actions;
  std::vector<std::string> Tokens = preprocess(R"(
#include "Top/Missing.h"
int in_main;
)",
                                               Actions);
  // The top-level module is loaded for its side effects, and the header that
  // is not part of the module is included textually.
  ASSERT_EQ(ModLoader.Loads.size(), 1u);
  EXPECT_EQ(ModLoader.Loads[0],
            std::make_pair(std::string("Top"), /*IsInclusion=*/true));
  EXPECT_EQ(Tokens, (std::vector<std::string>{"int", "in_header", ";", "int",
                                              "in_main", ";"}));
  EXPECT_FALSE(Diags.hasErrorOccurred());
  ASSERT_EQ(DiagConsumer.Messages.size(), 1u);
  EXPECT_EQ(DiagConsumer.Messages[0], "missing submodule 'Top.Missing'");
}

} // anonymous namespace
