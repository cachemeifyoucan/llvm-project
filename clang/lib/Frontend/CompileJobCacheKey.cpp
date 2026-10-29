//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "clang/Frontend/CompileJobCacheKey.h"
#include "clang/Basic/DiagnosticCAS.h"
#include "clang/Basic/Version.h"
#include "clang/CAS/IncludeTree.h"
#include "clang/Frontend/CompilerInvocation.h"
#include "llvm/CAS/ActionCache.h"
#include "llvm/CAS/NamedValuesSchema.h"
#include "llvm/CAS/ObjectStore.h"
#include "llvm/Support/StringSaver.h"
#include "llvm/Support/raw_ostream.h"
#include <optional>

using namespace clang;
using namespace clang::cas;
using namespace llvm;
using namespace llvm::cas;

template <typename R>
static std::optional<R> checkStore(DiagnosticsEngine &Diags, Expected<R> Ref) {
  if (Ref)
    return *Ref;
  Diags.Report(diag::err_cas_store) << Ref.takeError();
  return std::nullopt;
}

static std::optional<llvm::cas::CASID> createCompileJobCacheKeyForArgs(
    ObjectStore &CAS, DiagnosticsEngine &Diags, ArrayRef<const char *> CC1Args,
    ObjectRef InputRef, CachingInputKind InputKind) {
  assert(!CC1Args.empty() && StringRef(CC1Args[0]) == "-cc1");
  SmallString<256> CommandLine;
  for (StringRef Arg : CC1Args) {
    CommandLine.append(Arg);
    CommandLine.push_back(0);
  }

  llvm::cas::NamedValuesSchema::Builder Builder(CAS);
  switch (InputKind) {
  case CachingInputKind::IncludeTree:
    Builder.add("include-tree", InputRef);
    break;
  }

  auto CommandLineRef = checkStore(Diags, CAS.storeFromString({}, CommandLine));
  if (!CommandLineRef)
    return std::nullopt;
  auto CC1Ref = checkStore(Diags, CAS.storeFromString({}, "-cc1"));
  if (!CC1Ref)
    return std::nullopt;
  auto VersionRef =
      checkStore(Diags, CAS.storeFromString({}, getClangFullVersion()));
  if (!VersionRef)
    return std::nullopt;

  Builder.add("command-line", *CommandLineRef);
  Builder.add("computation", *CC1Ref);
  // FIXME: The version is maybe insufficient...
  Builder.add("version", *VersionRef);

  auto Result = checkStore(Diags, Builder.build());
  if (!Result)
    return std::nullopt;
  return Result->getID();
}

static void canonicalizeForCacheKey(FrontendOptions &FrontendOpts,
                                    DependencyOutputOptions &DepOpts,
                                    DiagnosticOptions &DiagOpts) {
  // Keep the key independent of the paths of these outputs.
  if (!FrontendOpts.OutputFile.empty())
    FrontendOpts.OutputFile = "-";
  if (!DepOpts.OutputFile.empty())
    DepOpts.OutputFile = "-";
  // Dependency options that do not affect the list of files are canonicalized.
  if (!DepOpts.Targets.empty())
    DepOpts.Targets = {"-"};
  DepOpts.UsePhonyTargets = false;
  // These are added in when the dependency file is generated, but they don't
  // affect the actual compilation.
  DepOpts.ExtraDeps.clear();

  // Canonicalize diagnostic options.

  // These options affect diagnostic rendering but not the cached diagnostics.
  DiagOpts.ShowLine = false;
  DiagOpts.ShowColumn = false;
  DiagOpts.ShowLocation = false;
  DiagOpts.ShowLevel = false;
  DiagOpts.AbsolutePath = false;
  DiagOpts.ShowCarets = false;
  DiagOpts.ShowFixits = false;
  DiagOpts.ShowSourceRanges = false;
  DiagOpts.ShowParseableFixits = false;
  DiagOpts.ShowPresumedLoc = false;
  DiagOpts.ShowOptionNames = false;
  DiagOpts.ShowNoteIncludeStack = false;
  DiagOpts.ShowCategories = false;
  DiagOpts.setShowColors(ShowColorsKind::Off);
  DiagOpts.UseANSIEscapeCodes = false;
  DiagOpts.VerifyDiagnostics = false;
  DiagOpts.setVerifyIgnoreUnexpected(DiagnosticLevelMask::None);
  // Note: ErrorLimit affects the set of diagnostics emitted, but since we do
  // not cache compilation failures, it's safe to clear here.
  DiagOpts.ErrorLimit = 0;
  DiagOpts.MacroBacktraceLimit = 0;
  DiagOpts.SnippetLineLimit = 0;
  DiagOpts.TabStop = 0;
  DiagOpts.MessageLength = 0;
  DiagOpts.DiagnosticLogFile.clear();
  DiagOpts.DiagnosticSerializationFile.clear();
  DiagOpts.VerifyPrefixes.clear();

  llvm::erase_if(DiagOpts.Remarks, [](StringRef Remark) -> bool {
    // These are intended for caching introspection, they are not cached.
    return Remark.starts_with("compile-job-cache");
  });
}

static std::optional<llvm::cas::CASID>
createCompileJobCacheKeyImpl(ObjectStore &CAS, DiagnosticsEngine &Diags,
                             CowCompilerInvocation CI) {
  canonicalizeForCacheKey(CI.getMutFrontendOpts(),
                          CI.getMutDependencyOutputOpts(),
                          CI.getMutDiagnosticOpts());
  // Generate a new command-line in case Invocation has been canonicalized.
  llvm::BumpPtrAllocator Alloc;
  llvm::StringSaver Saver(Alloc);
  llvm::SmallVector<const char *> Argv;
  Argv.push_back("-cc1");
  CI.generateCC1CommandLine(
      Argv, [&Saver](const llvm::Twine &T) { return Saver.save(T).data(); });

  // FIXME: currently correct since the main executable is always in the root
  // from scanning, but we should probably make it explicit here...
  StringRef InputIDString;
  CachingInputKind InputKind;
  if (!CI.getFrontendOpts().CASIncludeTreeID.empty()) {
    InputIDString = CI.getFrontendOpts().CASIncludeTreeID;
    InputKind = CachingInputKind::IncludeTree;
  } else {
    Diags.Report(diag::err_cas_cannot_parse_root_id) << "";
    return std::nullopt;
  }

  Expected<llvm::cas::CASID> InputID = CAS.parseID(InputIDString);
  if (!InputID) {
    llvm::consumeError(InputID.takeError());
    Diags.Report(diag::err_cas_cannot_parse_root_id) << InputIDString;
    return std::nullopt;
  }
  std::optional<llvm::cas::ObjectRef> InputRef = CAS.getReference(*InputID);
  if (!InputRef) {
    Diags.Report(diag::err_cas_missing_root_id) << InputIDString;
    return std::nullopt;
  }

  return createCompileJobCacheKeyForArgs(CAS, Diags, Argv, *InputRef,
                                         InputKind);
}

static CompileJobCachingOptions
canonicalizeForCaching(const llvm::cas::ObjectStore &CAS,
                       FrontendOptions &FrontendOpts, CASOptions &CASOpts) {
  CompileJobCachingOptions Opts;

  // Canonicalize settings for caching, extracting settings that affect the
  // compilation even if they will be cleared during the main compilation.
  FrontendOpts.CacheCompileJob = false;
  Opts.DisableCachedCompileJobReplay =
      FrontendOpts.DisableCachedCompileJobReplay;
  FrontendOpts.DisableCachedCompileJobReplay = false;
  FrontendOpts.IncludeTimestamps = false;

  // Hide the CAS configuration, canonicalizing it to keep the path to the
  // CAS from leaking to the compile job, where it might affect its
  // output (e.g., in a diagnostic).
  //
  // TODO: Extract CASOptions.Path first if we need it later since it'll
  // disappear here.
  CASOpts = {};
  // Set the CASPath to the hash schema.
  CASOpts.CASPath = CAS.getContext().getHashSchemaIdentifier().str();

  // TODO: Canonicalize DiagnosticOptions here to be "serialized" only. Pass in
  // a hook to mirror diagnostics to stderr (when writing there), and handle
  // other outputs during replay.

  // TODO: migrate the output path changes from createCompileJobCacheKey here.
  return Opts;
}

std::optional<std::pair<llvm::cas::CASID, llvm::cas::CASID>>
clang::canonicalizeAndCreateCacheKeys(ObjectStore &CAS, ActionCache &Cache,
                                      DiagnosticsEngine &Diags,
                                      CompilerInvocation &CI,
                                      CompileJobCachingOptions &Opts) {
  Opts = canonicalizeForCaching(CAS, CI.getFrontendOpts(), CI.getCASOpts());
  auto CacheKey = CI.withCowRef<std::optional<CASID>>(
      [&](const CowCompilerInvocation &CowCI) {
        return createCompileJobCacheKeyImpl(CAS, Diags, std::move(CowCI));
      });
  if (!CacheKey)
    return std::nullopt;

  return std::make_pair(*CacheKey, *CacheKey);
}

std::optional<llvm::cas::CASID>
clang::createCompileJobCacheKey(ObjectStore &CAS, DiagnosticsEngine &Diags,
                                const CompilerInvocation &OriginalCI) {
  return OriginalCI.withCowRef<std::optional<CASID>>(
      [&](const CowCompilerInvocation &CowOriginalCI) {
        return createCompileJobCacheKey(CAS, Diags, CowOriginalCI);
      });
}

std::optional<llvm::cas::CASID>
clang::createCompileJobCacheKey(ObjectStore &CAS, DiagnosticsEngine &Diags,
                                CowCompilerInvocation CI) {
  (void)canonicalizeForCaching(CAS, CI.getMutFrontendOpts(),
                               CI.getMutCASOpts());
  return createCompileJobCacheKeyImpl(CAS, Diags, std::move(CI));
}

static Error printCompileJobCacheKey(ObjectStore &CAS, ObjectProxy Node,
                                     raw_ostream &OS) {
  auto strError = [](const Twine &Err) {
    return createStringError(inconvertibleErrorCode(), Err);
  };

  auto Schema = NamedValuesSchema::create(CAS);
  if (!Schema)
    return Schema.takeError();
  Expected<NamedValuesProxy> Proxy = Schema->load(Node);
  if (!Proxy)
    return Proxy.takeError();

  // Not exhaustive, but quick check that this looks like a cache key.
  if (!Proxy->lookup("computation"))
    return strError("cas object is not a valid cache key");

  return Proxy->forEachEntry([&](const NamedValuesEntry &E) -> Error {
    OS << E.Name << ": " << CAS.getID(E.Ref);

    if (E.Name == "include-tree") {
      auto IncludeTree = IncludeTreeRoot::get(CAS, E.Ref);
      if (!IncludeTree)
        return IncludeTree.takeError();
      OS << '\n';
      return IncludeTree->print(OS, 2);
    }

    auto Blob = CAS.getProxy(E.Ref);
    if (!Blob)
      return Blob.takeError();

    auto Data = Blob->getData();
    if (E.Name == "command-line") {
      StringRef Arg;
      StringRef Trailing;
      do {
        std::tie(Arg, Data) = Data.split('\0');
        if (Arg.starts_with("-")) {
          OS << Trailing << "\n  " << Arg;
        } else {
          OS << " " << Arg;
        }
        Trailing = " \\";
      } while (!Data.empty());
    } else {
      OS << "\n  " << Data;
    }
    OS << "\n";
    return Error::success();
  });
}

Error clang::printCompileJobCacheKey(ObjectStore &CAS, const CASID &Key,
                                     raw_ostream &OS) {
  auto H = CAS.getProxy(Key);
  if (!H)
    return H.takeError();

  return ::printCompileJobCacheKey(CAS, *H, OS);
}
