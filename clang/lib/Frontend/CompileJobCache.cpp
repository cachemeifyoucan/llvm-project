//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "clang/Frontend/CompileJobCache.h"
#include "CachedDiagnostics.h"
#include "clang/Basic/DiagnosticCAS.h"
#include "clang/Frontend/CASDependencyCollector.h"
#include "clang/Frontend/CompileJobCacheKey.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Lex/PreprocessorOptions.h"
#include "llvm/ADT/ScopeExit.h"
#include "llvm/CAS/ActionCache.h"
#include "llvm/CAS/CASOutputBackend.h"
#include "llvm/Support/IOSandbox.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/PrefixMapper.h"
#include "llvm/Support/ScopedDurationTimer.h"
#include "llvm/Support/VirtualOutputBackends.h"

using namespace clang;
using llvm::Error;

/// Stores and retrieves compilation artifacts using \p llvm::cas::ObjectStore
/// and \p llvm::cas::ActionCache.
class CompileJobCache::CachingOutputs {
public:
  using OutputKind = clang::cas::CompileJobCacheResult::OutputKind;

  CachingOutputs(CompilerInstance &Clang, StringRef WorkingDir,
                 llvm::PrefixMapper Mapper,
                 std::shared_ptr<llvm::cas::ObjectStore> DB,
                 std::shared_ptr<llvm::cas::ActionCache> Cache);

  /// \returns true if result was found and replayed, false otherwise.
  Expected<bool>
  tryReplayCachedResult(const llvm::cas::CASID &ResultCacheKey,
                        const llvm::cas::CASID &CanonicalResultCacheKey);

  /// \returns true on failure, false on success.
  bool prepareOutputCollection();

  void stopDiagnosticsCapture();

  /// This is for an output file that was written directly on the file system.
  /// It's a workaround until all compilation consumers adopt
  /// \c llvm::vfs::OutputBackend.
  Error addNonVirtualOutputFile(StringRef FilePath);

  /// Finish writing outputs from a computed result, after a cache miss.
  /// If SkipCache is true, it should not insert the ResultCacheKey into
  /// Cache for future uses.
  Error finishComputedResult(const llvm::cas::CASID &ResultCacheKey,
                             const llvm::cas::CASID &CanonicalResultCacheKey,
                             bool SkipCache);

  Expected<bool> replayCachedResult(const llvm::cas::CASID &ResultCacheKey,
                                    clang::cas::CompileJobCacheResult &Result,
                                    bool JustComputedResult);

private:
  StringRef getPathForOutputKind(OutputKind Kind);

  Error replayCachedDiagnostics(StringRef DiagsData);

  Expected<llvm::cas::ObjectRef>
  writeOutputs(const llvm::cas::CASID &ResultCacheKey);

  /// Replay a cache hit.
  ///
  /// \returns an \c Error if there was a problem performing the replay, true if
  /// the result was found and replayed, false otherwise.
  Expected<bool> replayCachedResult(const llvm::cas::CASID &ResultCacheKey,
                                    llvm::cas::ObjectRef ResultID,
                                    bool JustComputedResult);

  CompilerInstance &Clang;
  const llvm::PrefixMapper PrefixMapper;
  clang::cas::CompileJobCacheResult::Builder CachedResultBuilder;
  std::string OutputFile;
  std::string DependenciesFile;
  std::unique_ptr<clang::cas::CachingDiagnosticsProcessor> DiagProcessor;
  std::shared_ptr<llvm::cas::ObjectStore> CAS;
  std::shared_ptr<llvm::cas::ActionCache> Cache;
  IntrusiveRefCntPtr<llvm::cas::CASOutputBackend> CASOutputs;
  std::optional<llvm::cas::ObjectRef> DependenciesOutput;
};

StringRef
CompileJobCache::CachingOutputs::getPathForOutputKind(OutputKind Kind) {
  switch (Kind) {
  case OutputKind::MainOutput:
    return OutputFile;
  case OutputKind::Dependencies:
    return DependenciesFile;
  default:
    return "";
  }
}

static std::string fixupRelativePath(const std::string &Path, FileManager &FM,
                                     StringRef WorkingDir) {
  if (llvm::sys::path::is_absolute(Path) || Path.empty() || Path == "-")
    return Path;

  // Apply -working-dir compiler option.
  // FIXME: this needs to stay in sync with createOutputFileImpl. Ideally, clang
  // would create output files by their "kind" rather than by path.
  SmallString<128> PathStorage(Path);
  if (FM.FixupRelativePath(PathStorage))
    return std::string(PathStorage);

  // Apply "normal" working directory.
  if (!WorkingDir.empty()) {
    SmallString<128> Tmp(Path);
    llvm::sys::path::make_absolute(WorkingDir, Tmp);
    return std::string(Tmp);
  }
  return Path;
}

CompileJobCache::CompileJobCache() = default;
CompileJobCache::~CompileJobCache() = default;

int CompileJobCache::reportCachingBackendError(DiagnosticsEngine &Diag,
                                               Error &&E) {
  Diag.Report(diag::err_caching_backend_fail) << llvm::toString(std::move(E));
  return 1;
}

std::optional<int> CompileJobCache::initialize(CompilerInstance &Clang) {
  CompilerInvocation &Invocation = Clang.getInvocation();
  DiagnosticsEngine &Diags = Clang.getDiagnostics();
  FrontendOptions &FrontendOpts = Invocation.getFrontendOpts();
  CacheCompileJob = FrontendOpts.CacheCompileJob;

  // Nothing else to do if we're not caching.
  if (!CacheCompileJob)
    return std::nullopt;

  std::tie(CAS, Cache) = Clang.getOrCreateCASDatabases();
  if (!CAS || !Cache)
    return 1; // Exit with error!

  CompileJobCachingOptions CacheOpts;
  auto Keys = canonicalizeAndCreateCacheKeys(*CAS, *Cache, Diags, Invocation,
                                             CacheOpts);
  if (!Keys)
    return 1; // Exit with error!
  std::tie(ResultCacheKey, ResultCacheKeyWithInputCacheKeysResolved) = *Keys;

  switch (FrontendOpts.ProgramAction) {
  case frontend::GeneratePCH:
    Clang.getPreprocessorOpts().CachingDiagOption = CachingDiagKind::Error;
    break;
  default:
    Clang.getPreprocessorOpts().CachingDiagOption = CachingDiagKind::Warning;
    break;
  }

  DisableCachedCompileJobReplay = CacheOpts.DisableCachedCompileJobReplay;

  // Ensure path canonicalization in text diagnostics.
  Invocation.getDiagnosticOpts().FallbackRealFileSystemAbsolutePaths = true;

  llvm::PrefixMapper PrefixMapper;

  CacheBackend = std::make_unique<CachingOutputs>(
      Clang, /*WorkingDir=*/"", std::move(PrefixMapper), CAS, Cache);

  return std::nullopt;
}

CompileJobCache::CachingOutputs::CachingOutputs(
    CompilerInstance &Clang, StringRef WorkingDir, llvm::PrefixMapper Mapper,
    std::shared_ptr<llvm::cas::ObjectStore> DB,
    std::shared_ptr<llvm::cas::ActionCache> Cache)
    : Clang(Clang), PrefixMapper(std::move(Mapper)), CAS(std::move(DB)),
      Cache(std::move(Cache)) {
  if (CAS)
    CASOutputs = llvm::makeIntrusiveRefCnt<llvm::cas::CASOutputBackend>(*CAS);

  CompilerInvocation &Invocation = Clang.getInvocation();
  FrontendOptions &FrontendOpts = Invocation.getFrontendOpts();
  if (!Clang.hasFileManager())
    Clang.createFileManager();
  FileManager &FM = Clang.getFileManager();
  OutputFile = fixupRelativePath(FrontendOpts.OutputFile, FM, WorkingDir);
  DependenciesFile = fixupRelativePath(
      Invocation.getDependencyOutputOpts().OutputFile, FM, WorkingDir);
  DiagProcessor = std::make_unique<clang::cas::CachingDiagnosticsProcessor>(
      PrefixMapper, FM);
}

Expected<bool> CompileJobCache::CachingOutputs::tryReplayCachedResult(
    const llvm::cas::CASID &ResultCacheKey,
    const llvm::cas::CASID &CanonicalResultCacheKey) {
  DiagnosticsEngine &Diags = Clang.getDiagnostics();

  std::optional<llvm::cas::CASID> Result;
  {
    llvm::ScopedDurationTimer ScopedTime([&Diags](double Seconds) {
      Diags.Report(diag::remark_compile_job_cache_timing_backend_key_query)
          << llvm::format("%.6fs", Seconds);
    });
    // Asking for the canonical cache key gives us better chances for cache hit.
    if (Error E = Cache->get(CanonicalResultCacheKey, /*CanBeDistributed=*/true)
                      .moveInto(Result))
      return std::move(E);
  }

  if (!Result) {
    Diags.Report(diag::remark_compile_job_cache_miss)
        << ResultCacheKey.toString();
    return false;
  }

  llvm::ScopedDurationTimer ScopedTime([&Diags](double Seconds) {
    Diags.Report(diag::remark_compile_job_cache_timing_backend_load)
        << llvm::format("%.6fs", Seconds);
  });

  std::optional<llvm::cas::ObjectRef> ResultRef = CAS->getReference(*Result);
  if (!ResultRef) {
    Diags.Report(diag::remark_compile_job_cache_miss_result_not_found)
        << ResultCacheKey.toString() << "result not in CAS";
    return false;
  }

  // Tasks depending on this one don't know the canonical cache key. Make sure
  // the requested key is associated with the result too.
  if (ResultCacheKey != CanonicalResultCacheKey)
    if (Error E =
            Cache->put(ResultCacheKey, *Result, /*CanBeDistributed=*/true))
      return std::move(E);

  // \c replayCachedResult emits remarks for a cache hit or miss.
  return replayCachedResult(ResultCacheKey, *ResultRef,
                            /*JustComputedResult=*/false);
}

std::optional<int>
CompileJobCache::tryReplayCachedResult(CompilerInstance &Clang) {
  if (!CacheCompileJob)
    return std::nullopt;

  DiagnosticsEngine &Diags = Clang.getDiagnostics();

  assert(ResultCacheKey.has_value() && "ResultCacheKey not initialized?");

  Clang.setCompileJobCacheKey(*ResultCacheKey);

  Expected<bool> ReplayedResult =
      DisableCachedCompileJobReplay
          ? false
          : CacheBackend->tryReplayCachedResult(
                *ResultCacheKey, *ResultCacheKeyWithInputCacheKeysResolved);
  if (!ReplayedResult)
    return reportCachingBackendError(Clang.getDiagnostics(),
                                     ReplayedResult.takeError());

  if (DisableCachedCompileJobReplay)
    Diags.Report(diag::remark_compile_job_cache_skipped)
        << ResultCacheKey->toString();

  if (*ReplayedResult)
    return 0;

  if (CacheBackend->prepareOutputCollection())
    return 1;

  return std::nullopt;
}

void CompileJobCache::CachingOutputs::stopDiagnosticsCapture() {
  DiagProcessor->removeDiagConsumer(Clang.getDiagnostics());
}

Error CompileJobCache::CachingOutputs::replayCachedDiagnostics(
    StringRef DiagsData) {
  DiagnosticConsumer &Consumer = *Clang.getDiagnostics().getClient();
  Consumer.BeginSourceFile(Clang.getLangOpts());
  if (Error E = DiagProcessor->replayCachedDiagnostics(DiagsData, Consumer))
    return E;
  Consumer.EndSourceFile();
  Clang.printDiagnosticStats();
  return Error::success();
}

bool CompileJobCache::CachingOutputs::prepareOutputCollection() {
  // Create an on-disk backend for streaming the results live if we run the
  // computation.
  IntrusiveRefCntPtr<llvm::vfs::OutputBackend> OnDiskOutputs =
      llvm::makeIntrusiveRefCnt<llvm::vfs::OnDiskOutputBackend>();

  // Set up the output backend so we can save / cache the result after.
  for (OutputKind K : clang::cas::CompileJobCacheResult::getAllOutputKinds()) {
    StringRef OutPath = getPathForOutputKind(K);
    if (!OutPath.empty())
      CachedResultBuilder.addKindMap(K, OutPath);
  }

  // Always filter out the dependencies file, since we build a CAS-specific
  // object for it.
  auto FilterBackend = llvm::vfs::makeFilteringOutputBackend(
      CASOutputs,
      [&](StringRef Path, std::optional<llvm::vfs::OutputConfig> Config) {
        return Path != DependenciesFile;
      });

  Clang.setOutputManager(llvm::vfs::makeMirroringOutputBackend(
      FilterBackend, std::move(OnDiskOutputs)));

  DiagProcessor->insertDiagConsumer(Clang.getDiagnostics());

  if (!Clang.getDependencyOutputOpts().OutputFile.empty())
    Clang.addDependencyCollector(std::make_shared<CASDependencyCollector>(
        Clang.getDependencyOutputOpts(), *CAS,
        [this](std::optional<llvm::cas::ObjectRef> Deps) {
          DependenciesOutput = Deps;
        }));

  return false;
}

bool CompileJobCache::finishComputedResult(CompilerInstance &Clang,
                                           bool Success) {
  // Nothing to do if not caching.
  if (!CacheCompileJob)
    return Success;

  CacheBackend->stopDiagnosticsCapture();

  // Don't cache failed builds.
  //
  // TODO: Consider caching failed builds! Note: when output files are written
  // without a temporary (non-atomically), failure may cause the removal of a
  // preexisting file. That behaviour is not currently modeled by the cache.
  if (!Success)
    return false;

  DiagnosticsEngine &Diags = Clang.getDiagnostics();

  bool UnsupportedOutput;
  if (Error E = maybeIngestNonVirtualOutputFromFileSystem(Clang).moveInto(
          UnsupportedOutput)) {
    reportCachingBackendError(Diags, std::move(E));
    return false;
  }

  // Check if we encounter any source that would generate non-reproducible
  // outputs.
  bool SkipCache =
      (Clang.hasPreprocessor() && Clang.isSourceNonReproducible()) ||
      UnsupportedOutput;
  if (SkipCache) {
    switch (Clang.getPreprocessorOpts().CachingDiagOption) {
    case CachingDiagKind::None:
      break;
    case CachingDiagKind::Warning:
      Diags.Report(diag::remark_compile_job_cache_skipped)
          << ResultCacheKey->toString();
      break;
    case CachingDiagKind::Error:
      llvm_unreachable("Should not reach here if there is an error");
    }
  }

  if (Error E = CacheBackend->finishComputedResult(
          *ResultCacheKey, *ResultCacheKeyWithInputCacheKeysResolved,
          SkipCache)) {
    reportCachingBackendError(Diags, std::move(E));
    return false;
  }
  return true;
}

Expected<bool> CompileJobCache::maybeIngestNonVirtualOutputFromFileSystem(
    CompilerInstance &Clang) {
  // FIXME: All consumers should adopt \c llvm::vfs::OutputBackend and this
  // function should go away.

  const auto &FrontendOpts = Clang.getFrontendOpts();
  if (FrontendOpts.ProgramAction == frontend::RunAnalysis) {
    StringRef OutputPath = FrontendOpts.OutputFile;
    if (OutputPath.empty())
      return false;
    bool IsDirectory = [&] {
      auto BypassSandbox = llvm::sys::sandbox::scopedDisable();
      return llvm::sys::fs::is_directory(OutputPath);
    }();
    if (IsDirectory) {
      // FIXME: A directory is produced for the 'html' output of the analyzer,
      // support it for caching purposes.
      Clang.getDiagnostics().Report(diag::warn_clang_cache_disabled_caching)
          << "analyzer output is not supported";
      return true;
    }
    if (Error E = CacheBackend->addNonVirtualOutputFile(OutputPath))
      return std::move(E);
    return false;
  }

  return false;
}

Expected<llvm::cas::ObjectRef> CompileJobCache::CachingOutputs::writeOutputs(
    const llvm::cas::CASID &ResultCacheKey) {
  DiagnosticsEngine &Diags = Clang.getDiagnostics();
  llvm::ScopedDurationTimer ScopedTime([&Diags](double Seconds) {
    Diags.Report(diag::remark_compile_job_cache_timing_backend_store)
        << llvm::format("%.6fs", Seconds);
  });

  Expected<std::optional<std::string>> SerialDiags =
      DiagProcessor->serializeEmittedDiagnostics();
  if (!SerialDiags)
    return SerialDiags.takeError();
  if (*SerialDiags) {
    Expected<llvm::cas::ObjectRef> DiagsRef =
        CAS->storeFromString({}, **SerialDiags);
    if (!DiagsRef)
      return DiagsRef.takeError();
    CachedResultBuilder.addOutput(OutputKind::SerializedDiagnostics, *DiagsRef);
  }

  if (DependenciesOutput)
    CachedResultBuilder.addOutput(OutputKind::Dependencies,
                                  *DependenciesOutput);

  auto BackendOutputs = CASOutputs->takeOutputs();
  for (auto &Output : BackendOutputs)
    if (auto Err = CachedResultBuilder.addOutput(Output.Path, Output.Object))
      return std::move(Err);

  // Cache the result.
  return CachedResultBuilder.build(*CAS);
}

Error CompileJobCache::CachingOutputs::addNonVirtualOutputFile(
    StringRef FilePath) {
  auto BypassSandbox = llvm::sys::sandbox::scopedDisable();

  auto F = llvm::sys::fs::openNativeFileForRead(FilePath);
  if (!F)
    return F.takeError();
  llvm::scope_exit CloseOnExit([&F]() { llvm::sys::fs::closeFile(*F); });

  std::optional<llvm::cas::ObjectRef> ObjRef;
  if (Error E = CAS->storeFromOpenFile(*F).moveInto(ObjRef))
    return E;
  CASOutputs->addObject(FilePath, *ObjRef);
  return Error::success();
}

Error CompileJobCache::CachingOutputs::finishComputedResult(
    const llvm::cas::CASID &ResultCacheKey,
    const llvm::cas::CASID &CanonicalResultCacheKey, bool SkipCache) {
  Expected<llvm::cas::ObjectRef> Result = writeOutputs(ResultCacheKey);
  if (!Result)
    return Result.takeError();

  // Skip caching if requested.
  if (!SkipCache) {
    DiagnosticsEngine &Diags = Clang.getDiagnostics();
    llvm::ScopedDurationTimer ScopedTime([&Diags](double Seconds) {
      Diags.Report(diag::remark_compile_job_cache_timing_backend_key_update)
          << llvm::format("%.6fs", Seconds);
    });
    if (llvm::Error E = Cache->put(CanonicalResultCacheKey, CAS->getID(*Result),
                                   /*CanBeDistributed=*/true))
      return E;
    // Tasks depending on this one don't know the canonical cache key. Make sure
    // the requested key is associated with the result too.
    if (ResultCacheKey != CanonicalResultCacheKey) {
      if (llvm::Error E = Cache->put(ResultCacheKey, CAS->getID(*Result),
                                     /*CanBeDistributed=*/true))
        return E;
    }
  }

  // Replay / decanonicalize as necessary.
  Expected<bool> Replayed = replayCachedResult(ResultCacheKey, *Result,
                                               /*JustComputedResult=*/true);
  if (!Replayed)
    return Replayed.takeError();
  return Error::success();
}

/// Replay a result after a cache hit.
Expected<bool> CompileJobCache::CachingOutputs::replayCachedResult(
    const llvm::cas::CASID &ResultCacheKey, llvm::cas::ObjectRef ResultID,
    bool JustComputedResult) {
  if (JustComputedResult)
    return false;

  std::optional<clang::cas::CompileJobCacheResult> Result;
  auto Schema = clang::cas::CompileJobResultSchema::create(*CAS);
  if (!Schema)
    return Schema.takeError();
  if (Error E = Schema->load(ResultID).moveInto(Result))
    return std::move(E);

  return replayCachedResult(ResultCacheKey, *Result, JustComputedResult);
}

Expected<bool> CompileJobCache::CachingOutputs::replayCachedResult(
    const llvm::cas::CASID &ResultCacheKey,
    clang::cas::CompileJobCacheResult &Result, bool JustComputedResult) {
  if (JustComputedResult)
    return false;

  llvm::cas::ObjectStore &CAS = Result.getCAS();
  DiagnosticsEngine &Diags = Clang.getDiagnostics();
  bool HasMissingOutput = false;
  std::optional<llvm::cas::ObjectProxy> SerialDiags;
  llvm::vfs::OnDiskOutputBackend Backend;

  auto processOutput = [&](clang::cas::CompileJobCacheResult::Output O,
                           std::optional<llvm::cas::ObjectProxy> Obj) -> Error {
    if (!Obj.has_value()) {
      Diags.Report(diag::remark_compile_job_cache_backend_output_not_found)
          << clang::cas::CompileJobCacheResult::getOutputKindName(O.Kind)
          << ResultCacheKey.toString() << CAS.getID(O.Object).toString();
      HasMissingOutput = true;
      return Error::success();
    }
    if (HasMissingOutput)
      return Error::success();

    if (O.Kind == OutputKind::SerializedDiagnostics) {
      SerialDiags = Obj;
      return Error::success();
    }

    std::string Path = std::string(getPathForOutputKind(O.Kind));
    if (Path.empty())
      // The output may be always generated but not needed with this invocation.
      return Error::success(); // continue

    // Always create parent directory of outputs, since it is hard to precisely
    // match which outputs rely on creating parents and the order outputs are
    // replayed in, in case a previous output would create the parent
    // (e.g. a .pcm and .diag file in the same directory).
    StringRef ParentPath = llvm::sys::path::parent_path(Path);
    if (!ParentPath.empty())
      llvm::sys::fs::create_directories(ParentPath);

    if (!JustComputedResult) {
      auto Output = Backend.createFile(Path);
      if (!Output)
        return Output.takeError();
      if (O.Kind == OutputKind::Dependencies) {
        if (auto E = CASDependencyCollector::replay(
                Clang.getDependencyOutputOpts(), CAS, *Obj, *Output))
          return E;
      } else {
        *Output << Obj->getData();
      }
      if (Error E = Output->keep())
        return E;
    }

    return Error::success();
  };

  if (auto Err = Result.forEachLoadedOutput(processOutput))
    return std::move(Err);

  if (HasMissingOutput) {
    Diags.Report(diag::remark_compile_job_cache_miss)
        << ResultCacheKey.toString();
    return false;
  }

  if (!JustComputedResult) {
    Diags.Report(diag::remark_compile_job_cache_hit)
        << ResultCacheKey.toString() << Result.getID().toString();

    if (SerialDiags) {
      if (Error E = replayCachedDiagnostics(SerialDiags->getData()))
        return std::move(E);
    }
  }

  if (JustComputedResult)
    return false;
  return true;
}
