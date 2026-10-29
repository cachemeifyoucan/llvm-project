//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_FRONTEND_COMPILEJOBCACHE_H
#define LLVM_CLANG_FRONTEND_COMPILEJOBCACHE_H

#include "clang/Frontend/CompileJobCacheResult.h"

namespace clang {

class CompilerInstance;
class CompilerInvocation;
class DiagnosticsEngine;

/// Manages caching and replay of compile jobs.
///
/// The cache key for a job is computed from its canonicalized
/// \c CompilerInvocation; options that don't affect the result (such as
/// output paths) are canonicalized away. On a cache hit the cached outputs
/// and diagnostics are replayed. On a miss the outputs are written to disk
/// as usual while also being captured, together with the emitted
/// diagnostics, and the result is stored in the cache.
class CompileJobCache {
public:
  CompileJobCache();
  ~CompileJobCache();

  /// Canonicalize \p Clang.
  ///
  /// \returns status if should exit immediately, otherwise None.
  ///
  /// TODO: Refactor \a cc1_main() so that instead this canonicalizes the
  /// CompilerInvocation before Clang gets access to command-line arguments, to
  /// control what might leak.
  std::optional<int> initialize(CompilerInstance &Clang);

  /// Try looking up a cached result and replaying it.
  ///
  /// \returns status if should exit immediately, otherwise None.
  std::optional<int> tryReplayCachedResult(CompilerInstance &Clang);

  /// Finish writing outputs from a computed result, after a cache miss.
  ///
  /// \returns true if finished successfully.
  bool finishComputedResult(CompilerInstance &Clang, bool Success);

  class CachingOutputs;

private:
  int reportCachingBackendError(DiagnosticsEngine &Diag, llvm::Error &&E);

  bool CacheCompileJob = false;
  bool DisableCachedCompileJobReplay = false;

  std::shared_ptr<llvm::cas::ObjectStore> CAS;
  std::shared_ptr<llvm::cas::ActionCache> Cache;
  std::optional<llvm::cas::CASID> ResultCacheKey;
  std::optional<llvm::cas::CASID> ResultCacheKeyWithInputCacheKeysResolved;

  std::unique_ptr<CachingOutputs> CacheBackend;
};

} // namespace clang

#endif // LLVM_CLANG_FRONTEND_COMPILEJOBCACHE_H
