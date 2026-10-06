//===- TableGenCache.h - Compilation caching for TableGen -------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Pieces for caching TableGen invocations in a CAS. Caching an invocation
// takes these steps (see runCached() in Main.cpp):
//
//   1. Dependency scanning: parse the input once to find every file it
//      includes (scanDependencies()).
//   2. CAS file system: snapshot the input and those files into the CAS
//      (cas::snapshotFiles()). The snapshot is part of the cache key.
//   3. CAS API: store the action, whose ID is the cache key, and look it up in
//      the ActionCache. On a hit, write the cached outputs.
//   4. Sandboxing: on a miss, run TableGen against a cas::createCASFileSystem()
//      view of the snapshot, inside the IO sandbox. Any read that bypasses the
//      VFS, or write that bypasses the OutputBackend, is a fatal error, so the
//      result depends on nothing outside the cache key. Store the result.
//
// An action is a NamedValuesSchema node with these entries:
//   "command-line"      the arguments, NUL-separated, without argv[0] and
//                       without the options that only control caching or
//                       where the dependency file goes
//   "executable"        BLAKE3 hash of the TableGen binary, in hex
//   "inputs"            FileSystemSchema snapshot of the main file and every
//                       file it includes
//   "kind"              version string for this encoding
//   "working-directory" the directory relative paths are resolved against
//
// The action holds everything needed to rerun the invocation without the
// original files.
//
// A result is a NamedValuesSchema node with an "output" entry for the main
// output and an "output:<suffix>" entry for each additional output. The
// dependency file is not part of it: the scan finds the dependencies on every
// run, hit or miss. Diagnostics are not part of it either, so a cache hit does
// not replay the warnings printed by the run that produced the result.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TABLEGEN_TABLEGENCACHE_H
#define LLVM_LIB_TABLEGEN_TABLEGENCACHE_H

#include "llvm/ADT/IntrusiveRefCntPtr.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/CAS/CASFileSystem.h"
#include "llvm/CAS/ObjectStore.h"
#include "llvm/Support/Error.h"
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace llvm {
class MemoryBuffer;
namespace vfs {
class FileSystem;
} // namespace vfs

namespace tablegen {

/// Everything that determines the outputs of a TableGen invocation.
struct CacheAction {
  std::string Executable;
  std::vector<std::string> CommandLine;
  std::string WorkingDirectory;
  std::optional<cas::FileSystemProxy> Inputs;
};

/// The outputs of a TableGen invocation.
struct CacheResult {
  std::string MainFile;
  /// Additional outputs, as (filename suffix, contents).
  std::vector<std::pair<std::string, std::string>> AdditionalFiles;
};

/// Hash the running executable, read through \p FS, so that results produced
/// by a different TableGen binary are not reused.
Expected<std::string> hashExecutable(vfs::FileSystem &FS, const char *Argv0);

/// Parse \p MainFile, reading included files from \p FS, and return the files
/// it includes, as named in a dependency file.
///
/// Errors are reported, and std::nullopt is returned if parsing fails.
/// Warnings are dropped: on a cache miss, the run that follows reports them.
std::optional<std::vector<std::string>>
scanDependencies(IntrusiveRefCntPtr<vfs::FileSystem> FS,
                 std::unique_ptr<MemoryBuffer> MainFile,
                 ArrayRef<std::string> IncludeDirs,
                 ArrayRef<std::string> Macros);

/// Store \p Action and return the node whose ID is the cache key.
Expected<cas::ObjectProxy> storeAction(cas::ObjectStore &CAS,
                                       const CacheAction &Action);

/// Load the action stored with the cache key \p Key.
Expected<CacheAction> loadAction(cas::ObjectStore &CAS, cas::ObjectRef Key);

/// Store \p Result.
Expected<cas::ObjectRef> storeResult(cas::ObjectStore &CAS,
                                     const CacheResult &Result);

/// Load a result stored by \a storeResult().
Expected<CacheResult> loadResult(cas::ObjectStore &CAS, cas::ObjectRef Ref);

} // namespace tablegen
} // namespace llvm

#endif // LLVM_LIB_TABLEGEN_TABLEGENCACHE_H
