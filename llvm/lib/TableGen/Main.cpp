//===- Main.cpp - Top-Level TableGen implementation -----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// TableGen is a tool which can be used to build up a description of something,
// then invoke one or more "tablegen backends" to emit information about the
// description in some predefined format. In practice, this is used by the LLVM
// code generators to automate generation of a code generator through a
// high-level description of the target.
//
//===----------------------------------------------------------------------===//

#include "llvm/TableGen/Main.h"
#include "TGLexer.h"
#include "TGParser.h"
#include "TableGenCache.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/Twine.h"
#include "llvm/CAS/ActionCache.h"
#include "llvm/CAS/BuiltinUnifiedCASDatabases.h"
#include "llvm/CAS/CASFileSystem.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorOr.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/IOSandbox.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/SMLoc.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/VirtualFileSystem.h"
#include "llvm/Support/VirtualOutputBackends.h"
#include "llvm/Support/WithColor.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/TableGen/Error.h"
#include "llvm/TableGen/Record.h"
#include "llvm/TableGen/TGTimer.h"
#include "llvm/TableGen/TableGenBackend.h"
#include <memory>
#include <string>
#include <system_error>
#include <utility>
using namespace llvm;
using namespace llvm::tablegen;

static cl::opt<std::string>
OutputFilename("o", cl::desc("Output filename"), cl::value_desc("filename"),
               cl::init("-"));

static cl::opt<std::string>
DependFilename("d",
               cl::desc("Dependency filename"),
               cl::value_desc("filename"),
               cl::init(""));

static cl::opt<std::string>
InputFilename(cl::Positional, cl::desc("<input file>"), cl::init("-"));

static cl::list<std::string>
IncludeDirs("I", cl::desc("Directory of include files"),
            cl::value_desc("directory"), cl::Prefix);

static cl::list<std::string>
MacroNames("D", cl::desc("Name of the macro to be defined"),
            cl::value_desc("macro name"), cl::Prefix);

static cl::opt<bool>
WriteIfChanged("write-if-changed", cl::desc("Only write output if it changed"));

static cl::opt<bool>
TimePhases("time-phases", cl::desc("Time phases of parser and backend"));

cl::opt<bool> llvm::EmitLongStrLiterals(
    "long-string-literals",
    cl::desc("when emitting large string tables, prefer string literals over "
             "comma-separated char literals. This can be a readability and "
             "compile-time performance win, but upsets some compilers"),
    cl::Hidden, cl::init(true));

static cl::opt<bool> NoWarnOnUnusedTemplateArgs(
    "no-warn-on-unused-template-args",
    cl::desc("Disable unused template argument warnings."));

static cl::OptionCategory CacheCategory("TableGen caching options");

static cl::opt<std::string>
    CASPath("cas-path",
            cl::desc("Cache results in the CAS at this path, and reuse results "
                     "cached for identical invocations"),
            cl::value_desc("path"), cl::cat(CacheCategory));

static cl::opt<bool> PrintCacheKey(
    "cas-print-cache-key",
    cl::desc("Print the cache key of the invocation and exit without running "
             "it"),
    cl::cat(CacheCategory));

static cl::opt<std::string> RunCacheKey(
    "cas-run-cache-key",
    cl::desc("Run the invocation recorded in a cache key, reading inputs from "
             "the CAS, instead of the one on the command line"),
    cl::value_desc("key"), cl::cat(CacheCategory));

static cl::opt<bool> CacheRemarks("cas-remarks",
                                  cl::desc("Report cache hits and misses"),
                                  cl::cat(CacheCategory));

static int reportError(const char *ProgName, Twine Msg) {
  errs() << ProgName << ": " << Msg;
  errs().flush();
  return 1;
}

/// Write \p Content to \p Filename through \p Outputs. With \p OnlyIfDifferent,
/// an existing file with the same contents is left untouched. Unless \p Keep,
/// the file is discarded instead of written.
static int writeFile(vfs::OutputBackend &Outputs, const char *argv0,
                     StringRef Filename, StringRef Content,
                     bool OnlyIfDifferent, bool Keep) {
  vfs::OutputConfig Config;
  Config.setText().setOnlyIfDifferent(OnlyIfDifferent);
  Expected<vfs::OutputFile> File = Outputs.createFile(Filename, Config);
  if (!File)
    return reportError(argv0, "error opening " + Filename + ": " +
                                  toString(File.takeError()) + "\n");
  *File << Content;
  if (Error E = Keep ? File->keep() : File->discard())
    return reportError(argv0, "error writing " + Filename + ": " +
                                  toString(std::move(E)) + "\n");
  return 0;
}

/// Create a dependency file for `-d` option.
///
/// This functionality is really only for the benefit of the build system.
/// It is similar to GCC's `-M*` family of options.
static int createDependencyFile(vfs::OutputBackend &Outputs,
                                ArrayRef<std::string> Dependencies,
                                const char *argv0) {
  if (OutputFilename == "-")
    return reportError(argv0, "the option -d must be used together with -o\n");

  std::string Content;
  raw_string_ostream OS(Content);
  OS << OutputFilename << ":";

  // Emit the primary input file as a dependency. This matches C compilers like
  // Clang and GCC. Without it, a .td file with no `include` directives would
  // produce a depfile listing zero dependencies. CMake's
  // `cmake_transform_depfile` then collapses that to a 0-byte file, which Ninja
  // treats as a missing depfile and re-runs the rule on every incremental
  // build.
  if (InputFilename != "-")
    OS << ' ' << InputFilename;

  for (const auto &Dep : Dependencies) {
    OS << ' ' << Dep;
  }
  OS << "\n";
  return writeFile(Outputs, argv0, DependFilename, Content,
                   /*OnlyIfDifferent=*/false, /*Keep=*/true);
}

static int WriteOutput(vfs::OutputBackend &Outputs, const char *argv0,
                       StringRef Filename, StringRef Content) {
  // With -write-if-changed, only update the real output file if there are any
  // differences. This prevents recompilation of all the files depending on it
  // if there aren't any.
  return writeFile(Outputs, argv0, Filename, Content, WriteIfChanged,
                   /*Keep=*/ErrorsPrinted == 0);
}

/// Write the dependency file and the outputs of an invocation.
static int writeResult(vfs::OutputBackend &Outputs, const char *argv0,
                       const CacheResult &Result,
                       ArrayRef<std::string> Dependencies) {
  // Always write the depfile, even if the main output hasn't changed.
  // If it's missing, Ninja considers the output dirty. If this was below
  // the early exit below and someone deleted the .inc.d file but not the .inc
  // file, tablegen would never write the depfile.
  if (!DependFilename.empty()) {
    if (int Ret = createDependencyFile(Outputs, Dependencies, argv0))
      return Ret;
  }

  if (int Ret = WriteOutput(Outputs, argv0, OutputFilename, Result.MainFile))
    return Ret;
  for (const auto &[Suffix, Content] : Result.AdditionalFiles) {
    SmallString<128> Filename(OutputFilename);
    // TODO: Format using the split-file convention when writing to stdout?
    if (Filename != "-") {
      sys::path::replace_extension(Filename, "");
      Filename.append(Suffix);
    }
    if (int Ret = WriteOutput(Outputs, argv0, Filename, Content))
      return Ret;
  }
  return 0;
}

/// Parse the input and run the backend, reading files only through \p FS. Hand
/// the outputs and the included files to \p Finish, which writes them through
/// an OutputBackend.
static int
TableGenMainImpl(const char *argv0, MultiFileTableGenMainFn MainFn,
                 IntrusiveRefCntPtr<vfs::FileSystem> FS,
                 function_ref<int(const CacheResult &Result,
                                  ArrayRef<std::string> Dependencies)>
                     Finish) {
  RecordKeeper Records;
  TGTimer &Timer = Records.getTimer();

  if (TimePhases)
    Timer.startPhaseTiming();

  // Parse the input file.

  Timer.startTimer("Parse, build records");
  ErrorOr<std::unique_ptr<MemoryBuffer>> FileOrErr = [&] {
    if (InputFilename != "-")
      return FS->getBufferForFile(InputFilename);
    // Standard input is not a file in any VFS, so this is the one read that
    // has to leave the sandbox. Caching rejects it, since it cannot be
    // snapshotted.
    auto BypassSandbox = sys::sandbox::scopedDisable();
    return MemoryBuffer::getSTDIN();
  }();
  if (std::error_code EC = FileOrErr.getError())
    return reportError(argv0, "Could not open input file '" + InputFilename +
                                  "': " + EC.message() + "\n");

  CacheResult Result;
  std::vector<std::string> Dependencies;
  {
    Records.saveInputFilename(InputFilename);

    // Tell SrcMgr about this buffer, which is what TGParser will pick up.
    SrcMgr.AddNewSourceBuffer(std::move(*FileOrErr), SMLoc());

    // Record the location of the include directory so that the lexer can find
    // it later.
    SrcMgr.setIncludeDirs(IncludeDirs);
    SrcMgr.setVirtualFileSystem(FS);

    TGParser Parser(SrcMgr, MacroNames, Records, NoWarnOnUnusedTemplateArgs);

    if (Parser.ParseFile())
      return 1;
    Timer.stopTimer();

    // Return early if any other errors were generated during parsing
    // (e.g., assert failures).
    if (ErrorsPrinted > 0)
      return reportError(argv0, Twine(ErrorsPrinted) + " errors.\n");

    // Write output to memory.
    Timer.startBackendTimer("Backend overall");
    TableGenOutputFiles OutFiles;
    unsigned status = 0;
    // ApplyCallback will return true if it did not apply any callback. In that
    // case, attempt to apply the MainFn.
    StringRef FilenamePrefix(sys::path::stem(OutputFilename));
    if (TableGen::Emitter::ApplyCallback(Records, OutFiles, FilenamePrefix))
      status = MainFn ? MainFn(OutFiles, Records) : 1;
    Timer.stopBackendTimer();
    if (status)
      return 1;

    Result.MainFile = std::move(OutFiles.MainFile);
    for (auto &[Suffix, Content] : OutFiles.AdditionalFiles)
      Result.AdditionalFiles.emplace_back(Suffix.str(), std::move(Content));
    Dependencies.assign(Parser.getDependencies().begin(),
                        Parser.getDependencies().end());
  }

  Timer.startTimer("Write output");
  if (int Ret = Finish(Result, Dependencies))
    return Ret;

  Timer.stopTimer();
  Timer.stopPhaseTiming();

  if (ErrorsPrinted > 0)
    return reportError(argv0, Twine(ErrorsPrinted) + " errors.\n");
  return 0;
}

int llvm::TableGenMain(const char *argv0, MultiFileTableGenMainFn MainFn) {
  if (!CASPath.empty())
    return reportError(argv0, "--" + CASPath.ArgStr +
                                  " is not supported by this tool\n");
  return TableGenMain(ArrayRef<const char *>(argv0), MainFn);
}

int llvm::TableGenMain(const char *argv0, TableGenMainFn MainFn) {
  return TableGenMain(argv0, [&MainFn](TableGenOutputFiles &OutFiles,
                                       const RecordKeeper &Records) {
    std::string S;
    raw_string_ostream OS(S);
    int Res = MainFn(OS, Records);
    OutFiles = {std::move(S), {}};
    return Res;
  });
}

/// Split \p Args into the arguments that determine the outputs, which go in
/// the cache key, and those that only control caching and where the
/// dependency file goes. The output file stays in the key, since its stem is
/// passed to backends.
static void partitionArgs(ArrayRef<const char *> Args,
                          std::vector<std::string> &KeyArgs,
                          std::vector<std::string> &ControlArgs) {
  const cl::Option *WithValue[] = {&DependFilename, &CASPath, &RunCacheKey};
  const cl::Option *Flags[] = {&WriteIfChanged, &PrintCacheKey, &CacheRemarks};
  auto matches = [](ArrayRef<const cl::Option *> Opts, StringRef Name) {
    return llvm::any_of(
        Opts, [&](const cl::Option *O) { return O->ArgStr == Name; });
  };

  bool SawDashDash = false;
  for (size_t I = 0, E = Args.size(); I != E; ++I) {
    StringRef Arg = Args[I];
    StringRef Name = Arg;
    if (SawDashDash || !Name.consume_front("-")) {
      KeyArgs.push_back(Arg.str());
      continue;
    }
    if (Name == "-") {
      SawDashDash = true;
      KeyArgs.push_back(Arg.str());
      continue;
    }
    Name.consume_front("-");
    auto [OptName, Value] = Name.split('=');
    if (matches(Flags, OptName)) {
      ControlArgs.push_back(Arg.str());
    } else if (matches(WithValue, OptName)) {
      ControlArgs.push_back(Arg.str());
      if (!Name.contains('=') && I + 1 != E)
        ControlArgs.push_back(Args[++I]);
    } else {
      KeyArgs.push_back(Arg.str());
    }
  }
}

static void remark(const Twine &Msg) {
  if (CacheRemarks)
    WithColor::remark() << Msg << "\n";
}

/// Load the invocation recorded in the cache key \p Key and make it the current
/// one, by re-parsing its command line followed by \p ControlArgs. Response
/// files are read from \p FS.
static Expected<CacheAction>
loadInvocation(cas::ObjectStore &CAS, vfs::FileSystem &FS,
               const cas::CASID &Key, StringRef Executable, const char *argv0,
               ArrayRef<std::string> ControlArgs) {
  std::optional<cas::ObjectRef> KeyRef = CAS.getReference(Key);
  if (!KeyRef)
    return createStringError("cache key '" + Key.toString() +
                             "' is not in the CAS");
  Expected<CacheAction> Action = loadAction(CAS, *KeyRef);
  if (!Action)
    return Action.takeError();
  if (Action->Executable != Executable)
    return createStringError("cache key '" + Key.toString() +
                             "' was created by a different TableGen "
                             "executable");

  std::vector<const char *> Argv = {argv0};
  for (const std::string &Arg : Action->CommandLine)
    Argv.push_back(Arg.c_str());
  for (const std::string &Arg : ControlArgs)
    Argv.push_back(Arg.c_str());
  std::string Errors;
  raw_string_ostream ErrorsOS(Errors);
  cl::ResetAllOptionOccurrences();
  if (!cl::ParseCommandLineOptions(Argv.size(), Argv.data(), /*Overview=*/"",
                                   &ErrorsOS, &FS))
    return createStringError(StringRef(Errors).trim());
  return Action;
}

static int runCached(ArrayRef<const char *> Args,
                     MultiFileTableGenMainFn MainFn,
                     IntrusiveRefCntPtr<vfs::FileSystem> RealFS,
                     vfs::OutputBackend &Outputs) {
  const char *argv0 = Args[0];
  auto fail = [argv0](Error E) {
    return reportError(argv0, toString(std::move(E)) + "\n");
  };

  // The CAS stores the inputs, actions, and results. The action cache maps the
  // cache key of an action to its result. The CAS manages its own files, and
  // is exempt from the sandbox internally.
  std::unique_ptr<cas::ObjectStore> CAS;
  std::unique_ptr<cas::ActionCache> Cache;
  {
    auto DBs = cas::createOnDiskUnifiedCASDatabases(CASPath);
    if (!DBs)
      return fail(DBs.takeError());
    std::tie(CAS, Cache) = std::move(*DBs);
  }

  Expected<std::string> Executable = hashExecutable(*RealFS, argv0);
  if (!Executable)
    return fail(Executable.takeError());

  // Expand response files the same way cl::ParseCommandLineOptions() did, so
  // that their contents are part of the key.
  BumpPtrAllocator Alloc;
  SmallVector<const char *> Expanded(Args.drop_front());
#ifdef _WIN32
  cl::ExpansionContext ECtx(Alloc, cl::TokenizeWindowsCommandLine,
                            RealFS.get());
#else
  cl::ExpansionContext ECtx(Alloc, cl::TokenizeGNUCommandLine, RealFS.get());
#endif
  if (Error E = ECtx.expandResponseFiles(Expanded))
    return fail(std::move(E));
  std::vector<std::string> KeyArgs, ControlArgs;
  partitionArgs(Expanded, KeyArgs, ControlArgs);

  // The inputs are normally read from the real file system. With
  // --cas-run-cache-key, the invocation recorded in the key runs instead, and
  // reads its inputs from the snapshot recorded with it. The steps below then
  // recreate the key, which checks that the scan finds the same inputs.
  IntrusiveRefCntPtr<vfs::FileSystem> FS = RealFS;
  std::optional<cas::CASID> GivenKey;
  if (!RunCacheKey.empty()) {
    if (!KeyArgs.empty())
      return reportError(argv0, "--" + RunCacheKey.ArgStr +
                                    " cannot be combined with '" +
                                    KeyArgs.front() + "'\n");
    // Tolerate surrounding whitespace, like the newline after a key printed
    // by --cas-print-cache-key.
    if (Error E = CAS->parseID(StringRef(RunCacheKey).trim()).moveInto(GivenKey))
      return fail(std::move(E));
    Expected<CacheAction> Recorded = loadInvocation(
        *CAS, *RealFS, *GivenKey, *Executable, argv0, ControlArgs);
    if (!Recorded)
      return fail(Recorded.takeError());
    KeyArgs = std::move(Recorded->CommandLine);
    FS =
        cas::createCASFileSystem(*Recorded->Inputs, Recorded->WorkingDirectory);
  }
  if (InputFilename == "-")
    return reportError(argv0,
                       "--" + CASPath.ArgStr + " requires an input file\n");

  // 1. Dependency scanning: find the files the input includes.
  ErrorOr<std::unique_ptr<MemoryBuffer>> MainFile =
      FS->getBufferForFile(InputFilename);
  if (std::error_code EC = MainFile.getError())
    return reportError(argv0, "Could not open input file '" + InputFilename +
                                  "': " + EC.message() + "\n");
  std::optional<std::vector<std::string>> Dependencies =
      scanDependencies(FS, std::move(*MainFile), IncludeDirs, MacroNames);
  if (!Dependencies)
    return 1;
  if (ErrorsPrinted > 0)
    return reportError(argv0, Twine(ErrorsPrinted) + " errors.\n");

  // 2. CAS file system: snapshot the input and the files it includes.
  CacheAction Action;
  Action.Executable = *Executable;
  Action.CommandLine = std::move(KeyArgs);
  if (Error E = errorOrToExpected(FS->getCurrentWorkingDirectory())
                    .moveInto(Action.WorkingDirectory))
    return fail(std::move(E));
  SmallVector<StringRef> Inputs = {InputFilename};
  append_range(Inputs, *Dependencies);
  if (Error E = cas::snapshotFiles(*CAS, *FS, Inputs).moveInto(Action.Inputs))
    return fail(std::move(E));

  // 3. CAS API: store the action, whose ID is the cache key, and look up its
  // result.
  Expected<cas::ObjectProxy> Key = storeAction(*CAS, Action);
  if (!Key)
    return fail(Key.takeError());
  std::string KeyStr = Key->getID().toString();
  if (GivenKey && Key->getID() != *GivenKey)
    return reportError(argv0, "cache key '" + KeyStr +
                                  "' recreated from the invocation in '" +
                                  GivenKey->toString() + "' does not match\n");
  if (PrintCacheKey) {
    outs() << KeyStr << "\n";
    return 0;
  }

  Expected<std::optional<cas::CASID>> Cached = Cache->get(Key->getID());
  if (!Cached)
    return fail(Cached.takeError());
  // The result may be missing from the CAS if it was pruned; rerun then.
  if (*Cached) {
    if (std::optional<cas::ObjectRef> ResultRef =
            CAS->getReference(**Cached)) {
      Expected<CacheResult> Result = loadResult(*CAS, *ResultRef);
      if (!Result)
        return fail(Result.takeError());
      remark("cache hit for '" + KeyStr + "'");
      return writeResult(Outputs, argv0, *Result, *Dependencies);
    }
  }

  // 4. Sandboxing: on a miss, run TableGen inside the IO sandbox against the
  // snapshot alone, so that the result depends on nothing outside the key.
  IntrusiveRefCntPtr<vfs::FileSystem> SnapshotFS =
      cas::createCASFileSystem(*Action.Inputs, Action.WorkingDirectory);
  remark("cache miss for '" + KeyStr + "'");
  return TableGenMainImpl(
      argv0, MainFn, SnapshotFS,
      [&](const CacheResult &Result,
          ArrayRef<std::string> RunDependencies) -> int {
        // A run that reported errors failed and its outputs are discarded, so
        // there is nothing to cache. Warnings are not part of the result: a
        // cache hit does not replay them.
        if (ErrorsPrinted == 0) {
          Expected<cas::ObjectRef> ResultRef = storeResult(*CAS, Result);
          if (!ResultRef)
            return fail(ResultRef.takeError());
          if (Error E = Cache->put(Key->getID(), CAS->getID(*ResultRef)))
            return fail(std::move(E));
        }
        return writeResult(Outputs, argv0, Result, RunDependencies);
      });
}

int llvm::TableGenMain(ArrayRef<const char *> Args,
                       MultiFileTableGenMainFn MainFn) {
  assert(!Args.empty() && "missing argv[0]");
  const char *argv0 = Args[0];
  if (CASPath.empty() && (PrintCacheKey || !RunCacheKey.empty()))
    return reportError(argv0, "--" + PrintCacheKey.ArgStr + " and --" +
                                  RunCacheKey.ArgStr + " require --" +
                                  CASPath.ArgStr + "\n");

  // Sandboxing: TableGen reads files only through a vfs::FileSystem and writes
  // them only through a vfs::OutputBackend. Acquire the real ones, then enter
  // the IO sandbox for the rest of the run, where any other access to the file
  // system is a fatal error, including acquiring another real file system.
  IntrusiveRefCntPtr<vfs::FileSystem> RealFS = vfs::getRealFileSystem();
  auto Outputs = makeIntrusiveRefCnt<vfs::OnDiskOutputBackend>();
  auto EnableSandbox = sys::sandbox::scopedEnable();

  if (!CASPath.empty())
    return runCached(Args, MainFn, RealFS, *Outputs);
  return TableGenMainImpl(
      argv0, MainFn, RealFS,
      [&](const CacheResult &Result, ArrayRef<std::string> Dependencies) {
        return writeResult(*Outputs, argv0, Result, Dependencies);
      });
}
