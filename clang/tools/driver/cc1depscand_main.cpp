//===- cc1depscand_main.cpp - Clang CC1 Dependency Scanning Daemon --------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MappingHelper.h"
#include "clang/Basic/DiagnosticDriver.h"
#include "clang/Basic/DiagnosticFrontend.h"
#include "clang/Basic/Stack.h"
#include "clang/Basic/TargetOptions.h"
#include "clang/CodeGen/ObjectFilePCHContainerOperations.h"
#include "clang/Config/config.h"
#include "clang/Driver/CC1DepScanDProtocol.h"
#include "clang/Driver/Options.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/CompilerInvocation.h"
#include "clang/Frontend/TextDiagnosticBuffer.h"
#include "clang/Frontend/TextDiagnosticPrinter.h"
#include "clang/Frontend/Utils.h"
#include "clang/FrontendTool/Utils.h"
#include "clang/Tooling/DependencyScanning/DependencyScanningService.h"
#include "clang/Tooling/DependencyScanning/DependencyScanningTool.h"
#include "llvm/ADT/ScopeExit.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CAS/CASDB.h"
#include "llvm/CAS/CachingOnDiskFileSystem.h"
#include "llvm/Option/ArgList.h"
#include "llvm/Support/BuryPointer.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/ManagedStatic.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/Process.h"
#include "llvm/Support/Signals.h"
#include "llvm/Support/ThreadPool.h"
#include "llvm/Support/raw_ostream.h"
#include <cstdio>
#include <mutex>
#include <shared_mutex>
#include <sys/file.h> // FIXME: Unix-only. Not portable.

#ifdef CLANG_HAVE_RLIMITS
#include <sys/resource.h>
#endif

using namespace clang;
using namespace llvm::opt;
using llvm::Error;

#ifdef CLANG_HAVE_RLIMITS
#if defined(__linux__) && defined(__PIE__)
static size_t getCurrentStackAllocation() {
  // If we can't compute the current stack usage, allow for 512K of command
  // line arguments and environment.
  size_t Usage = 512 * 1024;
  if (FILE *StatFile = fopen("/proc/self/stat", "r")) {
    // We assume that the stack extends from its current address to the end of
    // the environment space. In reality, there is another string literal (the
    // program name) after the environment, but this is close enough (we only
    // need to be within 100K or so).
    unsigned long StackPtr, EnvEnd;
    // Disable silly GCC -Wformat warning that complains about length
    // modifiers on ignored format specifiers. We want to retain these
    // for documentation purposes even though they have no effect.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat"
#endif
    if (fscanf(StatFile,
               "%*d %*s %*c %*d %*d %*d %*d %*d %*u %*lu %*lu %*lu %*lu %*lu "
               "%*lu %*ld %*ld %*ld %*ld %*ld %*ld %*llu %*lu %*ld %*lu %*lu "
               "%*lu %*lu %lu %*lu %*lu %*lu %*lu %*lu %*llu %*lu %*lu %*d %*d "
               "%*u %*u %*llu %*lu %*ld %*lu %*lu %*lu %*lu %*lu %*lu %lu %*d",
               &StackPtr, &EnvEnd) == 2) {
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
      Usage = StackPtr < EnvEnd ? EnvEnd - StackPtr : StackPtr - EnvEnd;
    }
    fclose(StatFile);
  }
  return Usage;
}

#include <alloca.h>

LLVM_ATTRIBUTE_NOINLINE
static void ensureStackAddressSpace() {
  // Linux kernels prior to 4.1 will sometimes locate the heap of a PIE binary
  // relatively close to the stack (they are only guaranteed to be 128MiB
  // apart). This results in crashes if we happen to heap-allocate more than
  // 128MiB before we reach our stack high-water mark.
  //
  // To avoid these crashes, ensure that we have sufficient virtual memory
  // pages allocated before we start running.
  size_t Curr = getCurrentStackAllocation();
  const int kTargetStack = DesiredStackSize - 256 * 1024;
  if (Curr < kTargetStack) {
    volatile char *volatile Alloc =
        static_cast<volatile char *>(alloca(kTargetStack - Curr));
    Alloc[0] = 0;
    Alloc[kTargetStack - Curr - 1] = 0;
  }
}
#else
static void ensureStackAddressSpace() {}
#endif

/// Attempt to ensure that we have at least 8MiB of usable stack space.
static void ensureSufficientStack() {
  struct rlimit rlim;
  if (getrlimit(RLIMIT_STACK, &rlim) != 0)
    return;

  // Increase the soft stack limit to our desired level, if necessary and
  // possible.
  if (rlim.rlim_cur != RLIM_INFINITY &&
      rlim.rlim_cur < rlim_t(DesiredStackSize)) {
    // Try to allocate sufficient stack.
    if (rlim.rlim_max == RLIM_INFINITY ||
        rlim.rlim_max >= rlim_t(DesiredStackSize))
      rlim.rlim_cur = DesiredStackSize;
    else if (rlim.rlim_cur == rlim.rlim_max)
      return;
    else
      rlim.rlim_cur = rlim.rlim_max;

    if (setrlimit(RLIMIT_STACK, &rlim) != 0 ||
        rlim.rlim_cur != DesiredStackSize)
      return;
  }


  // We should now have a stack of size at least DesiredStackSize. Ensure
  // that we can actually use that much, if necessary.
  ensureStackAddressSpace();
}
#else
static void ensureSufficientStack() {}
#endif

static void reportAsFatalIfError(llvm::Error E) {
  if (E)
    llvm::report_fatal_error(std::move(E));
}

template <typename T> static T reportAsFatalIfError(Expected<T> ValOrErr) {
  if (!ValOrErr)
    reportAsFatalIfError(ValOrErr.takeError());
  return std::move(*ValOrErr);
}

namespace {
class OneOffCompilationDatabase : public tooling::CompilationDatabase {
public:
  OneOffCompilationDatabase() = delete;
  template <class... ArgsT>
  OneOffCompilationDatabase(ArgsT &&... Args)
      : Command(std::forward<ArgsT>(Args)...) {}

  std::vector<tooling::CompileCommand>
  getCompileCommands(StringRef FilePath) const override {
    return {Command};
  }

  std::vector<tooling::CompileCommand> getAllCompileCommands() const override {
    return {Command};
  }

private:
  tooling::CompileCommand Command;
};
}

namespace {
class SharedStream {
public:
  SharedStream(raw_ostream &OS) : OS(OS) {}
  void applyLocked(llvm::function_ref<void(raw_ostream &OS)> Fn) {
    std::unique_lock<std::mutex> LockGuard(Lock);
    Fn(OS);
    OS.flush();
  }

private:
  std::mutex Lock;
  raw_ostream &OS;
};
} // namespace

// FIXME: This is a copy of Command::writeResponseFile. Command is too deeply
// tied with clang::Driver to use directly.
static void writeResponseFile(raw_ostream &OS,
                              SmallVectorImpl<const char *> &Arguments) {
  for (const auto *Arg : Arguments) {
    OS << '"';

    for (; *Arg != '\0'; Arg++) {
      if (*Arg == '\"' || *Arg == '\\') {
        OS << '\\';
      }
      OS << *Arg;
    }

    OS << "\" ";
  }
  OS << "\n";
}

int cc1depscan_main(ArrayRef<const char *> Argv, const char *Argv0,
                    void *MainAddr) {
  SmallString<128> DiagsBuffer;
  llvm::raw_svector_ostream DiagsOS(DiagsBuffer);
  auto DiagsConsumer = std::make_unique<TextDiagnosticPrinter>(
      DiagsOS, new DiagnosticOptions(), false);
  DiagnosticsEngine Diags(new DiagnosticIDs(), new DiagnosticOptions());
  Diags.setClient(DiagsConsumer.get(), /*ShouldOwnClient=*/false);

  // FIXME: Create a new OptionFlag group for cc1depscan.
  const OptTable &Opts = clang::driver::getDriverOptTable();
  unsigned MissingArgIndex, MissingArgCount;
  auto Args = Opts.ParseArgs(Argv, MissingArgIndex, MissingArgCount);
  if (MissingArgCount) {
    Diags.Report(diag::err_drv_missing_argument)
        << Args.getArgString(MissingArgIndex) << MissingArgCount;
    return 1;
  }

  llvm::BumpPtrAllocator Alloc;
  llvm::StringSaver Saver(Alloc);
  StringRef WorkingDirectory;
  if (auto *Arg =
          Args.getLastArg(clang::driver::options::OPT_working_directory))
    WorkingDirectory = Saver.save(Arg->getValue());

  cc1depscand::DepscanPrefixMapping PrefixMapping;
  for (auto &Arg :
       Args.getAllArgValues(clang::driver::options::OPT_fdepscan_prefix_map_EQ))
    PrefixMapping.PrefixMap.push_back(Saver.save(Arg));
  if (auto *Arg = Args.getLastArg(
          clang::driver::options::OPT_fdepscan_prefix_map_sdk_EQ))
    PrefixMapping.NewSDKPath = Saver.save(Arg->getValue());
  if (auto *Arg = Args.getLastArg(
          clang::driver::options::OPT_fdepscan_prefix_map_toolchain_EQ))
    PrefixMapping.NewToolchainPath = Saver.save(Arg->getValue());

  auto *CC1Args = Args.getLastArg(clang::driver::options::OPT_cc1_args);
  if (!CC1Args) {
    llvm::errs() << "missing -cc1-args option\n";
    return 1;
  }

  StringRef DumpDepscanTree;
  if (auto *Arg =
          Args.getLastArg(clang::driver::options::OPT_dump_depscan_tree_EQ))
    DumpDepscanTree = Saver.save(Arg->getValue());

  auto *OutputArg = Args.getLastArg(clang::driver::options::OPT_o);
  std::string OutputPath = OutputArg ? OutputArg->getValue() : "-";

  std::unique_ptr<llvm::cas::CASDB> CAS = reportAsFatalIfError(
      llvm::cas::createOnDiskCAS(llvm::cas::getDefaultOnDiskCASPath()));
  IntrusiveRefCntPtr<llvm::cas::CachingOnDiskFileSystem> FS =
      llvm::cantFail(llvm::cas::createCachingOnDiskFileSystem(*CAS));
  tooling::dependencies::DependencyScanningService Service(
      tooling::dependencies::ScanningMode::MinimizedSourcePreprocessing,
      tooling::dependencies::ScanningOutputFormat::Tree, FS,
      /*ReuseFileManager=*/false,
      /*SkipExcludedPPRanges=*/true);
  tooling::dependencies::DependencyScanningTool Tool(Service);
  SmallVector<const char *> NewArgs;
  Optional<llvm::cas::CASID> RootID;
  if (Error E =
          updateCC1Args(*FS, Tool, *DiagsConsumer, Argv0, CC1Args->getValues(),
                        WorkingDirectory, NewArgs, PrefixMapping,
                        [&](const Twine &T) { return Saver.save(T).data(); })
              .moveInto(RootID)) {
    llvm::errs() << "failed to update -cc1: " << toString(std::move(E)) << "\n";
    return 1;
  }

  // FIXME: Use OutputBackend to OnDisk only now.
  auto OutputBackend =
      llvm::makeIntrusiveRefCnt<llvm::vfs::OnDiskOutputBackend>();
  auto OutputFile =
      OutputBackend->createFile(OutputPath, llvm::vfs::OutputConfig()
                                                .setText(true)
                                                .setTextWithCRLF(true)
                                                .setCrashCleanup(false)
                                                .setAtomicWrite(false));
  if (!OutputFile) {
    Diags.Report(diag::err_fe_unable_to_open_output)
        << OutputArg->getValue() << llvm::toString(OutputFile.takeError());
    return 1;
  }

  if (!DumpDepscanTree.empty()) {
    std::error_code EC;
    llvm::raw_fd_ostream RootOS(DumpDepscanTree, EC);
    if (EC)
      Diags.Report(diag::err_fe_unable_to_open_output)
          << DumpDepscanTree << EC.message();
    RootOS << llvm::cantFail(CAS->convertCASIDToString(*RootID)) << "\n";
  }
  writeResponseFile(*(*OutputFile)->getOS(), NewArgs);

  if (auto Err = (*OutputFile)->close()) {
    llvm::errs() << "failed closing outputfile: "
                 << llvm::toString(std::move(Err)) << "\n";
    return 1;
  }
  return 0;
}

int cc1depscand_main(ArrayRef<const char *> Argv, const char *Argv0,
                     void *MainAddr) {
  ensureSufficientStack();

  if (Argv.size() < 2)
    llvm::report_fatal_error(
        "clang -cc1depscand: missing command and base-path");

  StringRef Command = Argv[0];
  StringRef BasePath = Argv[1];

  // Shutdown test mode. In shutdown test mode, daemon will open acception, but
  // not replying anything, just tear down the connect immediately.
  bool ShutDownTest = false;
  if (Argv.size() == 3 && StringRef(Argv[2]) == "-shutdown")
    ShutDownTest = true;

  if (Command == "-launch") {
    signal(SIGCHLD, SIG_IGN);
    const char *Args[] = {
        Argv0, "-cc1depscand", "-run", BasePath.begin(), nullptr,
    };
    int IgnoredPid;
    int EC = ::posix_spawn(&IgnoredPid, Args[0], /*file_actions=*/nullptr,
                           /*attrp=*/nullptr, const_cast<char **>(Args),
                           /*envp=*/nullptr);
    if (EC)
      llvm::report_fatal_error("clang -cc1depscand: failed to daemonize");
    ::exit(0);
  }

  if (Command != "-run")
    llvm::report_fatal_error("clang -cc1depscand: unknown command '" + Command +
                             "'");

  // Daemonize.
  if (::signal(SIGHUP, SIG_IGN) == SIG_ERR)
    llvm::report_fatal_error("clang -cc1depscand: failed to ignore SIGHUP");
  if (::setsid() == -1)
    llvm::report_fatal_error("clang -cc1depscand: setsid failed");

  // Check the pidfile.
  SmallString<128> PidPath, LogOutPath, LogErrPath;
  (BasePath + ".pid").toVector(PidPath);
  (BasePath + ".out").toVector(LogOutPath);
  (BasePath + ".err").toVector(LogErrPath);

  auto openAndReplaceFD = [&](int ReplacedFD, StringRef Path) {
    int FD;
    if (std::error_code EC = llvm::sys::fs::openFile(
            Path, FD, llvm::sys::fs::CD_CreateAlways, llvm::sys::fs::FA_Write,
            llvm::sys::fs::OF_None)) {
      // Ignoring error?
      ::close(ReplacedFD);
      return;
    }
    ::dup2(FD, ReplacedFD);
    ::close(FD);
  };
  openAndReplaceFD(1, LogOutPath);
  openAndReplaceFD(2, LogErrPath);

  bool ShouldKeepOutputs = true;
  auto DropOutputs = llvm::make_scope_exit([&]() {
    if (ShouldKeepOutputs)
      return;
    ::unlink(LogOutPath.c_str());
    ::unlink(LogErrPath.c_str());
  });

  int PidFD;
  [&]() {
    if (std::error_code EC = llvm::sys::fs::openFile(
            PidPath, PidFD, llvm::sys::fs::CD_OpenAlways,
            llvm::sys::fs::FA_Write, llvm::sys::fs::OF_None))
      llvm::report_fatal_error("clang -cc1depscand: cannot open pidfile");

    // Try to lock; failure means there's another daemon running.
    if (::flock(PidFD, LOCK_EX | LOCK_NB))
      ::exit(0);

    // FIXME: Should we actually write the pid here? Maybe we don't care.
  }();

  // Clean up the pidfile when we're done.
  auto ClosePidFile = [&]() {
    if (PidFD != -1)
      ::close(PidFD);
    PidFD = -1;
  };
  auto ClosePidFileAtExit = llvm::make_scope_exit([&]() { ClosePidFile(); });

  // Open the socket and start listening.
  int ListenSocket = cc1depscand::createSocket();
  if (ListenSocket == -1)
    llvm::report_fatal_error("clang -cc1depscand: cannot open socket");

  if (cc1depscand::bindToSocket(BasePath, ListenSocket))
    llvm::report_fatal_error(StringRef() +
                             "clang -cc1depscand: cannot bind to socket" +
                             ": " + strerror(errno));
  bool IsBound = true;
  auto RemoveBindFile = [&] {
    assert(IsBound);
    cc1depscand::unlinkBoundSocket(BasePath);
    IsBound = false;
  };
  auto RemoveBindFileAtExit = llvm::make_scope_exit([&]() {
    if (IsBound)
      RemoveBindFile();
  });

  llvm::ThreadPool Pool;
  if (::listen(ListenSocket, /*MaxBacklog=*/Pool.getThreadCount() * 16))
    llvm::report_fatal_error("clang -cc1depscand: cannot listen to socket");

  auto ShutdownCleanUp = [&]() {
    RemoveBindFile();
    ClosePidFile();
    ::close(ListenSocket);
  };

  // FIXME: Should use user-specified CAS, if any.
  std::unique_ptr<llvm::cas::CASDB> CAS = reportAsFatalIfError(
      llvm::cas::createOnDiskCAS(llvm::cas::getDefaultOnDiskCASPath()));

  IntrusiveRefCntPtr<llvm::cas::CachingOnDiskFileSystem> FS =
      llvm::cantFail(llvm::cas::createCachingOnDiskFileSystem(*CAS));
  tooling::dependencies::DependencyScanningService Service(
      tooling::dependencies::ScanningMode::MinimizedSourcePreprocessing,
      tooling::dependencies::ScanningOutputFormat::Tree, FS,
      /*ReuseFileManager=*/false,
      /*SkipExcludedPPRanges=*/true);

  std::atomic<bool> ShutDown(false);
  std::atomic<int> NumRunning(0);

  std::chrono::steady_clock::time_point Start =
      std::chrono::steady_clock::now();
  std::atomic<uint64_t> SecondsSinceLastClose;

  SharedStream SharedOS(llvm::errs());

#ifndef NDEBUG
  if (ShutDownTest)
    llvm::outs() << "launched in shutdown test state\n";
#endif

  for (unsigned I = 0; I < Pool.getThreadCount(); ++I) {
    Pool.async([&Service, &ShutDown, &ListenSocket, &NumRunning, &Start,
                &SecondsSinceLastClose, I, FS, Argv0, &SharedOS, ShutDownTest,
                &ShutdownCleanUp]() {
      Optional<tooling::dependencies::DependencyScanningTool> Tool;
      SmallString<256> Message;
      while (true) {
        if (ShutDown.load())
          return;

        int Data = cc1depscand::acceptSocket(ListenSocket);
        if (Data == -1)
          continue;

        auto CloseData = llvm::make_scope_exit([&]() { ::close(Data); });
        cc1depscand::CC1DepScanDProtocol Comms(Data);

        auto StopRunning = llvm::make_scope_exit([&]() {
          SecondsSinceLastClose.store(
              std::chrono::duration_cast<std::chrono::seconds>(
                  std::chrono::steady_clock::now() - Start)
                  .count());
          --NumRunning;
        });

        {
          ++NumRunning;

          // In test mode, just tear down everything.
          if (ShutDownTest) {
            ShutdownCleanUp();
            ShutDown.store(true);
            continue;
          }
          // Check again for shutdown, since the main thread could have
          // requested it before we created the service.
          //
          // FIXME: Return Optional<ServiceReference> from the map, handling
          // this condition in getOrCreateService().
          if (ShutDown.load()) {
            // Abort the work in shutdown state since the thread can go down
            // anytime.
            return; // FIXME: Tell the client about this?
          }
        }

        // First put a result kind as a handshake.
        if (auto E = Comms.putResultKind(
              cc1depscand::CC1DepScanDProtocol::SuccessResult)) {
          SharedOS.applyLocked([&](raw_ostream &OS) {
            OS << I << ": failed to send handshake\n";
            logAllUnhandledErrors(std::move(E), OS);
          });
          continue; // go back to wait when handshake failed.
        }

        llvm::BumpPtrAllocator Alloc;
        llvm::StringSaver Saver(Alloc);
        StringRef WorkingDirectory;
        SmallVector<const char *> Args;
        cc1depscand::DepscanPrefixMapping PrefixMapping;
        if (llvm::Error E = Comms.getCommand(Saver, WorkingDirectory, Args,
                                             PrefixMapping)) {
          SharedOS.applyLocked([&](raw_ostream &OS) {
            OS << I << ": failed to get command\n";
            logAllUnhandledErrors(std::move(E), OS);
          });
          continue; // FIXME: Tell the client something went wrong.
        }

        auto printScannedCC1 = [&](raw_ostream &OS) {
          OS << I << ": scanned -cc1:";
          for (const char *Arg : Args)
            OS << " " << Arg;
          OS << "\n";
        };

        // Is this safe to reuse? Or does DependendencyScanningWorkerFileSystem
        // make some bad assumptions about relative paths?
        if (!Tool)
          Tool.emplace(Service);

        // auto DiagsConsumer = std::make_unique<IgnoringDiagConsumer>();
        SmallString<128> DiagsBuffer;
        llvm::raw_svector_ostream DiagsOS(DiagsBuffer);
        auto DiagsConsumer = std::make_unique<TextDiagnosticPrinter>(
            DiagsOS, new DiagnosticOptions(), false);

        SmallVector<const char *> NewArgs;
        if (Error E = updateCC1Args(
                          *FS, *Tool, *DiagsConsumer, Argv0, Args,
                          WorkingDirectory, NewArgs, PrefixMapping,
                          [&](const Twine &T) { return Saver.save(T).data(); })
                          .takeError()) {
          SharedOS.applyLocked([&](raw_ostream &OS) {
            printScannedCC1(OS);
            OS << I << ": failed to create compiler invocation\n";
            OS << DiagsBuffer;
          });
          consumeError(Comms.putResultKind(
              cc1depscand::CC1DepScanDProtocol::ErrorResult));
          continue;
        }

        if (llvm::Error E = Comms.putResultKind(
                cc1depscand::CC1DepScanDProtocol::SuccessResult)) {
          SharedOS.applyLocked([&](raw_ostream &OS) {
            printScannedCC1(OS);
            logAllUnhandledErrors(std::move(E), OS);
          });
          continue; // FIXME: Tell the client something went wrong.
        }

        auto printComputedCC1 = [&](raw_ostream &OS) {
          OS << I << ": sending back new -cc1 args:\n";
          for (const char *Arg : NewArgs)
            OS << " " << Arg;
          OS << "\n";
        };
        if (llvm::Error E = Comms.putArgs(NewArgs)) {
          SharedOS.applyLocked([&](raw_ostream &OS) {
            printScannedCC1(OS);
            printComputedCC1(OS);
            logAllUnhandledErrors(std::move(E), OS);
          });
          continue; // FIXME: Tell the client something went wrong.
        }

        // Done!
#ifndef NDEBUG
        // In +asserts mode, print out -cc1s even on success.
        SharedOS.applyLocked([&](raw_ostream &OS) {
          printScannedCC1(OS);
          printComputedCC1(OS);
        });
#endif
      }
    });
  };

  // Wait for the work to finish.
  const uint64_t SecondsBetweenAttempts = 5;
  const uint64_t SecondsBeforeDestruction = 15;
  uint64_t SleepTime = SecondsBeforeDestruction;
  while (true) {
    ::sleep(SleepTime);
    SleepTime = SecondsBetweenAttempts;

    if (NumRunning.load())
      continue;

    if (ShutDown.load())
      break;

    // Figure out the latest access time that we'll delete.
    uint64_t LastAccessToDestroy =
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - Start)
            .count();
    if (LastAccessToDestroy < SecondsBeforeDestruction)
      continue; // In case ::sleep returns slightly early.
    LastAccessToDestroy -= SecondsBeforeDestruction;

    if (LastAccessToDestroy < SecondsSinceLastClose)
      continue;

    // Tear down the socket and bind file immediately but wait till all existing
    // jobs to finish.
    ShutdownCleanUp();
    ShutDown.store(true);
  }

  return 0;
}

Expected<llvm::cas::CASID>
clang::updateCC1Args(llvm::cas::CachingOnDiskFileSystem &FS,
                     tooling::dependencies::DependencyScanningTool &Tool,
                     DiagnosticConsumer &DiagsConsumer, const char *Exec,
                     ArrayRef<const char *> InputArgs,
                     StringRef WorkingDirectory,
                     SmallVectorImpl<const char *> &OutputArgs,
                     const cc1depscand::DepscanPrefixMapping &PrefixMapping,
                     llvm::function_ref<const char *(const Twine &)> SaveArg) {
  // FIXME: Should use user-specified CAS, if any.
  llvm::cas::CASDB &CAS = FS.getCAS();

  DiagnosticsEngine Diags(new DiagnosticIDs(), new DiagnosticOptions());
  Diags.setClient(&DiagsConsumer, /*ShouldOwnClient=*/false);
  auto Invocation = std::make_shared<CompilerInvocation>();
  if (!CompilerInvocation::CreateFromArgs(*Invocation, InputArgs, Diags, Exec))
    return llvm::createStringError(llvm::inconvertibleErrorCode(),
                                   "failed to create compiler invocation");

  llvm::BumpPtrAllocator Alloc;
  llvm::StringSaver Saver(Alloc);
  SmallVector<std::pair<StringRef, StringRef>> ComputedMapping;
  if (llvm::Error E = computeFullMapping(Saver, FS, Exec, *Invocation,
                                         PrefixMapping, ComputedMapping))
    return std::move(E);

  Optional<llvm::cas::CASID> Root;
  if (Error E = Tool.getDependencyTreeFromCompilerInvocation(
                        std::make_shared<CompilerInvocation>(*Invocation),
                        WorkingDirectory, DiagsConsumer,
                        [&](const llvm::vfs::CachedDirectoryEntry &Entry) {
                          return remapPath(Entry.getTreePath(), Saver,
                                           ComputedMapping);
                        })
                    .moveInto(Root))
    return std::move(E);
  std::string RootID = llvm::cantFail(CAS.convertCASIDToString(*Root));
  updateCompilerInvocation(*Invocation, Saver, FS, RootID, WorkingDirectory,
                           ComputedMapping);

  OutputArgs.resize(1);
  OutputArgs[0] = "-cc1";
  Invocation->generateCC1CommandLine(OutputArgs, SaveArg);
  return *Root;
}

Expected<llvm::cas::CASID>
clang::updateCC1Args(const char *Exec, ArrayRef<const char *> InputArgs,
                     SmallVectorImpl<const char *> &OutputArgs,
                     const cc1depscand::DepscanPrefixMapping &PrefixMapping,
                     llvm::function_ref<const char *(const Twine &)> SaveArg) {
  // FIXME: Should use user-specified CAS, if any.
  std::unique_ptr<llvm::cas::CASDB> CAS = reportAsFatalIfError(
      llvm::cas::createOnDiskCAS(llvm::cas::getDefaultOnDiskCASPath()));

  IntrusiveRefCntPtr<llvm::cas::CachingOnDiskFileSystem> FS =
      llvm::cantFail(llvm::cas::createCachingOnDiskFileSystem(*CAS));

  tooling::dependencies::DependencyScanningService Service(
      tooling::dependencies::ScanningMode::MinimizedSourcePreprocessing,
      tooling::dependencies::ScanningOutputFormat::Tree, FS,
      /*ReuseFileManager=*/false,
      /*SkipExcludedPPRanges=*/true);
  tooling::dependencies::DependencyScanningTool Tool(Service);

  auto DiagsConsumer = std::make_unique<IgnoringDiagConsumer>();

  SmallString<128> WorkingDirectory;
  reportAsFatalIfError(
      llvm::errorCodeToError(llvm::sys::fs::current_path(WorkingDirectory)));

  return updateCC1Args(*FS, Tool, *DiagsConsumer, Exec, InputArgs,
                       WorkingDirectory, OutputArgs, PrefixMapping, SaveArg);
}
