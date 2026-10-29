//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Clang CC1 Dependency Scanning.
///
//===----------------------------------------------------------------------===//

#include "cc1depscanProtocol.h"
#include "clang/Basic/DiagnosticCAS.h"
#include "clang/Basic/DiagnosticDriver.h"
#include "clang/Basic/DiagnosticFrontend.h"
#include "clang/Basic/Stack.h"
#include "clang/Config/config.h"
#include "clang/DependencyScanning/DependencyScanningService.h"
#include "clang/DependencyScanning/ScanAndUpdateArgs.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/CompilerInvocation.h"
#include "clang/Frontend/TextDiagnosticPrinter.h"
#include "clang/Frontend/Utils.h"
#include "clang/Options/Options.h"
#include "clang/Tooling/DependencyScanningTool.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/ScopeExit.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CAS/ActionCache.h"
#include "llvm/CAS/CASProvidingFileSystem.h"
#include "llvm/CAS/ObjectStore.h"
#include "llvm/Option/ArgList.h"
#include "llvm/Support/BLAKE3.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/IOSandbox.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/Process.h"
#include "llvm/Support/ScopedDurationTimer.h"
#include "llvm/Support/Signals.h"
#include "llvm/Support/ThreadPool.h"
#include "llvm/Support/VirtualOutputBackends.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/raw_socket_stream.h"
#include <chrono>
#include <mutex>
#include <optional>

#if LLVM_ON_UNIX
#include <sys/file.h>
#include <sys/signal.h>
#include <unistd.h>
#endif // LLVM_ON_UNIX

#ifdef _WIN32
#include <io.h>
#include <windows.h>
#endif

#ifdef CLANG_HAVE_RLIMITS
#include <sys/resource.h>
#endif

using namespace clang;
using namespace llvm::opt;
using cc1depscand::DepscanSharing;
using llvm::Error;

#define DEBUG_TYPE "cc1depscand"

ALWAYS_ENABLED_STATISTIC(NumRequests, "Number of -cc1 update requests");

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
#elif defined(_WIN32)
/// Check that we have sufficient stack space on Windows.
/// Note: Unlike Unix, Windows stack size cannot be changed at runtime.
/// It's set at process creation time via the PE header or CreateThread.
static void ensureSufficientStack() {
  // GetCurrentThreadStackLimits requires Windows 8 or later
  ULONG_PTR LowLimit, HighLimit;
  GetCurrentThreadStackLimits(&LowLimit, &HighLimit);

  // Calculate available stack size
  size_t StackSize = HighLimit - LowLimit;

  // If we don't have enough stack space, warn the user
  // We can't increase it dynamically on Windows like we can on Unix
  if (StackSize < DesiredStackSize) {
    llvm::errs() << "Warning: Stack size (" << StackSize
                 << " bytes) is less than desired size (" << DesiredStackSize
                 << " bytes).\n";
    llvm::errs() << "Consider increasing stack size via linker flags "
                 << "(/STACK on MSVC) or executable properties.\n";
  }
}
#else
static void ensureSufficientStack() {}
#endif

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

static std::optional<std::string>
makeDepscanDaemonKey(StringRef Mode, const DepscanSharing &Sharing) {
  auto completeKey = [&Sharing](llvm::BLAKE3 &Hasher) -> std::string {
    // Only share depscan daemons that originated from the same clang version.
    Hasher.update(getClangFullVersion());
    for (const char *Arg : Sharing.CASArgs)
      Hasher.update(StringRef(Arg));
    // Using same hash size as the module cache hash.
    auto Hash = Hasher.final<sizeof(uint64_t)>();
    uint64_t HashVal =
        llvm::support::endian::read<uint64_t, llvm::endianness::native>(
            Hash.data());
    return toString(llvm::APInt(64, HashVal), 36, /*Signed=*/false);
  };

  auto makePIDKey = [&completeKey](uint64_t PID) -> std::string {
    llvm::BLAKE3 Hasher;
    Hasher.update(ArrayRef(reinterpret_cast<uint8_t *>(&PID), sizeof(PID)));
    return completeKey(Hasher);
  };

  // Still daemonize, but use the PID from this process as the key to avoid
  // sharing state.
  if (Mode == "daemon")
    return makePIDKey(llvm::sys::Process::getProcessId());

  // Mode == "auto".
  //
  // TODO: consider returning ThisPID (same as "daemon") once the daemon can
  // share a CAS instance without sharing filesystem caching. Or maybe delete
  // "auto" at that point and make "-fdepscan" default to "-fdepscan=daemon".
  return std::nullopt;
}

static std::optional<std::string>
makeDepscanDaemonPath(StringRef Mode, const DepscanSharing &Sharing) {
  if (Mode == "inline")
    return std::nullopt;

  if (Sharing.Path)
    return Sharing.Path->str();

  if (auto Key = makeDepscanDaemonKey(Mode, Sharing))
    return cc1depscand::getBasePath(*Key);

  return std::nullopt;
}

//===----------------------------------------------------------------------===//
// Shared Scanning Functions
//===----------------------------------------------------------------------===//

static Expected<llvm::cas::CASID> scanAndUpdateCC1InlineWithTool(
    tooling::DependencyScanningTool &Tool, DiagnosticConsumer &DiagsConsumer,
    raw_ostream *VerboseOS, const char *Exec, ArrayRef<const char *> InputArgs,
    StringRef WorkingDirectory, SmallVectorImpl<const char *> &OutputArgs,
    llvm::function_ref<const char *(const Twine &)> SaveArg) {
  DiagnosticOptions DiagOpts;
  DiagnosticsEngine Diags(new DiagnosticIDs(), DiagOpts);
  Diags.setClient(&DiagsConsumer, /*ShouldOwnClient=*/false);
  auto Invocation = std::make_shared<CompilerInvocation>();
  if (!CompilerInvocation::CreateFromArgs(*Invocation, InputArgs, Diags, Exec))
    return llvm::createStringError(llvm::inconvertibleErrorCode(),
                                   "failed to create compiler invocation");

  Expected<llvm::cas::CASID> Root = scanAndUpdateCC1InlineWithTool(
      Tool, DiagsConsumer, VerboseOS, *Invocation, WorkingDirectory);
  if (!Root)
    return Root;

  OutputArgs.resize(1);
  OutputArgs[0] = "-cc1";
  Invocation->generateCC1CommandLine(OutputArgs, SaveArg);
  return *Root;
}

static int
scanAndUpdateCC1Inline(const char *Exec, ArrayRef<const char *> InputArgs,
                       StringRef WorkingDirectory,
                       SmallVectorImpl<const char *> &OutputArgs,
                       llvm::function_ref<const char *(const Twine &)> SaveArg,
                       const CASOptions &CASOpts, DiagnosticsEngine &Diag,
                       std::optional<llvm::cas::CASID> &RootID) {
  auto [DB, Cache] = CASOpts.createDatabases(Diag);
  if (!DB || !Cache)
    return 1;

  dependencies::DependencyScanningServiceOptions Opts;
  Opts.MakeVFS = [DB = DB] {
    auto BypassSandbox = llvm::sys::sandbox::scopedDisable();
    return llvm::cas::createCASProvidingFileSystem(
        DB, llvm::vfs::createPhysicalFileSystem());
  };
  Opts.Compilation = dependencies::IncludeTreeCompilation{CASOpts, DB, Cache};
  Opts.AsCompilation = true;
  dependencies::DependencyScanningService Service(std::move(Opts));
  tooling::DependencyScanningTool Tool(Service);

  std::unique_ptr<DiagnosticOptions> DiagOpts =
      CreateAndPopulateDiagOpts(InputArgs);
  auto DiagsConsumer =
      std::make_unique<TextDiagnosticPrinter>(llvm::errs(), *DiagOpts, false);

  auto E = scanAndUpdateCC1InlineWithTool(
               Tool, *DiagsConsumer, /*VerboseOS*/ nullptr, Exec, InputArgs,
               WorkingDirectory, OutputArgs, SaveArg)
               .moveInto(RootID);
  if (E) {
    Diag.Report(diag::err_cas_depscan_failed) << std::move(E);
    return 1;
  }

  return DiagsConsumer->getNumErrors() != 0;
}

//===----------------------------------------------------------------------===//
// Client-Side Code
//===----------------------------------------------------------------------===//

static int scanAndUpdateCC1UsingDaemon(
    const char *Exec, ArrayRef<const char *> OldArgs,
    StringRef WorkingDirectory, SmallVectorImpl<const char *> &NewArgs,
    StringRef Path, const DepscanSharing &Sharing, DiagnosticsEngine &Diag,
    llvm::function_ref<const char *(const Twine &)> SaveArg,
    const CASOptions &CASOpts, std::optional<llvm::cas::CASID> &Root) {
  using namespace clang::cc1depscand;
  auto reportScanFailure = [&](Error E) {
    Diag.Report(diag::err_cas_depscan_failed) << std::move(E);
    return 1;
  };

  // FIXME: Skip some of this if -fcas-fs has been passed.

  bool NoSpawnDaemon = (bool)Sharing.Path;
  // llvm::dbgs() << "connecting to daemon...\n";
  auto Daemon = NoSpawnDaemon
                    ? ScanDaemon::connectToDaemonAndShakeHands(Path)
                    : ScanDaemon::constructAndShakeHands(Path, Exec, Sharing);
  if (!Daemon)
    return reportScanFailure(Daemon.takeError());
  CC1DepScanDProtocol Comms(Daemon->getStream());

  // llvm::dbgs() << "sending request...\n";
  if (auto E = Comms.putCommand(WorkingDirectory, OldArgs))
    return reportScanFailure(std::move(E));

  llvm::BumpPtrAllocator Alloc;
  llvm::StringSaver Saver(Alloc);
  SmallVector<const char *> RawNewArgs;
  CC1DepScanDProtocol::ResultKind Result;
  StringRef FailedReason;
  StringRef RootID;
  StringRef DiagOut;
  auto E = Comms.getScanResult(Saver, Result, FailedReason, RootID, RawNewArgs,
                               DiagOut);
  // Send the diagnostics to std::err.
  llvm::errs() << DiagOut;
  if (E)
    return reportScanFailure(std::move(E));

  if (Result != CC1DepScanDProtocol::SuccessResult)
    return reportScanFailure(
        llvm::createStringError("depscan daemon failed: " + FailedReason));

  // FIXME: Avoid this duplication.
  NewArgs.resize(RawNewArgs.size());
  for (int I = 0, E = RawNewArgs.size(); I != E; ++I)
    NewArgs[I] = SaveArg(RawNewArgs[I]);

  // Create CAS after daemon returns the result so daemon can perform corrupted
  // CAS recovery.
  auto [CAS, _] = CASOpts.createDatabases(Diag);
  if (!CAS)
    return 1;

  if (auto E = CAS->parseID(RootID).moveInto(Root))
    return reportScanFailure(std::move(E));

  return 0;
}

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

static int scanAndUpdateCC1(const char *Exec, ArrayRef<const char *> OldArgs,
                            SmallVectorImpl<const char *> &NewArgs,
                            llvm::vfs::FileSystem &VFS, DiagnosticsEngine &Diag,
                            const llvm::opt::ArgList &Args,
                            const CASOptions &CASOpts,
                            std::optional<llvm::cas::CASID> &RootID) {
  llvm::ScopedDurationTimer ScopedTime([&Diag](double Seconds) {
    Diag.Report(diag::remark_compile_job_cache_timing_depscan)
        << llvm::format("%.6fs", Seconds);
  });

  StringRef WorkingDirectory;
  std::string WorkingDirectoryBuf;
  if (auto *Arg = Args.getLastArg(clang::options::OPT_working_directory)) {
    WorkingDirectory = Arg->getValue();
  } else {
    auto CWD = VFS.getCurrentWorkingDirectory();
    if (Error E = llvm::errorCodeToError(CWD.getError())) {
      Diag.Report(diag::err_cas_depscan_failed) << std::move(E);
      return 1;
    }
    WorkingDirectoryBuf = std::move(*CWD);
    WorkingDirectory = WorkingDirectoryBuf;
  }

  DepscanSharing Sharing;
  if (Arg *A = Args.getLastArg(options::OPT_fdepscan_daemon_EQ))
    Sharing.Path = A->getValue();

  StringRef Mode = "auto";
  if (Arg *A = Args.getLastArg(clang::options::OPT_fdepscan_EQ)) {
    Mode = A->getValue();
    // Note: -cc1depscan does not accept '-fdepscan=off'.
    if (Mode != "daemon" && Mode != "inline" && Mode != "auto") {
      Diag.Report(diag::err_drv_invalid_argument_to_option)
          << Mode << A->getOption().getName();
      return 1;
    }
  }

  auto SaveArg = [&Args](const Twine &T) { return Args.MakeArgString(T); };
  CompilerInvocation::GenerateCASArgs(CASOpts, Sharing.CASArgs, SaveArg);

  if (auto DaemonPath = makeDepscanDaemonPath(Mode, Sharing))
    return scanAndUpdateCC1UsingDaemon(Exec, OldArgs, WorkingDirectory, NewArgs,
                                       *DaemonPath, Sharing, Diag, SaveArg,
                                       CASOpts, RootID);

  return scanAndUpdateCC1Inline(Exec, OldArgs, WorkingDirectory, NewArgs,
                                SaveArg, CASOpts, Diag, RootID);
}

int cc1depscan_main(ArrayRef<const char *> Argv, const char *Argv0,
                    void *MainAddr) {
  std::unique_ptr<DiagnosticOptions> DiagOpts;
  {
    auto FoundCC1Args =
        std::find_if(Argv.begin(), Argv.end(), [](const char *Arg) -> bool {
          return (StringRef(Arg) == "-cc1-args");
        });
    if (FoundCC1Args != Argv.end()) {
      SmallVector<const char *, 8> WarnOpts{Argv0};
      WarnOpts.append(FoundCC1Args + 1, Argv.end());
      DiagOpts = CreateAndPopulateDiagOpts(WarnOpts);
    } else {
      DiagOpts = std::make_unique<DiagnosticOptions>();
    }
  }
  auto DiagsConsumer =
      std::make_unique<TextDiagnosticPrinter>(llvm::errs(), *DiagOpts, false);
  DiagnosticsEngine Diags(new DiagnosticIDs(), *DiagOpts);
  Diags.setClient(DiagsConsumer.get(), /*ShouldOwnClient=*/false);
  auto VFS = [] {
    auto BypassSandbox = llvm::sys::sandbox::scopedDisable();
    return llvm::vfs::getRealFileSystem();
  }();
  ProcessWarningOptions(Diags, *DiagOpts, *VFS);
  if (Diags.hasErrorOccurred())
    return 1;

  // FIXME: Create a new OptionFlag group for cc1depscan.
  const OptTable &Opts = getDriverOptTable();
  unsigned MissingArgIndex, MissingArgCount;
  auto Args = Opts.ParseArgs(Argv, MissingArgIndex, MissingArgCount);
  if (MissingArgCount) {
    Diags.Report(diag::err_drv_missing_argument)
        << Args.getArgString(MissingArgIndex) << MissingArgCount;
    return 1;
  }

  auto *CC1Args = Args.getLastArg(clang::options::OPT_cc1_args);
  if (!CC1Args) {
    llvm::errs() << "missing -cc1-args option\n";
    return 1;
  }

  auto *OutputArg = Args.getLastArg(clang::options::OPT_o);
  std::string OutputPath = OutputArg ? OutputArg->getValue() : "-";

  std::optional<StringRef> DumpDepscanTree;
  if (auto *Arg = Args.getLastArg(clang::options::OPT_dump_depscan_tree_EQ))
    DumpDepscanTree = Arg->getValue();

  SmallVector<const char *> NewArgs;
  std::optional<llvm::cas::CASID> RootID;

  CASOptions CASOpts;
  auto ParsedCC1Args =
      Opts.ParseArgs(CC1Args->getValues(), MissingArgIndex, MissingArgCount);
  CompilerInvocation::ParseCASArgs(CASOpts, ParsedCC1Args, Diags);
  CASOpts.ensurePersistentCAS();

#if defined(_WIN32)
  llvm::WSABalancer _;
#endif

  if (int Ret = scanAndUpdateCC1(Argv0, CC1Args->getValues(), NewArgs, *VFS,
                                 Diags, Args, CASOpts, RootID))
    return Ret;

  // FIXME: Use OutputBackend to OnDisk only now.
  auto OutputBackend =
      llvm::makeIntrusiveRefCnt<llvm::vfs::OnDiskOutputBackend>();
  auto OutputFile = consumeDiscardOnDestroy(
      OutputBackend->createFile(OutputPath, llvm::vfs::OutputConfig()
                                                .setTextWithCRLF(true)
                                                .setDiscardOnSignal(false)
                                                .setAtomicWrite(false)));
  if (!OutputFile) {
    Diags.Report(diag::err_fe_unable_to_open_output)
        << OutputArg->getValue() << llvm::toString(OutputFile.takeError());
    return 1;
  }

  if (DumpDepscanTree) {
    std::error_code EC;
    llvm::raw_fd_ostream RootOS(*DumpDepscanTree, EC);
    if (EC)
      Diags.Report(diag::err_fe_unable_to_open_output)
          << *DumpDepscanTree << EC.message();
    RootOS << RootID->toString() << "\n";
  }
  writeResponseFile(*OutputFile, NewArgs);

  if (auto Err = OutputFile->keep()) {
    llvm::errs() << "failed closing outputfile: "
                 << llvm::toString(std::move(Err)) << "\n";
    return 1;
  }
  return 0;
}

//===----------------------------------------------------------------------===//
// Server-Side Code
//===----------------------------------------------------------------------===//

namespace {
llvm::ExitOnError ExitOnErr("clang -cc1depscand: ");

void reportError(const llvm::Twine &Message) {
  ExitOnErr(llvm::createStringError(llvm::inconvertibleErrorCode(), Message));
}

struct ScanServer {
  const char *Argv0 = nullptr;
  SmallString<128> BasePath;
  CASOptions CASOpts;
  int PidFD = -1;
  /// \p std::nullopt means it runs indefinitely.
  std::optional<unsigned> TimeoutSeconds;
  std::unique_ptr<llvm::ListeningSocket> Listener;
  std::atomic<bool> ShutDown{false};

  ~ScanServer() { shutdown(); }

  void start(bool Exclusive, ArrayRef<const char *> CASArgs);
  int listen();

  /// Tear down the socket and bind file immediately but wait till all existing
  /// jobs to finish.
  void shutdown() {
    ShutDown.store(true);
    // Clean up the pidfile when we're done.
    if (PidFD != -1) {
      // Unlock pidfile first so another daemon can spin up when it can't find
      // the socket.
      llvm::sys::fs::unlockFile(PidFD);
      llvm::sys::Process::SafelyCloseFileDescriptor(PidFD);
      PidFD = -1; // Prevent double-close
    }
    if (Listener)
      Listener->shutdown();
  }
};

void ScanServer::start(bool Exclusive, ArrayRef<const char *> CASArgs) {
  // Parse CAS options and validate if needed.
  DiagnosticOptions DiagOpts;
  DiagnosticsEngine Diags(new DiagnosticIDs(), DiagOpts);

  const OptTable &Opts = getDriverOptTable();
  unsigned MissingArgIndex, MissingArgCount;
  auto ParsedCASArgs =
      Opts.ParseArgs(CASArgs, MissingArgIndex, MissingArgCount);
  CompilerInvocation::ParseCASArgs(CASOpts, ParsedCASArgs, Diags);
  CASOpts.ensurePersistentCAS();

  auto BypassSandbox = llvm::sys::sandbox::scopedDisable();

  // Check the pidfile.
  SmallString<128> PidPath;
  (BasePath + ".pid").toVector(PidPath);

  [&]() {
    if (std::error_code EC = llvm::sys::fs::openFile(
            PidPath, PidFD, llvm::sys::fs::CD_OpenAlways,
            llvm::sys::fs::FA_Write, llvm::sys::fs::OF_None))
      reportError("cannot open pidfile");

    // Try to lock; failure means there's another daemon running.
    if (std::error_code EC =
            llvm::sys::fs::tryLockFile(PidFD, std::chrono::milliseconds(0),
                                       llvm::sys::fs::LockKind::Exclusive)) {
      if (Exclusive)
        reportError("another daemon using the base path");
      ::exit(0);
    }

    // FIXME: Should we actually write the pid here? Maybe we don't care.
  }();

  unsigned MaxBacklog =
      llvm::hardware_concurrency().compute_thread_count() * 16;

  SmallString<128> SocketPath = BasePath;
  SocketPath.append(".sock");

  // Open the socket and start listening.
  llvm::Expected<llvm::ListeningSocket> Socket =
      llvm::ListeningSocket::createUnix(SocketPath, MaxBacklog);
  if (!Socket)
    reportError("cannot create listener: " +
                llvm::toString(Socket.takeError()));
  Listener = std::make_unique<llvm::ListeningSocket>(std::move(*Socket));
}

int ScanServer::listen() {
  llvm::DefaultThreadPool Pool;

  DiagnosticOptions DiagOpts;
  DiagnosticsEngine Diags(new DiagnosticIDs(), DiagOpts);
  std::shared_ptr<llvm::cas::ObjectStore> CAS;
  std::shared_ptr<llvm::cas::ActionCache> Cache;
  std::tie(CAS, Cache) = CASOpts.createDatabases(Diags);
  if (!CAS)
    reportError("cannot create CAS");
  if (!Cache)
    reportError("cannot create ActionCache");

  dependencies::DependencyScanningServiceOptions Opts;
  Opts.MakeVFS = [CAS] {
    auto BypassSandbox = llvm::sys::sandbox::scopedDisable();
    return llvm::cas::createCASProvidingFileSystem(
        CAS, llvm::vfs::createPhysicalFileSystem());
  };
  Opts.Compilation = dependencies::IncludeTreeCompilation{CASOpts, CAS, Cache};
  Opts.AsCompilation = true;
  dependencies::DependencyScanningService Service(std::move(Opts));

  std::atomic<int> NumRunning(0);
  std::mutex AcceptLock;

  std::chrono::steady_clock::time_point Start =
      std::chrono::steady_clock::now();
  std::atomic<uint64_t> SecondsSinceLastClose(0);

  SharedStream SharedOS(llvm::errs());

  auto ServiceLoop = [this, &Service, &NumRunning, &Start,
                      &SecondsSinceLastClose, &SharedOS,
                      &AcceptLock](unsigned I) {
    std::optional<tooling::DependencyScanningTool> Tool;
    SmallString<256> Message;
    while (true) {
      if (ShutDown.load())
        return;

      std::unique_ptr<llvm::raw_socket_stream> stream;
      {
        std::unique_lock<std::mutex> Guard(AcceptLock);
        if (ShutDown.load())
          return;

        llvm::Expected<std::unique_ptr<llvm::raw_socket_stream>> fd =
            Listener->accept(std::chrono::milliseconds(1000));
        if (!fd) {
          // Timeout or error - continue waiting for next connection
          llvm::consumeError(fd.takeError());
          continue;
        }
        stream = std::move(*fd);
      }

      cc1depscand::CC1DepScanDProtocol Comms(*stream);

      // Explicitly close the socket and clear any errors before the stream is
      // destroyed to prevent fatal error in raw_ostream destructor. On Windows,
      // socket close can trigger errors when the peer has already closed the
      // connection. By calling close() first, we set FD = -1 and ShouldClose =
      // false, so the base class destructor's close path is skipped entirely.
      llvm::scope_exit CloseStream([&]() {
        stream->close();
        stream->clear_error();
      });

      llvm::scope_exit StopRunning([&]() {
        SecondsSinceLastClose.store(
            std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - Start)
                .count());
        --NumRunning;
      });

      {
        ++NumRunning;

        // Check again for shutdown, since the main thread could have
        // requested it before we created the service.
        //
        // FIXME: Return std::optional<ServiceReference> from the map, handling
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
      if (llvm::Error E = Comms.getCommand(Saver, WorkingDirectory, Args)) {
        SharedOS.applyLocked([&](raw_ostream &OS) {
          OS << I << ": failed to get command\n";
          logAllUnhandledErrors(std::move(E), OS);
        });
        continue; // FIXME: Tell the client something went wrong.
      }

      // cc1 request.
      ++NumRequests;
      auto printScannedCC1 = [&](raw_ostream &OS) {
        OS << I << ": scanned -cc1:";
        for (const char *Arg : Args)
          OS << " " << Arg;
        OS << "\n";
      };

      if (!Tool)
        Tool.emplace(Service);

      std::unique_ptr<DiagnosticOptions> DiagOpts =
          CreateAndPopulateDiagOpts(Args);
      SmallString<128> DiagsBuffer;
      llvm::raw_svector_ostream DiagsOS(DiagsBuffer);
      DiagsOS.enable_colors(true);
      auto DiagsConsumer =
          std::make_unique<TextDiagnosticPrinter>(DiagsOS, *DiagOpts, false);

      SmallVector<const char *> NewArgs;
      auto RootID = scanAndUpdateCC1InlineWithTool(
          *Tool, *DiagsConsumer, &DiagsOS, Argv0, Args, WorkingDirectory,
          NewArgs, [&](const Twine &T) { return Saver.save(T).data(); });
      if (!RootID) {
        consumeError(Comms.putScanResultFailed(toString(RootID.takeError()),
                                               DiagsOS.str()));
        SharedOS.applyLocked([&](raw_ostream &OS) {
          printScannedCC1(OS);
          OS << I << ": failed to create compiler invocation\n";
          OS << DiagsBuffer;
        });
        continue;
      }

      auto printComputedCC1 = [&](raw_ostream &OS) {
        OS << I << ": sending back new -cc1 args:\n";
        for (const char *Arg : NewArgs)
          OS << " " << Arg;
        OS << "\n";
      };
      if (llvm::Error E = Comms.putScanResultSuccess(RootID->toString(),
                                                     NewArgs, DiagsOS.str())) {
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
  };

  for (unsigned I = 0; I < Pool.getMaxConcurrency(); ++I)
    Pool.async(ServiceLoop, I);

  if (!TimeoutSeconds) {
    Pool.wait();
    return 0;
  }

  // Wait for the work to finish.
  const uint64_t SecondsBetweenAttempts = 5;
  const uint64_t SecondsBeforeDestruction = *TimeoutSeconds;
  uint64_t SleepTime = SecondsBeforeDestruction;
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(SleepTime));
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

    shutdown();
  }

  return 0;
}
} // anonymous namespace

/// Accepted options are:
///
/// * -run <path> [-long-running] [-cas-args ...]
/// Runs the daemon until a timeout is reached without a new connection.
/// "-long-running" increases the timeout. stdout/stderr are redirected to files
/// relative to <path>. This is how the clang driver auto-spawns a new daemon.
///
/// * -serve <path> [-cas-args ...]
/// Runs indefinitely (until ctrl+c or the process is killed). There's no
/// stdout/stderr redirection. Useful for debugging and for starting a permanent
/// daemon before directing clang invocations to connect to it.
///
/// * -execute <path> [-cas-args ...] -- <command ...>
/// Sets up the socket path, sets \p CLANG_CACHE_SCAN_DAEMON_SOCKET_PATH
/// enviroment variable to the socket path, and executes the provided command.
/// It exits with the same exit code as the command. Useful for lit testing.
int cc1depscand_main(ArrayRef<const char *> Argv, const char *Argv0,
                     void *MainAddr) {
  if (Argv.size() < 2)
    reportError("missing command and base-path");

  ensureSufficientStack();

#if defined(_WIN32)
  llvm::WSABalancer _;
#endif

  StringRef Command = Argv[0];

  ScanServer Server;
  Server.Argv0 = Argv0;
  Server.BasePath = Argv[1];

  ArrayRef<const char *> CommandArgsToExecute;
  auto Sep = llvm::find_if(
      Argv, [](const char *Arg) { return StringRef(Arg) == "--"; });
  if (Sep != Argv.end()) {
    CommandArgsToExecute = Argv.drop_front(Sep - Argv.begin() + 1);
    Argv = Argv.slice(0, Sep - Argv.begin());
  }

  // Whether the daemon can safely stay alive a longer period of time.
  // FIXME: Consider designing a mechanism to notify daemons, started for a
  // particular "build session", to shutdown, then have it stay alive until the
  // session is finished.
  bool LongRunning = false;
  ArrayRef<const char *> CASArgs;
  for (auto [Index, Arg] : llvm::enumerate(Argv)) {
    if (StringRef(Arg) == "-long-running") {
      LongRunning = true;
    } else if (StringRef(Arg) == "-cas-args") {
      CASArgs = Argv.drop_front(Index + 1);
      break;
    }
  }

  {
    // Create the base directory if necessary.
    auto BypassSandbox = llvm::sys::sandbox::scopedDisable();
    StringRef BaseDir = llvm::sys::path::parent_path(Server.BasePath);
    if (std::error_code EC = llvm::sys::fs::create_directories(BaseDir))
      reportError(Twine("cannot create basedir: ") + EC.message());
  }

  if (Command == "-serve") {
    Server.start(/*Exclusive*/ true, CASArgs);
    return Server.listen();
  } else if (Command == "-execute") {
    SmallVector<StringRef, 32> RefArgs;
    RefArgs.reserve(CommandArgsToExecute.size());
    for (const char *Arg : CommandArgsToExecute)
      RefArgs.push_back(Arg);

    // Make sure to start the server before executing the command.
    Server.start(/*Exclusive*/ true, CASArgs);
    std::thread ServerThread([&Server]() { Server.listen(); });

#if defined(_WIN32)
    static_cast<void>(::_putenv_s("CLANG_CACHE_SCAN_DAEMON_SOCKET_PATH",
                                  Server.BasePath.c_str()));
#else
    setenv("CLANG_CACHE_SCAN_DAEMON_SOCKET_PATH", Server.BasePath.c_str(),
           true);
#endif

    std::string ErrMsg;
    int Result = llvm::sys::ExecuteAndWait(
        RefArgs.front(), RefArgs, /*Env*/ std::nullopt,
        /*Redirects*/ {}, /*SecondsToWait*/ 0,
        /*MemoryLimit*/ 0, &ErrMsg);

    Server.shutdown();
    ServerThread.join();

    if (!ErrMsg.empty())
      reportError("failed executing command: " + ErrMsg);
    return Result;
  } else if (Command != "-run") {
    reportError("unknown command '" + Command + "'");
  }

  Server.TimeoutSeconds = LongRunning ? 120 : 15;

  // Setup signal handlers for graceful shutdown (cross-platform).
  llvm::sys::AddSignalHandler(
      [](void *Cookie) {
        auto *Srv = static_cast<ScanServer *>(Cookie);
        Srv->shutdown();
      },
      &Server);

  llvm::sys::SetInterruptFunction([]() {
    // Note: This handler is called on interrupt (Ctrl+C).
    // Cannot call non-reentrant functions here, so just exit immediately.
    std::_Exit(0);
  });

  // Redirect stdout/stderr to log files.
  SmallString<128> LogOutPath, LogErrPath;
  (Server.BasePath + ".out").toVector(LogOutPath);
  (Server.BasePath + ".err").toVector(LogErrPath);

#ifdef _WIN32
  // On Windows, redirect stdout/stderr using freopen.
  // We don't call FreeConsole() to avoid invalidating these handles.
  if (std::freopen(LogOutPath.c_str(), "w", stdout) == nullptr)
    reportError("failed to redirect stdout");
  if (std::freopen(LogErrPath.c_str(), "w", stderr) == nullptr)
    reportError("failed to redirect stderr");
#else
  // On Unix, create a new session and redirect stdio.
  if (::signal(SIGHUP, SIG_IGN) == SIG_ERR)
    reportError("failed to ignore SIGHUP");
  if (::setsid() == -1)
    reportError("setsid failed");

  // Redirect stdout and stderr using dup2.
  auto openAndReplaceFD = [&](int ReplacedFD, StringRef Path) {
    auto BypassSandbox = llvm::sys::sandbox::scopedDisable();
    int FD;
    if (std::error_code EC = llvm::sys::fs::openFile(
            Path, FD, llvm::sys::fs::CD_CreateAlways, llvm::sys::fs::FA_Write,
            llvm::sys::fs::OF_None)) {
      llvm::sys::Process::SafelyCloseFileDescriptor(ReplacedFD);
      return;
    }
    ::dup2(FD, ReplacedFD);
    llvm::sys::Process::SafelyCloseFileDescriptor(FD);
  };
  openAndReplaceFD(1, LogOutPath);
  openAndReplaceFD(2, LogErrPath);
#endif

  Server.start(/*Exclusive*/ false, CASArgs);
  return Server.listen();
}
