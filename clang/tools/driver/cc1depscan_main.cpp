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

#include "clang/Basic/DiagnosticCAS.h"
#include "clang/Basic/DiagnosticDriver.h"
#include "clang/Basic/DiagnosticFrontend.h"
#include "clang/DependencyScanning/DependencyScanningService.h"
#include "clang/DependencyScanning/ScanAndUpdateArgs.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/CompilerInvocation.h"
#include "clang/Frontend/TextDiagnosticPrinter.h"
#include "clang/Frontend/Utils.h"
#include "clang/Options/Options.h"
#include "clang/Tooling/DependencyScanningTool.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CAS/ActionCache.h"
#include "llvm/CAS/CASProvidingFileSystem.h"
#include "llvm/CAS/ObjectStore.h"
#include "llvm/Option/ArgList.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/IOSandbox.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/ScopedDurationTimer.h"
#include "llvm/Support/VirtualOutputBackends.h"
#include "llvm/Support/raw_ostream.h"
#include <optional>

using namespace clang;
using namespace llvm::opt;
using llvm::Error;

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

  StringRef Mode = "auto";
  if (Arg *A = Args.getLastArg(clang::options::OPT_fdepscan_EQ)) {
    Mode = A->getValue();
    // Note: -cc1depscan does not accept '-fdepscan=off'.
    if (Mode != "inline" && Mode != "auto") {
      Diag.Report(diag::err_drv_invalid_argument_to_option)
          << Mode << A->getOption().getName();
      return 1;
    }
  }

  auto SaveArg = [&Args](const Twine &T) { return Args.MakeArgString(T); };
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
