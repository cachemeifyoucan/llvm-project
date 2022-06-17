//===- DependencyScanningWorker.cpp - clang-scan-deps worker --------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "clang/Tooling/DependencyScanning/DependencyScanningWorker.h"
#include "clang/CodeGen/ObjectFilePCHContainerOperations.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/CompilerInvocation.h"
#include "clang/Frontend/FrontendActions.h"
#include "clang/Frontend/TextDiagnosticPrinter.h"
#include "clang/Frontend/Utils.h"
#include "clang/Lex/PreprocessorOptions.h"
#include "clang/Tooling/DependencyScanning/DependencyScanningService.h"
#include "clang/Tooling/DependencyScanning/ModuleDepCollector.h"
#include "clang/Tooling/Tooling.h"
#include "llvm/CAS/CachingOnDiskFileSystem.h"

using namespace clang;
using namespace tooling;
using namespace dependencies;

namespace {

/// Forwards the gatherered dependencies to the consumer.
class DependencyConsumerForwarder : public DependencyFileGenerator {
public:
  DependencyConsumerForwarder(std::unique_ptr<DependencyOutputOptions> Opts,
                              DependencyConsumer &C,
                              bool EmitDependencyFile)
      : DependencyFileGenerator(*Opts), Opts(std::move(Opts)), C(C),
        EmitDependencyFile(EmitDependencyFile) {}

  void finishedMainFile(DiagnosticsEngine &Diags) override {
    C.handleDependencyOutputOpts(*Opts);
    llvm::SmallString<256> CanonPath;
    for (const auto &File : getDependencies()) {
      CanonPath = File;
      llvm::sys::path::remove_dots(CanonPath, /*remove_dot_dot=*/true);
      C.handleFileDependency(CanonPath);
    }
    if (EmitDependencyFile)
      DependencyFileGenerator::finishedMainFile(Diags);
  }

private:
  std::unique_ptr<DependencyOutputOptions> Opts;
  DependencyConsumer &C;
  bool EmitDependencyFile = false;
};

using PrebuiltModuleFilesT = decltype(HeaderSearchOptions::PrebuiltModuleFiles);

/// A listener that collects the imported modules and optionally the input
/// files.
class PrebuiltModuleListener : public ASTReaderListener {
public:
  PrebuiltModuleListener(PrebuiltModuleFilesT &PrebuiltModuleFiles,
                         llvm::StringSet<> &InputFiles, bool VisitInputFiles,
                         llvm::SmallVector<std::string> &NewModuleFiles)
      : PrebuiltModuleFiles(PrebuiltModuleFiles), InputFiles(InputFiles),
        VisitInputFiles(VisitInputFiles), NewModuleFiles(NewModuleFiles) {}

  bool needsImportVisitation() const override { return true; }
  bool needsInputFileVisitation() override { return VisitInputFiles; }
  bool needsSystemInputFileVisitation() override { return VisitInputFiles; }

  void visitImport(StringRef ModuleName, StringRef Filename) override {
    if (PrebuiltModuleFiles.insert({ModuleName.str(), Filename.str()}).second)
      NewModuleFiles.push_back(Filename.str());
  }

  bool visitInputFile(StringRef Filename, bool isSystem, bool isOverridden,
                      bool isExplicitModule) override {
    InputFiles.insert(Filename);
    return true;
  }

private:
  PrebuiltModuleFilesT &PrebuiltModuleFiles;
  llvm::StringSet<> &InputFiles;
  bool VisitInputFiles;
  llvm::SmallVector<std::string> &NewModuleFiles;
};

/// Visit the given prebuilt module and collect all of the modules it
/// transitively imports and contributing input files.
static void visitPrebuiltModule(StringRef PrebuiltModuleFilename,
                                CompilerInstance &CI,
                                PrebuiltModuleFilesT &ModuleFiles,
                                llvm::StringSet<> &InputFiles,
                                bool VisitInputFiles) {
  // List of module files to be processed.
  llvm::SmallVector<std::string> Worklist{PrebuiltModuleFilename.str()};
  PrebuiltModuleListener Listener(ModuleFiles, InputFiles, VisitInputFiles,
                                  Worklist);

  while (!Worklist.empty())
    ASTReader::readASTFileControlBlock(
        Worklist.pop_back_val(), CI.getFileManager(),
        CI.getPCHContainerReader(),
        /*FindModuleFileExtensions=*/false, Listener,
        /*ValidateDiagnosticOptions=*/false);
}

/// Transform arbitrary file name into an object-like file name.
static std::string makeObjFileName(StringRef FileName) {
  SmallString<128> ObjFileName(FileName);
  llvm::sys::path::replace_extension(ObjFileName, "o");
  return std::string(ObjFileName.str());
}

/// Deduce the dependency target based on the output file and input files.
static std::string
deduceDepTarget(const std::string &OutputFile,
                const SmallVectorImpl<FrontendInputFile> &InputFiles) {
  if (OutputFile != "-")
    return OutputFile;

  if (InputFiles.empty() || !InputFiles.front().isFile())
    return "clang-scan-deps\\ dependency";

  return makeObjFileName(InputFiles.front().getFile());
}

/// Sanitize diagnostic options for dependency scan.
static void sanitizeDiagOpts(DiagnosticOptions &DiagOpts) {
  // Don't print 'X warnings and Y errors generated'.
  DiagOpts.ShowCarets = false;
  // Don't write out diagnostic file.
  DiagOpts.DiagnosticSerializationFile.clear();
  // Don't treat warnings as errors.
  DiagOpts.Warnings.push_back("no-error");
}

/// A clang tool that runs the preprocessor in a mode that's optimized for
/// dependency scanning for the given compiler invocation.
class DependencyScanningAction : public tooling::ToolAction {
public:
  DependencyScanningAction(
      StringRef WorkingDirectory, DependencyConsumer &Consumer,
      const CASOptions &CASOpts,
      llvm::IntrusiveRefCntPtr<DependencyScanningWorkerFilesystem> DepFS,
      llvm::IntrusiveRefCntPtr<DependencyScanningCASFilesystem> DepCASFS,
      ScanningOutputFormat Format, bool OptimizeArgs, bool EmitDependencyFile,
      bool DisableFree, llvm::Optional<StringRef> ModuleName = None)
      : WorkingDirectory(WorkingDirectory), Consumer(Consumer),
        CASOpts(CASOpts), DepFS(std::move(DepFS)),
        DepCASFS(std::move(DepCASFS)), Format(Format),
        OptimizeArgs(OptimizeArgs), EmitDependencyFile(EmitDependencyFile),
        DisableFree(DisableFree), ModuleName(ModuleName) {}

  bool runInvocation(std::shared_ptr<CompilerInvocation> Invocation,
                     FileManager *FileMgr,
                     std::shared_ptr<PCHContainerOperations> PCHContainerOps,
                     DiagnosticConsumer *DiagConsumer) override {
    // Make a deep copy of the original Clang invocation.
    CompilerInvocation OriginalInvocation(*Invocation);
    // Restore the value of DisableFree, which may be modified by Tooling.
    OriginalInvocation.getFrontendOpts().DisableFree = DisableFree;

    // Create a compiler instance to handle the actual work.
    CompilerInstance ScanInstance(std::move(PCHContainerOps));
    ScanInstance.setInvocation(std::move(Invocation));
    ScanInstance.getInvocation().getCASOpts() = CASOpts;

    // Create the compiler's actual diagnostics engine.
    sanitizeDiagOpts(ScanInstance.getDiagnosticOpts());
    ScanInstance.createDiagnostics(DiagConsumer, /*ShouldOwnClient=*/false);
    if (!ScanInstance.hasDiagnostics())
      return false;

    ScanInstance.getPreprocessorOpts().AllowPCHWithDifferentModulesCachePath =
        true;

    ScanInstance.getFrontendOpts().GenerateGlobalModuleIndex = false;
    ScanInstance.getFrontendOpts().UseGlobalModuleIndex = false;

    FileMgr->getFileSystemOpts().WorkingDir = std::string(WorkingDirectory);
    ScanInstance.setFileManager(FileMgr);
    ScanInstance.createSourceManager(*FileMgr);

    llvm::StringSet<> PrebuiltModulesInputFiles;
    // Store the list of prebuilt module files into header search options. This
    // will prevent the implicit build to create duplicate modules and will
    // force reuse of the existing prebuilt module files instead.
    if (!ScanInstance.getPreprocessorOpts().ImplicitPCHInclude.empty())
      visitPrebuiltModule(
          ScanInstance.getPreprocessorOpts().ImplicitPCHInclude, ScanInstance,
          ScanInstance.getHeaderSearchOpts().PrebuiltModuleFiles,
          PrebuiltModulesInputFiles,
          /*VisitInputFiles=*/getDepScanFS() != nullptr);

    // Use the dependency scanning optimized file system if requested to do so.
    if (DepFS) {
      // Support for virtual file system overlays on top of the caching
      // filesystem.
      FileMgr->setVirtualFileSystem(createVFSFromCompilerInvocation(
          ScanInstance.getInvocation(), ScanInstance.getDiagnostics(), DepFS));

      llvm::IntrusiveRefCntPtr<DependencyScanningWorkerFilesystem> LocalDepFS =
          DepFS;
      ScanInstance.getPreprocessorOpts().DependencyDirectivesForFile =
          [LocalDepFS = std::move(LocalDepFS)](FileEntryRef File)
          -> Optional<ArrayRef<dependency_directives_scan::Directive>> {
        if (llvm::ErrorOr<EntryRef> Entry =
                LocalDepFS->getOrCreateFileSystemEntry(File.getName()))
          return Entry->getDirectiveTokens();
        return None;
      };
    }
    // CAS Implementation.
    if (DepCASFS) {
      // Support for virtual file system overlays on top of the caching
      // filesystem.
      FileMgr->setVirtualFileSystem(createVFSFromCompilerInvocation(
          ScanInstance.getInvocation(), ScanInstance.getDiagnostics(),
          DepCASFS));

      llvm::IntrusiveRefCntPtr<DependencyScanningCASFilesystem> LocalDepCASFS =
          DepCASFS;
      ScanInstance.getPreprocessorOpts().DependencyDirectivesForFile =
          [LocalDepCASFS = std::move(LocalDepCASFS)](FileEntryRef File)
          -> Optional<ArrayRef<dependency_directives_scan::Directive>> {
        return LocalDepCASFS->getDirectiveTokens(File.getName());
      };
    }

    // Create the dependency collector that will collect the produced
    // dependencies.
    //
    // This also moves the existing dependency output options from the
    // invocation to the collector. The options in the invocation are reset,
    // which ensures that the compiler won't create new dependency collectors,
    // and thus won't write out the extra '.d' files to disk.
    auto Opts = std::make_unique<DependencyOutputOptions>();
    std::swap(*Opts, ScanInstance.getInvocation().getDependencyOutputOpts());
    // We need at least one -MT equivalent for the generator of make dependency
    // files to work.
    if (Opts->Targets.empty())
      Opts->Targets = {
          deduceDepTarget(ScanInstance.getFrontendOpts().OutputFile,
                          ScanInstance.getFrontendOpts().Inputs)};
    Opts->IncludeSystemHeaders = true;

    switch (Format) {
    case ScanningOutputFormat::Make:
    case ScanningOutputFormat::Tree:
      ScanInstance.addDependencyCollector(
          std::make_shared<DependencyConsumerForwarder>(std::move(Opts),
                                                        Consumer,
                                                        EmitDependencyFile));
      break;
    case ScanningOutputFormat::Full:
    case ScanningOutputFormat::FullTree:
      ScanInstance.addDependencyCollector(std::make_shared<ModuleDepCollector>(
          std::move(Opts), ScanInstance, Consumer,
          std::move(OriginalInvocation), OptimizeArgs));
      break;
    }

    // Consider different header search and diagnostic options to create
    // different modules. This avoids the unsound aliasing of module PCMs.
    //
    // TODO: Implement diagnostic bucketing to reduce the impact of strict
    // context hashing.
    ScanInstance.getHeaderSearchOpts().ModulesStrictContextHash = true;

    std::unique_ptr<FrontendAction> Action;

    if (ModuleName.hasValue())
      Action = std::make_unique<GetDependenciesByModuleNameAction>(*ModuleName);
    else
      Action = std::make_unique<ReadPCHAndPreprocessAction>();

    const bool Result = ScanInstance.ExecuteAction(*Action);
    if (!getDepScanFS())
      FileMgr->clearStatCache();
    return Result;
  }

  IntrusiveRefCntPtr<llvm::vfs::FileSystem> getDepScanFS() {
    if (DepFS) {
      assert(!DepCASFS && "CAS DepFS should not be set");
      return DepFS;
    }
    if (DepCASFS) {
      assert(!DepFS && "DepFS should not be set");
      return DepCASFS;
    }
    return nullptr;
  }

private:
  StringRef WorkingDirectory;
  DependencyConsumer &Consumer;
  const CASOptions &CASOpts;
  llvm::IntrusiveRefCntPtr<DependencyScanningWorkerFilesystem> DepFS;
  llvm::IntrusiveRefCntPtr<DependencyScanningCASFilesystem> DepCASFS;
  ScanningOutputFormat Format;
  bool OptimizeArgs;
  bool EmitDependencyFile = false;
  bool DisableFree;
  llvm::Optional<StringRef> ModuleName;
};

} // end anonymous namespace

DependencyScanningWorker::DependencyScanningWorker(
    DependencyScanningService &Service)
    : Format(Service.getFormat()), OptimizeArgs(Service.canOptimizeArgs()),
      CASOpts(Service.getCASOpts()), UseCAS(Service.useCASScanning()) {
  PCHContainerOps = std::make_shared<PCHContainerOperations>();
  PCHContainerOps->registerReader(
      std::make_unique<ObjectFilePCHContainerReader>());
  // We don't need to write object files, but the current PCH implementation
  // requires the writer to be registered as well.
  PCHContainerOps->registerWriter(
      std::make_unique<ObjectFilePCHContainerWriter>());

  if (!Service.useCASScanning()) {
    auto OverlayFS = llvm::makeIntrusiveRefCnt<llvm::vfs::OverlayFileSystem>(
        llvm::vfs::createPhysicalFileSystem());
    InMemoryFS = llvm::makeIntrusiveRefCnt<llvm::vfs::InMemoryFileSystem>();
    OverlayFS->pushOverlay(InMemoryFS);
    RealFS = OverlayFS;
  } else {
    // FIXME: Need to teach CachingFileSystem to understand overlay.
    CacheFS = Service.getSharedFS().createProxyFS();
    RealFS = CacheFS;
  }

  if (Service.getMode() == ScanningMode::DependencyDirectivesScan) {
    if (Service.useCASScanning())
      DepCASFS = new DependencyScanningCASFilesystem(CacheFS);
    else
      DepFS = new DependencyScanningWorkerFilesystem(Service.getSharedCache(),
                                                     RealFS);
  }
  if (Service.canReuseFileManager())
    Files = new FileManager(FileSystemOptions(), RealFS);
}

llvm::IntrusiveRefCntPtr<FileManager>
DependencyScanningWorker::getOrCreateFileManager() const {
  if (Files)
    return Files;
  return new FileManager(FileSystemOptions(), RealFS);
}

static llvm::Error
runWithDiags(DiagnosticOptions *DiagOpts,
             llvm::function_ref<bool(DiagnosticConsumer &, DiagnosticOptions &)>
                 BodyShouldSucceed) {
  sanitizeDiagOpts(*DiagOpts);

  // Capture the emitted diagnostics and report them to the client
  // in the case of a failure.
  std::string DiagnosticOutput;
  llvm::raw_string_ostream DiagnosticsOS(DiagnosticOutput);
  TextDiagnosticPrinter DiagPrinter(DiagnosticsOS, DiagOpts);

  if (BodyShouldSucceed(DiagPrinter, *DiagOpts))
    return llvm::Error::success();
  return llvm::make_error<llvm::StringError>(DiagnosticsOS.str(),
                                             llvm::inconvertibleErrorCode());
}

llvm::Error DependencyScanningWorker::computeDependencies(
    StringRef WorkingDirectory, const std::vector<std::string> &CommandLine,
    DependencyConsumer &Consumer, llvm::Optional<StringRef> ModuleName) {
  // Reset what might have been modified in the previous worker invocation.
  RealFS->setCurrentWorkingDirectory(WorkingDirectory);
  if (Files)
    Files->setVirtualFileSystem(RealFS);

  llvm::IntrusiveRefCntPtr<FileManager> CurrentFiles =
      Files ? Files : new FileManager(FileSystemOptions(), RealFS);

  Optional<std::vector<std::string>> ModifiedCommandLine;
  if (ModuleName.hasValue()) {
    ModifiedCommandLine = CommandLine;
    InMemoryFS->addFile(*ModuleName, 0, llvm::MemoryBuffer::getMemBuffer(""));
    ModifiedCommandLine->emplace_back(*ModuleName);
  }

  const std::vector<std::string> &FinalCommandLine =
      ModifiedCommandLine ? *ModifiedCommandLine : CommandLine;

  std::vector<const char *> FinalCCommandLine(CommandLine.size(), nullptr);
  llvm::transform(CommandLine, FinalCCommandLine.begin(),
                  [](const std::string &Str) { return Str.c_str(); });

  return runWithDiags(CreateAndPopulateDiagOpts(FinalCCommandLine).release(),
                      [&](DiagnosticConsumer &DC, DiagnosticOptions &DiagOpts) {
                        // DisableFree is modified by Tooling for running
                        // in-process; preserve the original value, which is
                        // always true for a driver invocation.
                        bool DisableFree = true;
                        DependencyScanningAction Action(
                            WorkingDirectory, Consumer, getCASOpts(), DepFS,
                            DepCASFS, Format,
                            OptimizeArgs, /*EmitDependencyFile=*/false,
                            DisableFree, ModuleName);
                        // Create an invocation that uses the underlying file
                        // system to ensure that any file system requests that
                        // are made by the driver do not go through the
                        // dependency scanning filesystem.
                        ToolInvocation Invocation(FinalCommandLine, &Action,
                                                  CurrentFiles.get(),
                                                  PCHContainerOps);
                        Invocation.setDiagnosticConsumer(&DC);
                        Invocation.setDiagnosticOptions(&DiagOpts);
                        return Invocation.run();
                      });
}

void DependencyScanningWorker::computeDependenciesFromCompilerInvocation(
    std::shared_ptr<CompilerInvocation> Invocation, StringRef WorkingDirectory,
    DependencyConsumer &DepsConsumer, DiagnosticConsumer &DiagsConsumer) {
  RealFS->setCurrentWorkingDirectory(WorkingDirectory);

  // Adjust the invocation.
  auto &Frontend = Invocation->getFrontendOpts();
  Frontend.ProgramAction = frontend::RunPreprocessorOnly;
  Frontend.OutputFile = "/dev/null";
  Frontend.DisableFree = false;

  // // Reset dependency options.
  // Dependencies = DependencyOutputOptions();
  // Dependencies.IncludeSystemHeaders = true;
  // Dependencies.OutputFile = "/dev/null";

  // Make the output file path absolute relative to WorkingDirectory.
  std::string &DepFile = Invocation->getDependencyOutputOpts().OutputFile;
  if (!llvm::sys::path::is_absolute(DepFile)) {
    // FIXME: On Windows, WorkingDirectory is insufficient for making an
    // absolute path if OutputFile has a root name.
    llvm::SmallString<128> Path = StringRef(DepFile);
    llvm::sys::fs::make_absolute(WorkingDirectory, Path);
    DepFile = Path.str().str();
  }

  // FIXME: EmitDependencyFile should only be set when it's for a real
  // compilation.
  DependencyScanningAction Action(WorkingDirectory, DepsConsumer, getCASOpts(),
                                  DepFS, DepCASFS, Format,
                                  /*OptimizeArgs=*/false,
                                  /*EmitDependencyFile=*/true,
                                  /*DisableFree=*/false);

  // Ignore result; we're just collecting dependencies.
  //
  // FIXME: will clients other than -cc1scand care?
  IntrusiveRefCntPtr<FileManager> ActiveFiles = Files;
  if (!ActiveFiles) // Pass in RealFS via the file manager.
    ActiveFiles = new FileManager(Invocation->getFileSystemOpts(), RealFS);
  (void)Action.runInvocation(std::move(Invocation), ActiveFiles.get(),
                             PCHContainerOps, &DiagsConsumer);
}
