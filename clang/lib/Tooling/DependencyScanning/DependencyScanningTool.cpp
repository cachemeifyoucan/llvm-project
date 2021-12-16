//===- DependencyScanningTool.cpp - clang-scan-deps service ---------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "clang/Tooling/DependencyScanning/DependencyScanningTool.h"
#include "clang/Frontend/Utils.h"
#include "llvm/CAS/CachingOnDiskFileSystem.h"

namespace clang{
namespace tooling{
namespace dependencies{

std::vector<std::string> FullDependencies::getAdditionalArgs(
    std::function<StringRef(ModuleID)> LookupPCMPath,
    std::function<const ModuleDeps &(ModuleID)> LookupModuleDeps) const {
  std::vector<std::string> Ret = getAdditionalArgsWithoutModulePaths();

  std::vector<std::string> PCMPaths;
  std::vector<std::string> ModMapPaths;
  dependencies::detail::collectPCMAndModuleMapPaths(
      ClangModuleDeps, LookupPCMPath, LookupModuleDeps, PCMPaths, ModMapPaths);
  for (const std::string &PCMPath : PCMPaths)
    Ret.push_back("-fmodule-file=" + PCMPath);
  for (const std::string &ModMapPath : ModMapPaths)
    Ret.push_back("-fmodule-map-file=" + ModMapPath);

  return Ret;
}

std::vector<std::string>
FullDependencies::getAdditionalArgsWithoutModulePaths() const {
  std::vector<std::string> Args{
      "-fno-implicit-modules",
      "-fno-implicit-module-maps",
  };

  for (const PrebuiltModuleDep &PMD : PrebuiltModuleDeps) {
    Args.push_back("-fmodule-file=" + PMD.PCMFile);
    Args.push_back("-fmodule-map-file=" + PMD.ModuleMapFile);
  }

  return Args;
}

DependencyScanningTool::DependencyScanningTool(
    DependencyScanningService &Service)
    : Worker(Service) {}

llvm::Expected<std::string> DependencyScanningTool::getDependencyFile(
    const std::vector<std::string> &CommandLine, StringRef CWD,
    llvm::Optional<StringRef> ModuleName) {
  /// Prints out all of the gathered dependencies into a string.
  class MakeDependencyPrinterConsumer : public DependencyConsumer {
  public:
    void
    handleDependencyOutputOpts(const DependencyOutputOptions &Opts) override {
      this->Opts = std::make_unique<DependencyOutputOptions>(Opts);
    }

    void handleFileDependency(StringRef File) override {
      Dependencies.push_back(std::string(File));
    }

    void handlePrebuiltModuleDependency(PrebuiltModuleDep PMD) override {
      // Same as `handleModuleDependency`.
    }

    void handleModuleDependency(ModuleDeps MD) override {
      // These are ignored for the make format as it can't support the full
      // set of deps, and handleFileDependency handles enough for implicitly
      // built modules to work.
    }

    void handleContextHash(std::string Hash) override {}

    void printDependencies(std::string &S) {
      assert(Opts && "Handled dependency output options.");

      class DependencyPrinter : public DependencyFileGenerator {
      public:
        DependencyPrinter(DependencyOutputOptions &Opts,
                          ArrayRef<std::string> Dependencies)
            : DependencyFileGenerator(Opts) {
          for (const auto &Dep : Dependencies)
            addDependency(Dep);
        }

        void printDependencies(std::string &S) {
          llvm::raw_string_ostream OS(S);
          outputDependencyFile(OS);
        }
      };

      DependencyPrinter Generator(*Opts, Dependencies);
      Generator.printDependencies(S);
    }

  private:
    std::unique_ptr<DependencyOutputOptions> Opts;
    std::vector<std::string> Dependencies;
  };

  MakeDependencyPrinterConsumer Consumer;
  auto Result =
      Worker.computeDependencies(CWD, CommandLine, Consumer, ModuleName);
  if (Result)
    return std::move(Result);
  std::string Output;
  Consumer.printDependencies(Output);
  return Output;
}

namespace {
/// Returns a CAS tree containing the dependencies.
class MakeDependencyTree : public DependencyConsumer {
public:
  void handleFileDependency(StringRef File) override {
    // FIXME: Probably we want to delete this class, since we're getting
    // dependencies more accurately (including directories) by intercepting
    // filesystem accesses.
    //
    // On the other hand, for implicitly-discovered modules, we really want to
    // drop a bunch of extra dependencies from the directory iteration.
    //
    // For now just disable this.
    //
    // E = llvm::joinErrors(std::move(E), Builder->push(File));
  }

  void handleModuleDependency(ModuleDeps) override {}
  void handlePrebuiltModuleDependency(PrebuiltModuleDep) override {}
  void handleDependencyOutputOpts(const DependencyOutputOptions &) override {}

  void handleContextHash(std::string) override {}

  Expected<llvm::cas::TreeRef> makeTree() {
    if (E)
      return std::move(E);
    return Builder->create();
  }

  MakeDependencyTree(llvm::cas::CachingOnDiskFileSystem &FS)
      : E(llvm::Error::success()), Builder(FS.createTreeBuilder()) {}

  ~MakeDependencyTree() {
    // Ignore the error if makeTree wasn't called.
    llvm::consumeError(std::move(E));
  }

private:
  llvm::Error E;
  std::unique_ptr<llvm::cas::CachingOnDiskFileSystem::TreeBuilder> Builder;
};
}

llvm::Expected<llvm::cas::TreeRef> DependencyScanningTool::getDependencyTree(
    const std::vector<std::string> &CommandLine, StringRef CWD,
    llvm::Optional<StringRef> ModuleName) {
  llvm::cas::CachingOnDiskFileSystem &FS = Worker.getRealFS();
  FS.trackNewAccesses();
  MakeDependencyTree Consumer(FS);
  auto Result =
      Worker.computeDependencies(CWD, CommandLine, Consumer, ModuleName);
  if (Result)
    return std::move(Result);
  // return Consumer.makeTree();
  //
  // FIXME: This is needed because the dependency scanner doesn't track
  // directories are accessed -- in particular, we need the CWD to be included.
  // However, if we *want* to filter out certain accesses (such as for modules)
  // this will get in the way.
  //
  // The right fix is to add an API for listing directories that are
  // dependencies, and explicitly add the CWD and other things that matter.
  // (The 'make' output can ignore directories.)
  return FS.createTreeFromNewAccesses();
}

llvm::Expected<llvm::cas::TreeRef>
DependencyScanningTool::getDependencyTreeFromCompilerInvocation(
    std::shared_ptr<CompilerInvocation> Invocation, StringRef CWD,
    DiagnosticConsumer &DiagsConsumer,
    llvm::function_ref<StringRef(const llvm::vfs::CachedDirectoryEntry &)>
        RemapPath) {
  llvm::cas::CachingOnDiskFileSystem &FS = Worker.getRealFS();
  FS.trackNewAccesses();
  FS.setCurrentWorkingDirectory(CWD);
  MakeDependencyTree DepsConsumer(FS);
  Worker.computeDependenciesFromCompilerInvocation(std::move(Invocation), CWD,
                                                   DepsConsumer, DiagsConsumer);
  // return DepsConsumer.makeTree();
  //
  // FIXME: See FIXME in getDepencyTree().
  return FS.createTreeFromNewAccesses(RemapPath);
}

llvm::Expected<llvm::cas::TreeRef>
DependencyScanningTool::getDependencyTreeFromCC1CommandLine(
    ArrayRef<const char *> Args, StringRef CWD) {
  llvm::cas::CachingOnDiskFileSystem &FS = Worker.getRealFS();
  FS.trackNewAccesses();
  MakeDependencyTree DepsConsumer(FS);
  Worker.computeDependenciesFromCC1CommandLine(Args, CWD, DepsConsumer);
  // return DepsConsumer.makeTree();
  //
  // FIXME: See FIXME in getDepencyTree().
  return FS.createTreeFromNewAccesses();
}

llvm::Expected<FullDependenciesResult>
DependencyScanningTool::getFullDependencies(
    const std::vector<std::string> &CommandLine, StringRef CWD,
    const llvm::StringSet<> &AlreadySeen,
    llvm::Optional<StringRef> ModuleName) {
  class FullDependencyPrinterConsumer : public DependencyConsumer {
  public:
    FullDependencyPrinterConsumer(const llvm::StringSet<> &AlreadySeen)
        : AlreadySeen(AlreadySeen) {}

    void
    handleDependencyOutputOpts(const DependencyOutputOptions &Opts) override {}

    void handleFileDependency(StringRef File) override {
      Dependencies.push_back(std::string(File));
    }

    void handlePrebuiltModuleDependency(PrebuiltModuleDep PMD) override {
      PrebuiltModuleDeps.emplace_back(std::move(PMD));
    }

    void handleModuleDependency(ModuleDeps MD) override {
      ClangModuleDeps[MD.ID.ContextHash + MD.ID.ModuleName] = std::move(MD);
    }

    void handleContextHash(std::string Hash) override {
      ContextHash = std::move(Hash);
    }

    FullDependenciesResult getFullDependencies() const {
      FullDependencies FD;

      FD.ID.ContextHash = std::move(ContextHash);

      FD.FileDeps.assign(Dependencies.begin(), Dependencies.end());

      for (auto &&M : ClangModuleDeps) {
        auto &MD = M.second;
        if (MD.ImportedByMainFile)
          FD.ClangModuleDeps.push_back(MD.ID);
      }

      FD.PrebuiltModuleDeps = std::move(PrebuiltModuleDeps);

      FullDependenciesResult FDR;

      for (auto &&M : ClangModuleDeps) {
        // TODO: Avoid handleModuleDependency even being called for modules
        //   we've already seen.
        if (AlreadySeen.count(M.first))
          continue;
        FDR.DiscoveredModules.push_back(std::move(M.second));
      }

      FDR.FullDeps = std::move(FD);
      return FDR;
    }

  private:
    std::vector<std::string> Dependencies;
    std::vector<PrebuiltModuleDep> PrebuiltModuleDeps;
    std::map<std::string, ModuleDeps> ClangModuleDeps;
    std::string ContextHash;
    std::vector<std::string> OutputPaths;
    const llvm::StringSet<> &AlreadySeen;
  };

  FullDependencyPrinterConsumer Consumer(AlreadySeen);
  llvm::Error Result =
      Worker.computeDependencies(CWD, CommandLine, Consumer, ModuleName);
  if (Result)
    return std::move(Result);
  return Consumer.getFullDependencies();
}

} // end namespace dependencies
} // end namespace tooling
} // end namespace clang
