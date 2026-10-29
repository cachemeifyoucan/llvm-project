//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "clang/Basic/DiagnosticCAS.h"
#include "clang/CAS/IncludeTree.h"
#include "clang/DependencyScanning/CachingActions.h"
#include "clang/DependencyScanning/ScanAndUpdateArgs.h"
#include "clang/Frontend/CompileJobCacheKey.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Lex/Preprocessor.h"
#include "llvm/CAS/ObjectStore.h"
#include "llvm/Support/PrefixMapper.h"
#include "llvm/Support/PrefixMappingFileSystem.h"

using namespace clang;
using namespace dependencies;
using llvm::Error;

namespace {
class IncludeTreeBuilder;

class IncludeTreeActionController : public CallbackActionController {
public:
  IncludeTreeActionController(CASOptions CASOpts, cas::ObjectStore &DB,
                              LookupModuleOutputCallback LookupOutput)
      : CallbackActionController(LookupOutput), CASOpts(std::move(CASOpts)),
        DB(DB) {}

  Expected<cas::IncludeTreeRoot> getIncludeTree();

private:
  std::unique_ptr<DependencyActionController> clone() const override;

  void initializeScanInvocation(CompilerInvocation &ScanInvocation) override;
  bool initialize(CompilerInstance &ScanInstance,
                  CompilerInvocation &NewInvocation) override;
  bool finalize(CompilerInstance &ScanInstance,
                CowCompilerInvocation &NewInvocation) override;
  std::optional<std::string>
  getCacheKey(const CompilerInvocation &NewInvocation) override;

private:
  IncludeTreeBuilder &current() {
    assert(!BuilderStack.empty());
    return *BuilderStack.back();
  }

private:
  CASOptions CASOpts;
  cas::ObjectStore &DB;
  llvm::PrefixMapper PrefixMapper;
  // IncludeTreePPCallbacks keeps a pointer to the current builder, so use a
  // pointer so the builder cannot move when resizing.
  SmallVector<std::unique_ptr<IncludeTreeBuilder>> BuilderStack;
  std::optional<cas::IncludeTreeRoot> IncludeTreeResult;
  llvm::StringMap<std::string> OutputToCacheKey;
};

/// Callbacks for building an include-tree for a given translation unit.
class IncludeTreeBuilder {
public:
  IncludeTreeBuilder(cas::ObjectStore &DB, llvm::PrefixMapper &PrefixMapper)
      : DB(DB), PrefixMapper(PrefixMapper) {}

  Expected<cas::IncludeTreeRoot>
  finishIncludeTree(CompilerInstance &ScanInstance,
                    const CompilerInvocationBase &NewInvocation);

  void enteredInclude(Preprocessor &PP, FileID FID);

  void exitedInclude(Preprocessor &PP, FileID IncludedBy, FileID Include,
                     SourceLocation ExitLoc);

  void handleHasIncludeCheck(Preprocessor &PP, bool Result);

private:
  struct FilePPState {
    SrcMgr::CharacteristicKind FileCharacteristic;
    cas::ObjectRef File;
    SmallVector<cas::IncludeTree::IncludeInfo, 6> Includes;
    llvm::SmallBitVector HasIncludeChecks;
  };

  Error addModuleInputs(ASTReader &Reader);
  Expected<cas::ObjectRef> getObjectForFile(Preprocessor &PP, FileID FID);
  Expected<cas::ObjectRef>
  getObjectForFileNonCached(FileManager &FM, const SrcMgr::FileInfo &FI);
  Expected<cas::ObjectRef> getObjectForBuffer(const SrcMgr::FileInfo &FI);
  Expected<cas::ObjectRef> addToFileList(FileManager &FM, FileEntryRef FE);
  Expected<cas::IncludeTree> getCASTreeForFileIncludes(FilePPState &&PPState);
  Expected<cas::IncludeTree::File> createIncludeFile(StringRef Filename,
                                                     cas::ObjectRef Contents);

  bool hasErrorOccurred() const { return ErrorToReport.has_value(); }

  template <typename T> std::optional<T> check(Expected<T> &&E) {
    if (!E) {
      ErrorToReport = E.takeError();
      return std::nullopt;
    }
    return *E;
  }

private:
  cas::ObjectStore &DB;
  llvm::PrefixMapper &PrefixMapper;

  std::optional<cas::ObjectRef> PCHRef;
  SmallVector<cas::IncludeTree::FileList::FileEntry> IncludedFiles;
  SmallVector<cas::ObjectRef> IncludedFileLists;
  std::optional<cas::ObjectRef> PredefinesBufferRef;
  SmallVector<FilePPState> IncludeStack;
  llvm::DenseMap<const FileEntry *, std::optional<cas::ObjectRef>>
      ObjectForFile;
  std::optional<llvm::Error> ErrorToReport;
};

/// A utility for adding \c PPCallbacks and/or \cASTReaderListener to a compiler
/// instance at the appropriate time.
struct AttachOnlyDependencyCollector : public DependencyCollector {
  using MakePPCB =
      llvm::unique_function<std::unique_ptr<PPCallbacks>(Preprocessor &)>;
  using MakeASTReaderL =
      llvm::unique_function<std::unique_ptr<ASTReaderListener>(ASTReader &R)>;
  MakePPCB CreatePPCB;
  MakeASTReaderL CreateASTReaderL;
  AttachOnlyDependencyCollector(MakePPCB CreatePPCB, MakeASTReaderL CreateL)
      : CreatePPCB(std::move(CreatePPCB)),
        CreateASTReaderL(std::move(CreateL)) {}

  void attachToPreprocessor(Preprocessor &PP) final {
    if (CreatePPCB) {
      std::unique_ptr<PPCallbacks> CB = CreatePPCB(PP);
      assert(CB);
      PP.addPPCallbacks(std::move(CB));
    }
  }

  void attachToASTReader(ASTReader &R) final {
    if (CreateASTReaderL) {
      std::unique_ptr<ASTReaderListener> L = CreateASTReaderL(R);
      assert(L);
      R.addListener(std::move(L));
    }
  }
};

struct IncludeTreePPCallbacks : public PPCallbacks {
  IncludeTreeBuilder &Builder;
  Preprocessor &PP;

public:
  IncludeTreePPCallbacks(IncludeTreeBuilder &Builder, Preprocessor &PP)
      : Builder(Builder), PP(PP) {}

  void LexedFileChanged(FileID FID, LexedFileChangeReason Reason,
                        SrcMgr::CharacteristicKind FileType, FileID PrevFID,
                        SourceLocation Loc) override {
    switch (Reason) {
    case LexedFileChangeReason::EnterFile:
      Builder.enteredInclude(PP, FID);
      break;
    case LexedFileChangeReason::ExitFile: {
      Builder.exitedInclude(PP, FID, PrevFID, Loc);
      break;
    }
    }
  }

  void HasInclude(SourceLocation Loc, StringRef FileName, bool IsAngled,
                  OptionalFileEntryRef File,
                  SrcMgr::CharacteristicKind FileType) override {
    Builder.handleHasIncludeCheck(PP, File.has_value());
  }
};
} // namespace

/// The PCH recorded file paths with canonical paths, create a VFS that
/// allows remapping back to the non-canonical source paths so that they are
/// found during dep-scanning.
void dependencies::addReversePrefixMappingFileSystem(
    const llvm::PrefixMapper &PrefixMapper, CompilerInstance &ScanInstance) {
  llvm::PrefixMapper ReverseMapper;
  ReverseMapper.addInverseRange(PrefixMapper.getMappings());
  ReverseMapper.sort();
  IntrusiveRefCntPtr<llvm::vfs::FileSystem> FS =
      llvm::vfs::createPrefixMappingFileSystem(
          std::move(ReverseMapper), &ScanInstance.getVirtualFileSystem());

  ScanInstance.setVirtualFileSystem(FS);
  ScanInstance.getFileManager().setVirtualFileSystem(std::move(FS));
}

Expected<cas::IncludeTreeRoot> IncludeTreeActionController::getIncludeTree() {
  if (IncludeTreeResult)
    return *IncludeTreeResult;
  return llvm::createStringError(llvm::inconvertibleErrorCode(),
                                 "failed to produce include-tree");
}

std::unique_ptr<DependencyActionController>
IncludeTreeActionController::clone() const {
  return std::make_unique<IncludeTreeActionController>(CASOpts, DB,
                                                       LookupModuleOutput);
}

void IncludeTreeActionController::initializeScanInvocation(
    CompilerInvocation &ScanInvocation) {
  // Enable caching in the resulting commands.
  ScanInvocation.getFrontendOpts().CacheCompileJob = true;
  ScanInvocation.getCASOpts() = CASOpts;
}

bool IncludeTreeActionController::initialize(
    CompilerInstance &ScanInstance, CompilerInvocation &NewInvocation) {
  // Clang modules are not supported by include-tree scanning yet.
  if (ScanInstance.getLangOpts().Modules ||
      !NewInvocation.getFrontendOpts().ModuleMapFiles.empty()) {
    ScanInstance.getDiagnostics().Report(diag::err_cas_depscan_failed)
        << "include-tree scanning does not support clang modules";
    return false;
  }

  DepscanPrefixMapping::configurePrefixMapper(NewInvocation, PrefixMapper);

  auto ensurePathRemapping = [&]() {
    if (PrefixMapper.empty())
      return;

    PreprocessorOptions &PPOpts = ScanInstance.getPreprocessorOpts();
    if (PPOpts.Includes.empty() && PPOpts.ImplicitPCHInclude.empty() &&
        !ScanInstance.getLangOpts().Modules)
      return;

    addReversePrefixMappingFileSystem(PrefixMapper, ScanInstance);

    // TODO: Confirm why it's not enough to do this in
    // DepscanPrefixMapping::remapInvocationPaths.
    // These are written in the predefines buffer, so we need to remap them.
    for (std::string &Include : PPOpts.Includes)
      PrefixMapper.mapInPlace(Include);
  };
  ensurePathRemapping();

  BuilderStack.push_back(
      std::make_unique<IncludeTreeBuilder>(DB, PrefixMapper));

  // Attach callbacks for the IncludeTree of the TU. The preprocessor
  // does not exist yet, so we need to indirect this via DependencyCollector.
  auto DC = std::make_shared<AttachOnlyDependencyCollector>(
      [&Builder = current()](Preprocessor &PP) {
        return std::make_unique<IncludeTreePPCallbacks>(Builder, PP);
      },
      /*CreateASTReaderL=*/nullptr);
  ScanInstance.addDependencyCollector(std::move(DC));

  return true;
}

bool IncludeTreeActionController::finalize(
    CompilerInstance &ScanInstance, CowCompilerInvocation &NewInvocation) {
  auto GetInputCacheKey = [&]() -> std::optional<StringRef> {
    if (NewInvocation.getFrontendOpts().Inputs.size() != 1)
      return {};
    const auto &FIF = NewInvocation.getFrontendOpts().Inputs.front();
    if (!FIF.isFile())
      return {};
    auto It = OutputToCacheKey.find(FIF.getFile());
    if (It == OutputToCacheKey.end())
      return {};

    return It->second;
  };

  std::string InputID;
  CachingInputKind InputKind;
  if (auto InputCacheKey = GetInputCacheKey()) {
    InputID = InputCacheKey->str();
    InputKind = CachingInputKind::CachedCompilation;
  } else {
    assert(!IncludeTreeResult);
    assert(BuilderStack.size() == 1);
    auto Builder = BuilderStack.pop_back_val();
    Error E = Builder->finishIncludeTree(ScanInstance, NewInvocation)
                  .moveInto(IncludeTreeResult);
    if (E) {
      ScanInstance.getDiagnostics().Report(diag::err_cas_depscan_failed)
          << std::move(E);
      return false;
    }
    InputID = IncludeTreeResult->getID().toString();
    InputKind = CachingInputKind::IncludeTree;
  }

  configureInvocationForCaching(NewInvocation, CASOpts, InputID, InputKind);

  DepscanPrefixMapping::remapInvocationPaths(NewInvocation, PrefixMapper);

  auto &CAS = ScanInstance.getOrCreateObjectStore();
  // FIXME: Make this return an error and propagate it up.
  auto Key = createCompileJobCacheKey(CAS, ScanInstance.getDiagnostics(),
                                      NewInvocation);
  if (Key)
    OutputToCacheKey[NewInvocation.getFrontendOpts().OutputFile] =
        Key->toString();
  return true;
}

std::optional<std::string> IncludeTreeActionController::getCacheKey(
    const CompilerInvocation &NewInvocation) {
  auto It = OutputToCacheKey.find(NewInvocation.getFrontendOpts().OutputFile);
  // FIXME: Assert this does not happen.
  if (It == OutputToCacheKey.end())
    return std::nullopt;
  return It->second;
}

void IncludeTreeBuilder::enteredInclude(Preprocessor &PP, FileID FID) {
  if (hasErrorOccurred())
    return;

  std::optional<cas::ObjectRef> FileRef = check(getObjectForFile(PP, FID));
  if (!FileRef)
    return;
  const SrcMgr::FileInfo &FI =
      PP.getSourceManager().getSLocEntry(FID).getFile();
  IncludeStack.push_back({FI.getFileCharacteristic(), *FileRef, {}, {}});
}

void IncludeTreeBuilder::exitedInclude(Preprocessor &PP, FileID IncludedBy,
                                       FileID Include, SourceLocation ExitLoc) {
  if (hasErrorOccurred())
    return;

  assert(*check(getObjectForFile(PP, Include)) == IncludeStack.back().File);
  std::optional<cas::IncludeTree> IncludeTree =
      check(getCASTreeForFileIncludes(IncludeStack.pop_back_val()));
  if (!IncludeTree)
    return;
  assert(*check(getObjectForFile(PP, IncludedBy)) == IncludeStack.back().File);
  SourceManager &SM = PP.getSourceManager();
  std::pair<FileID, unsigned> LocInfo = SM.getDecomposedExpansionLoc(ExitLoc);

  IncludeStack.back().Includes.push_back({IncludeTree->getRef(), LocInfo.second,
                                          cas::IncludeTree::NodeKind::Tree});
}

void IncludeTreeBuilder::handleHasIncludeCheck(Preprocessor &PP, bool Result) {
  if (hasErrorOccurred())
    return;

  IncludeStack.back().HasIncludeChecks.push_back(Result);
}

Expected<cas::IncludeTreeRoot> IncludeTreeBuilder::finishIncludeTree(
    CompilerInstance &ScanInstance,
    const CompilerInvocationBase &NewInvocation) {
  if (ErrorToReport)
    return std::move(*ErrorToReport);

  FileManager &FM = ScanInstance.getFileManager();

  auto addFile = [&](StringRef FilePath,
                     bool IgnoreFileError = false) -> Error {
    if (FilePath.empty())
      return Error::success();
    llvm::Expected<FileEntryRef> FE = FM.getFileRef(FilePath);
    if (!FE) {
      auto Err = FE.takeError();
      if (IgnoreFileError) {
        llvm::consumeError(std::move(Err));
        return Error::success();
      }
      return Err;
    }
    std::optional<cas::ObjectRef> Ref;
    return addToFileList(FM, *FE).moveInto(Ref);
  };

  for (StringRef FilePath : NewInvocation.getLangOpts().NoSanitizeFiles) {
    if (Error E = addFile(FilePath))
      return std::move(E);
  }
  // Add profile files.
  // FIXME: Do not have the logic here to determine which path should be set
  // but ideally only the path needed for the compilation is set and we already
  // checked the file needed exists. Just try load and ignore errors.
  if (Error E = addFile(NewInvocation.getCodeGenOpts().ProfileInstrumentUsePath,
                        /*IgnoreFileError=*/true))
    return std::move(E);
  if (Error E = addFile(NewInvocation.getCodeGenOpts().SampleProfileFile,
                        /*IgnoreFileError=*/true))
    return std::move(E);
  if (Error E = addFile(NewInvocation.getCodeGenOpts().ProfileRemappingFile,
                        /*IgnoreFileError=*/true))
    return std::move(E);

  StringRef Sysroot = NewInvocation.getHeaderSearchOpts().Sysroot;
  if (!Sysroot.empty()) {
    // Include 'SDKSettings.json', if it exists, to accomodate availability
    // checks during the compilation.
    llvm::SmallString<256> FilePath = Sysroot;
    llvm::sys::path::append(FilePath, "SDKSettings.json");
    if (Error E = addFile(FilePath, /*IgnoreFileError*/ true))
      return std::move(E);
  }

  auto FinishIncludeTree = [&]() -> Error {
    IntrusiveRefCntPtr<ASTReader> Reader = ScanInstance.getASTReader();
    if (!Reader)
      return Error::success(); // no need for additional work.

    // Go through all the recorded input files.
    if (Error E = addModuleInputs(*Reader))
      return E;

    const PreprocessorOptions &PPOpts = NewInvocation.getPreprocessorOpts();
    if (PPOpts.ImplicitPCHInclude.empty())
      return Error::success(); // no need for additional work.

    llvm::ErrorOr<cas::ObjectRef> CASContents =
        FM.getObjectRefForFileContent(PPOpts.ImplicitPCHInclude);
    if (!CASContents)
      return llvm::createFileError(PPOpts.ImplicitPCHInclude,
                                   CASContents.getError());

    // Use a unique 'filename' for the PCH to make sure there are no conflicts
    // during lookup if there is another chained parent PCH.
    SmallString<140> PCHFileName;
    {
      llvm::raw_svector_ostream OS(PCHFileName);
      OS << "<PCH-";
      OS << llvm::toHex(DB.getID(*CASContents).getHash());
      OS << '>';
    }
    auto PCHFile =
        cas::IncludeTree::File::create(DB, PCHFileName.str(), *CASContents);
    if (!PCHFile)
      return PCHFile.takeError();
    PCHRef = PCHFile->getRef();
    return llvm::Error::success();
  };

  if (Error E = FinishIncludeTree())
    return std::move(E);

  if (ErrorToReport)
    return std::move(*ErrorToReport);

  assert(IncludeStack.size() == 1);
  Expected<cas::IncludeTree> MainIncludeTree =
      getCASTreeForFileIncludes(IncludeStack.pop_back_val());
  if (!MainIncludeTree)
    return MainIncludeTree.takeError();

  auto FileList =
      cas::IncludeTree::FileList::create(DB, IncludedFiles, IncludedFileLists);
  if (!FileList)
    return FileList.takeError();

  return cas::IncludeTreeRoot::create(DB, MainIncludeTree->getRef(),
                                      FileList->getRef(), PCHRef);
}

Error IncludeTreeBuilder::addModuleInputs(ASTReader &Reader) {
  for (serialization::ModuleFile &MF : Reader.getModuleManager()) {
    // Only add direct imports to avoid duplication. Each include tree is a
    // superset of its imported modules' include trees.
    if (!MF.isDirectlyImported())
      continue;

    assert(!MF.IncludeTreeID.empty() && "missing include-tree for import");

    std::optional<cas::CASID> ID;
    if (Error E = DB.parseID(MF.IncludeTreeID).moveInto(ID))
      return E;
    std::optional<cas::ObjectRef> Ref = DB.getReference(*ID);
    if (!Ref)
      return DB.createUnknownObjectError(*ID);
    std::optional<cas::IncludeTreeRoot> Root;
    if (Error E = cas::IncludeTreeRoot::get(DB, *Ref).moveInto(Root))
      return E;

    IncludedFileLists.push_back(Root->getFileListRef());
  }

  return Error::success();
}

Expected<cas::ObjectRef> IncludeTreeBuilder::getObjectForFile(Preprocessor &PP,
                                                              FileID FID) {
  SourceManager &SM = PP.getSourceManager();
  const SrcMgr::FileInfo &FI = SM.getSLocEntry(FID).getFile();
  if (PP.getPredefinesFileID() == FID) {
    if (!PredefinesBufferRef) {
      auto Ref = getObjectForBuffer(FI);
      if (!Ref)
        return Ref.takeError();
      PredefinesBufferRef = *Ref;
    }
    return *PredefinesBufferRef;
  }
  assert(FI.getContentCache().OrigEntry);
  auto &FileRef = ObjectForFile[*FI.getContentCache().OrigEntry];
  if (!FileRef) {
    auto Ref = getObjectForFileNonCached(SM.getFileManager(), FI);
    if (!Ref)
      return Ref.takeError();
    FileRef = *Ref;
  }
  return *FileRef;
}

Expected<cas::ObjectRef>
IncludeTreeBuilder::getObjectForFileNonCached(FileManager &FM,
                                              const SrcMgr::FileInfo &FI) {
  OptionalFileEntryRef FE = FI.getContentCache().OrigEntry;
  assert(FE);
  return addToFileList(FM, *FE);
}

Expected<cas::ObjectRef>
IncludeTreeBuilder::getObjectForBuffer(const SrcMgr::FileInfo &FI) {
  // This is a non-file buffer, like the predefines.
  auto Ref = DB.storeFromString(
      {}, FI.getContentCache().getBufferIfLoaded()->getBuffer());
  if (!Ref)
    return Ref.takeError();
  Expected<cas::IncludeTree::File> FileNode =
      createIncludeFile(FI.getName(), *Ref);
  if (!FileNode)
    return FileNode.takeError();
  return FileNode->getRef();
}

Expected<cas::ObjectRef> IncludeTreeBuilder::addToFileList(FileManager &FM,
                                                           FileEntryRef FE) {
  SmallString<128> PathStorage;
  StringRef Filename = FE.getName();
  // Apply -working-directory to relative paths. This option causes filesystem
  // lookups to use absolute paths, so make paths in the include-tree filesystem
  // absolute to match.
  if (!llvm::sys::path::is_absolute(Filename) &&
      !FM.getFileSystemOpts().WorkingDir.empty()) {
    PathStorage = Filename;
    FM.FixupRelativePath(PathStorage);
    Filename = PathStorage;
  }

  llvm::ErrorOr<cas::ObjectRef> CASContents =
      FM.getObjectRefForFileContent(Filename);
  if (!CASContents)
    return llvm::createFileError(Filename, CASContents.getError());

  auto addFile = [&](StringRef Filename) -> Expected<cas::ObjectRef> {
    assert(!Filename.empty());
    auto FileNode = createIncludeFile(Filename, *CASContents);
    if (!FileNode)
      return FileNode.takeError();
    IncludedFiles.push_back(
        {FileNode->getRef(),
         static_cast<cas::IncludeTree::FileList::FileSizeTy>(FE.getSize())});
    return FileNode->getRef();
  };

  return addFile(Filename);
}

Expected<cas::IncludeTree>
IncludeTreeBuilder::getCASTreeForFileIncludes(FilePPState &&PPState) {
  return cas::IncludeTree::create(DB, PPState.FileCharacteristic, PPState.File,
                                  PPState.Includes, PPState.HasIncludeChecks);
}

Expected<cas::IncludeTree::File>
IncludeTreeBuilder::createIncludeFile(StringRef Filename,
                                      cas::ObjectRef Contents) {
  SmallString<256> MappedPath;
  if (!PrefixMapper.empty()) {
    PrefixMapper.map(Filename, MappedPath);
    Filename = MappedPath;
  }
  return cas::IncludeTree::File::create(DB, Filename, std::move(Contents));
}

std::unique_ptr<DependencyActionController>
dependencies::createIncludeTreeActionController(
    LookupModuleOutputCallback LookupModuleOutput, CASOptions CASOpts,
    cas::ObjectStore &DB) {
  return std::make_unique<IncludeTreeActionController>(std::move(CASOpts), DB,
                                                       LookupModuleOutput);
}
