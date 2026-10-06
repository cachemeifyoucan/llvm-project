//===- TableGenCache.cpp - Compilation caching for TableGen ---------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "TableGenCache.h"
#include "TGParser.h"
#include "llvm/ADT/ScopeExit.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/CAS/NamedValuesSchema.h"
#include "llvm/Support/BLAKE3.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/VirtualFileSystem.h"
#include "llvm/TableGen/Error.h"
#include "llvm/TableGen/Record.h"

using namespace llvm;
using namespace llvm::tablegen;

static constexpr StringLiteral ActionKind = "llvm::tablegen::cache::action::v1";

Expected<std::string> tablegen::hashExecutable(vfs::FileSystem &FS,
                                               const char *Argv0) {
  std::string Path = sys::fs::getMainExecutable(
      Argv0, reinterpret_cast<void *>(&tablegen::hashExecutable));
  ErrorOr<std::unique_ptr<MemoryBuffer>> Buffer = FS.getBufferForFile(Path);
  if (!Buffer)
    return createStringError(Buffer.getError(), "could not hash executable '" +
                                                    Path + "': " +
                                                    Buffer.getError().message());
  return toHex(BLAKE3::hash(arrayRefFromStringRef((*Buffer)->getBuffer())),
               /*LowerCase=*/true);
}

/// Report errors from the scan, along with the notes attached to them, and drop
/// everything else. \p Context points to whether the last diagnostic that was
/// not a note was an error.
static void handleScanDiagnostic(const SMDiagnostic &Diag, void *Context) {
  bool &InError = *static_cast<bool *>(Context);
  if (Diag.getKind() != SourceMgr::DK_Note)
    InError = Diag.getKind() == SourceMgr::DK_Error;
  if (!InError)
    return;
  // Print it the way SrcMgr would without a handler, with the include stack.
  SrcMgr.setDiagHandler(nullptr);
  SrcMgr.PrintMessage(errs(), Diag);
  SrcMgr.setDiagHandler(handleScanDiagnostic, Context);
}

std::optional<std::vector<std::string>>
tablegen::scanDependencies(IntrusiveRefCntPtr<vfs::FileSystem> FS,
                           std::unique_ptr<MemoryBuffer> MainFile,
                           ArrayRef<std::string> IncludeDirs,
                           ArrayRef<std::string> Macros) {
  // TableGen reports diagnostics through the global SrcMgr, so the parser has
  // to use it. Hand it back empty for the run that follows; the records refer
  // to its buffers, so it is reset after they are destroyed.
  assert(SrcMgr.getNumBuffers() == 0 && "SrcMgr is in use");
  scope_exit ResetSrcMgr([] { SrcMgr = SourceMgr(); });

  bool InError = false;
  SrcMgr.setDiagHandler(handleScanDiagnostic, &InError);
  SrcMgr.AddNewSourceBuffer(std::move(MainFile), SMLoc());
  SrcMgr.setIncludeDirs(IncludeDirs);
  SrcMgr.setVirtualFileSystem(std::move(FS));

  // The parser records each file it includes. Including is all that matters
  // here, but it is interleaved with building records, so build them too.
  RecordKeeper Records;
  TGParser Parser(SrcMgr, Macros, Records,
                  /*NoWarnOnUnusedTemplateArgs=*/true);
  if (Parser.ParseFile())
    return std::nullopt;
  return std::vector<std::string>(Parser.getDependencies().begin(),
                                  Parser.getDependencies().end());
}

/// Join \p Strings, terminating each with a NUL.
static std::string flattenStrings(ArrayRef<std::string> Strings) {
  std::string Flattened;
  for (StringRef S : Strings) {
    Flattened += S;
    Flattened.push_back(0);
  }
  return Flattened;
}

static Expected<std::vector<std::string>> splitStrings(StringRef Flattened) {
  std::vector<std::string> Strings;
  while (!Flattened.empty()) {
    size_t Null = Flattened.find('\0');
    if (Null == StringRef::npos)
      return createStringError("malformed TableGen cache object: unterminated "
                               "string list");
    Strings.push_back(Flattened.take_front(Null).str());
    Flattened = Flattened.drop_front(Null + 1);
  }
  return Strings;
}

/// Store \p Data as a blob and add it to \p Builder as \p Name.
static Error addBlob(cas::ObjectStore &CAS,
                     cas::NamedValuesSchema::Builder &Builder,
                     const Twine &Name, StringRef Data) {
  Expected<cas::ObjectRef> Ref = CAS.storeFromString({}, Data);
  if (!Ref)
    return Ref.takeError();
  Builder.add(Name.str(), *Ref);
  return Error::success();
}

/// Load the blob named \p Name in \p Node.
static Expected<StringRef> loadBlob(const cas::NamedValuesProxy &Node,
                                    StringRef Name) {
  std::optional<cas::NamedValuesEntry> Entry = Node.lookup(Name);
  if (!Entry)
    return createStringError("malformed TableGen cache object: missing '" +
                             Name + "'");
  Expected<cas::ObjectProxy> Blob = Node.getCAS().getProxy(Entry->Ref);
  if (!Blob)
    return Blob.takeError();
  return Blob->getData();
}

Expected<cas::ObjectProxy> tablegen::storeAction(cas::ObjectStore &CAS,
                                                 const CacheAction &Action) {
  assert(Action.Inputs && "action has no inputs");
  cas::NamedValuesSchema::Builder Builder(CAS);
  if (Error E = addBlob(CAS, Builder, "command-line",
                        flattenStrings(Action.CommandLine)))
    return std::move(E);
  if (Error E = addBlob(CAS, Builder, "executable", Action.Executable))
    return std::move(E);
  if (Error E = addBlob(CAS, Builder, "kind", ActionKind))
    return std::move(E);
  if (Error E =
          addBlob(CAS, Builder, "working-directory", Action.WorkingDirectory))
    return std::move(E);
  Builder.add("inputs", Action.Inputs->getRef());

  Expected<cas::NamedValuesProxy> Node = Builder.build();
  if (!Node)
    return Node.takeError();
  return *Node;
}

Expected<CacheAction> tablegen::loadAction(cas::ObjectStore &CAS,
                                           cas::ObjectRef Key) {
  Expected<cas::NamedValuesSchema> Schema = cas::NamedValuesSchema::create(CAS);
  if (!Schema)
    return Schema.takeError();
  Expected<cas::NamedValuesProxy> Node = Schema->load(Key);
  if (!Node)
    return Node.takeError();

  Expected<StringRef> Kind = loadBlob(*Node, "kind");
  if (!Kind)
    return Kind.takeError();
  if (*Kind != ActionKind)
    return createStringError("not a TableGen cache key");

  CacheAction Action;
  Expected<StringRef> CommandLine = loadBlob(*Node, "command-line");
  if (!CommandLine)
    return CommandLine.takeError();
  if (Error E = splitStrings(*CommandLine).moveInto(Action.CommandLine))
    return std::move(E);
  Expected<StringRef> Executable = loadBlob(*Node, "executable");
  if (!Executable)
    return Executable.takeError();
  Action.Executable = Executable->str();
  Expected<StringRef> WorkingDirectory = loadBlob(*Node, "working-directory");
  if (!WorkingDirectory)
    return WorkingDirectory.takeError();
  Action.WorkingDirectory = WorkingDirectory->str();
  std::optional<cas::NamedValuesEntry> Inputs = Node->lookup("inputs");
  if (!Inputs)
    return createStringError(
        "malformed TableGen cache object: missing 'inputs'");
  Expected<cas::FileSystemSchema> InputsSchema =
      cas::FileSystemSchema::create(CAS);
  if (!InputsSchema)
    return InputsSchema.takeError();
  if (Error E = InputsSchema->load(Inputs->Ref).moveInto(Action.Inputs))
    return std::move(E);
  return Action;
}

Expected<cas::ObjectRef> tablegen::storeResult(cas::ObjectStore &CAS,
                                               const CacheResult &Result) {
  cas::NamedValuesSchema::Builder Builder(CAS);
  if (Error E = addBlob(CAS, Builder, "output", Result.MainFile))
    return std::move(E);
  for (const auto &[Suffix, Contents] : Result.AdditionalFiles)
    if (Error E = addBlob(CAS, Builder, "output:" + Suffix, Contents))
      return std::move(E);

  Expected<cas::NamedValuesProxy> Node = Builder.build();
  if (!Node)
    return Node.takeError();
  return Node->getRef();
}

Expected<CacheResult> tablegen::loadResult(cas::ObjectStore &CAS,
                                           cas::ObjectRef Ref) {
  Expected<cas::NamedValuesSchema> Schema = cas::NamedValuesSchema::create(CAS);
  if (!Schema)
    return Schema.takeError();
  Expected<cas::NamedValuesProxy> Node = Schema->load(Ref);
  if (!Node)
    return Node.takeError();

  CacheResult Result;
  Expected<StringRef> MainFile = loadBlob(*Node, "output");
  if (!MainFile)
    return MainFile.takeError();
  Result.MainFile = MainFile->str();

  // Entries are sorted by name, so additional outputs come back in the same
  // order they were produced in, which is sorted by suffix.
  for (size_t I = 0, E = Node->size(); I != E; ++I) {
    StringRef Name = Node->getName(I);
    if (!Name.consume_front("output:"))
      continue;
    Expected<StringRef> Contents = loadBlob(*Node, Node->getName(I));
    if (!Contents)
      return Contents.takeError();
    Result.AdditionalFiles.emplace_back(Name.str(), Contents->str());
  }
  return Result;
}
