//===- CASOutputBackend.cpp -------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/CAS/CASOutputBackend.h"
#include "llvm/CAS/CASDB.h"
#include "llvm/CAS/Utils.h"
#include "llvm/Support/AlignOf.h"
#include "llvm/Support/Allocator.h"

using namespace llvm;
using namespace llvm::cas;

void CASOutputBackend::anchor() {}

namespace {
class CASOutputFile final : public vfs::OutputFileImpl {
public:
  Error keep() override { return OnKeep(Path, Bytes); }
  Error discard() override { return Error::success(); }
  raw_pwrite_stream &getOS() override { return OS; }

  using OnKeepType = llvm::unique_function<Error(StringRef, StringRef)>;
  CASOutputFile(StringRef Path, OnKeepType OnKeep)
      : Path(Path.str()), OS(Bytes), OnKeep(std::move(OnKeep)) {}

private:
  std::string Path;
  SmallString<16> Bytes;
  raw_svector_ostream OS;
  OnKeepType OnKeep;
};
} // namespace

CASOutputBackend::CASOutputBackend(std::shared_ptr<CASDB> CAS)
    : CASOutputBackend(*CAS) {
  this->OwnedCAS = std::move(CAS);
}

CASOutputBackend::CASOutputBackend(CASDB &CAS) : CAS(CAS) {}

CASOutputBackend::~CASOutputBackend() = default;

struct CASOutputBackend::PrivateImpl {
  // FIXME: Use a NodeBuilder here once it exists.
  SmallVector<CASID> IDs;
};

Expected<std::unique_ptr<vfs::OutputFileImpl>>
CASOutputBackend::createFileImpl(StringRef ResolvedPath,
                                 Optional<vfs::OutputConfig> Config) {
  if (!Impl)
    Impl = std::make_unique<PrivateImpl>();

  // FIXME: CASIDOutputBackend.createFile() should be called NOW (not inside
  // the OnKeep closure) so that if there are initialization errors (such as
  // output directory not existing) they're reported by createFileImpl().
  //
  // The opened file can be kept inside \a CASOutputFile and forwarded.
  return std::make_unique<CASOutputFile>(
      ResolvedPath, [&](StringRef Path, StringRef Bytes) -> Error {
        Optional<BlobProxy> PathBlob;
        Optional<BlobProxy> BytesBlob;
        if (Error E = CAS.createBlob(Path).moveInto(PathBlob))
          return E;
        if (Error E = CAS.createBlob(Bytes).moveInto(BytesBlob))
          return E;

        // FIXME: Should there be a lock taken before accessing PrivateImpl?
        Impl->IDs.push_back(PathBlob->getID());
        Impl->IDs.push_back(BytesBlob->getID());
        return Error::success();
      });
}

Expected<NodeProxy> CASOutputBackend::createNode() {
  // FIXME: Should there be a lock taken before accessing PrivateImpl?
  if (!Impl)
    return CAS.createNode(None, "");

  SmallVector<CASID> MovedIDs;
  std::swap(MovedIDs, Impl->IDs);
  return CAS.createNode(MovedIDs, "");
}

Error CASOutputBackend::addObject(StringRef Path, const CASID &Object) {
  if (!Impl)
    Impl = std::make_unique<PrivateImpl>();

  Optional<BlobProxy> PathBlob;
  if (Error E = CAS.createBlob(Path).moveInto(PathBlob))
    return E;

  Impl->IDs.push_back(PathBlob->getID());
  Impl->IDs.push_back(Object);
  return Error::success();
}
