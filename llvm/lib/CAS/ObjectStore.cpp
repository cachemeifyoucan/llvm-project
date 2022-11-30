//===- ObjectStore.cpp ------------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/CAS/ObjectStore.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/SmallVectorMemoryBuffer.h"

using namespace llvm;
using namespace llvm::cas;

CASContext::~CASContext() {}
ObjectStore::~ObjectStore() {}

LLVM_DUMP_METHOD void CASID::dump() const { print(dbgs()); }
LLVM_DUMP_METHOD void ObjectStore::dump() const { print(dbgs()); }
LLVM_DUMP_METHOD void ObjectRef::dump() const { print(dbgs()); }
LLVM_DUMP_METHOD void ObjectHandle::dump() const { print(dbgs()); }

std::string CASID::toString() const {
  std::string S;
  raw_string_ostream(S) << *this;
  return S;
}

ArrayRef<uint8_t> CASID::getHash() const {
  return arrayRefFromStringRef<uint8_t>(Hash);
}

static void printReferenceBase(raw_ostream &OS, StringRef Kind,
                               uint64_t InternalRef, Optional<CASID> ID) {
  OS << Kind << "=" << InternalRef;
  if (ID)
    OS << "[" << *ID << "]";
}

void ReferenceBase::print(raw_ostream &OS, const ObjectHandle &This) const {
  assert(this == &This);

  Optional<CASID> ID;
#if LLVM_ENABLE_ABI_BREAKING_CHECKS
  if (CAS)
    ID = CAS->getID(This);
#endif
  printReferenceBase(OS, "object-handle", InternalRef, ID);
}

void ReferenceBase::print(raw_ostream &OS, const ObjectRef &This) const {
  assert(this == &This);

  Optional<CASID> ID;
#if LLVM_ENABLE_ABI_BREAKING_CHECKS
  if (CAS)
    ID = CAS->getID(This);
#endif
  printReferenceBase(OS, "object-ref", InternalRef, ID);
}

std::unique_ptr<MemoryBuffer>
ObjectStore::getMemoryBuffer(ObjectHandle Node, StringRef Name,
                             bool RequiresNullTerminator) {
  return MemoryBuffer::getMemBuffer(
      toStringRef(getData(Node, RequiresNullTerminator)), Name,
      RequiresNullTerminator);
}

Expected<ObjectProxy> ObjectStore::getProxy(const CASID &ID) {
  Optional<ObjectRef> Ref = getReference(ID);
  if (!Ref)
    return createUnknownObjectError(ID);

  Optional<ObjectHandle> H;
  if (Error E = load(*Ref).moveInto(H))
    return std::move(E);

  return ObjectProxy::load(*this, *H);
}

Expected<Optional<ObjectProxy>> ObjectStore::getProxyOrNone(const CASID &ID) {
  Optional<ObjectRef> Ref = getReference(ID);
  if (!Ref)
    return None;

  Optional<ObjectHandle> H;
  if (Error E = load(*Ref).moveInto(H))
    return std::move(E);

  return ObjectProxy::load(*this, *H);
}

Expected<ObjectProxy> ObjectStore::getProxy(ObjectRef Ref) {
  return getProxy(load(Ref));
}

Expected<ObjectProxy> ObjectStore::getProxy(Expected<ObjectHandle> H) {
  if (!H)
    return H.takeError();
  return ObjectProxy::load(*this, *H);
}

Error ObjectStore::createUnknownObjectError(const CASID &ID) {
  return createStringError(std::make_error_code(std::errc::invalid_argument),
                           "unknown object '" + ID.toString() + "'");
}

Expected<ObjectProxy> ObjectStore::createProxy(ArrayRef<ObjectRef> Refs,
                                               StringRef Data) {
  Expected<ObjectRef> Ref = store(Refs, arrayRefFromStringRef<char>(Data));
  if (!Ref)
    return Ref.takeError();
  return getProxy(*Ref);
}

std::unique_ptr<MemoryBuffer>
ObjectProxy::getMemoryBuffer(StringRef Name,
                             bool RequiresNullTerminator) const {
  return CAS->getMemoryBuffer(H, Name, RequiresNullTerminator);
}
