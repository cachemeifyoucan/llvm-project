//===- TreeSchema.cpp -----------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/CAS/TreeSchema.h"
#include "llvm/Support/Alignment.h"
#include "llvm/Support/Endian.h"
#include "llvm/Support/EndianStream.h"
#include "llvm/Support/MemoryBufferRef.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/StringSaver.h"

using namespace llvm;
using namespace llvm::cas;

char TreeSchema::ID = 0;
constexpr StringLiteral TreeSchema::SchemaName;

void TreeSchema::anchor() {}

bool TreeSchema::isNode(const NodeHandle &Node) const {
  // Load the first ref to check its content.
  if (CAS.getNumRefs(Node) < 1)
    return false;

  // If can't load the first ref, consume error and return false.
  auto FirstRef = CAS.readRef(Node, 0);
  return CAS.getObjectID(FirstRef) == getKindID();
}

TreeSchema::TreeSchema(cas::CASDB &CAS) : TreeSchema::RTTIExtends(CAS) {
  auto Kind = cantFail(CAS.createNode(None, SchemaName));
  TreeKindID.emplace(CAS.getObjectID(Kind));
}

CASID TreeSchema::getKindID() const { return *TreeKindID; }

size_t TreeSchema::getNumTreeEntries(TreeNodeProxy Tree) const {
  return (CAS.getNumRefs(Tree) - 1);
}

Error TreeSchema::forEachTreeEntry(
    TreeNodeProxy Tree,
    function_ref<Error(const NamedTreeEntry &)> Callback) const {
  for (size_t I = 0, IE = getNumTreeEntries(Tree); I != IE; ++I)
    if (Error E = Callback(loadTreeEntry(Tree, I)))
      return E;

  return Error::success();
}

Error TreeSchema::walkFileTreeRecursively(
    CASDB &CAS, const ObjectHandle &Root,
    function_ref<Error(const NamedTreeEntry &, Optional<TreeNodeProxy>)>
        Callback) {
  BumpPtrAllocator Alloc;
  StringSaver Saver(Alloc);
  SmallString<128> PathStorage;
  SmallVector<NamedTreeEntry> Stack;
  Stack.emplace_back(CAS.getReference(Root), TreeEntry::Tree, "/");

  while (!Stack.empty()) {
    if (Stack.back().getKind() != TreeEntry::Tree) {
      if (Error E = Callback(Stack.pop_back_val(), None))
        return E;
      continue;
    }

    NamedTreeEntry Parent = Stack.pop_back_val();
    Expected<TreeNodeProxy> ExpTree = loadTree(Parent.getRef());
    if (Error E = ExpTree.takeError())
      return E;
    TreeNodeProxy Tree = *ExpTree;
    if (Error E = Callback(Parent, Tree))
      return E;
    for (int I = Tree.size(), E = 0; I != E; --I) {
      Optional<NamedTreeEntry> Child = Tree.get(I - 1);
      assert(Child && "Expected no corruption");

      SmallString<128> PathStorage = Parent.getName();
      sys::path::append(PathStorage, sys::path::Style::posix, Child->getName());
      Stack.emplace_back(Child->getRef(), Child->getKind(),
                         Saver.save(StringRef(PathStorage)));
    }
  }

  return Error::success();
}

static inline StringRef getNameFromNode(TreeNodeProxy Tree, size_t I) {
  // FIXME: Can assume alignment here?
  uint32_t StartIdx = support::endian::read<uint32_t, 1>(
      Tree.getData().data() + sizeof(uint32_t) * I,
      support::endianness::little);
  uint32_t EndIdx = support::endian::read<uint32_t, 1>(
      Tree.getData().data() + sizeof(uint32_t) * (I + 1),
      support::endianness::little);

  return StringRef(Tree.getData().data() + StartIdx, EndIdx - StartIdx);
}

NamedTreeEntry TreeSchema::loadTreeEntry(TreeNodeProxy Tree, size_t I) const {
  TreeEntry::EntryKind Kind =
      (TreeEntry::EntryKind)
          Tree.getData()[I + (Tree.size() + 1) * sizeof(uint32_t)];

  StringRef Name = getNameFromNode(Tree, I);
  auto ObjectRef = Tree.getReference(I + 1);

  return {ObjectRef, Kind, Name};
}

Optional<size_t> TreeSchema::lookupTreeEntry(TreeNodeProxy Tree,
                                             StringRef Name) const {
  size_t NumNames = Tree.size();
  if (!NumNames)
    return None;

  SmallVector<StringRef> Names(NumNames);

  // Start with a binary search, if there are enough entries.
  //
  // FIXME: Should just use std::lower_bound, but we need the actual iterators
  // to know the index in the NameCache...
  const size_t MaxLinearSearchSize = 4;
  size_t Last = NumNames;
  size_t First = 0;
  while (Last - First > MaxLinearSearchSize) {
    auto I = First + (Last - First) / 2;
    StringRef NameI = getNameFromNode(Tree, I);
    switch (Name.compare(NameI)) {
    case 0:
      return I;
    case -1:
      Last = I;
      break;
    case 1:
      First = I + 1;
      break;
    }
  }

  // Use a linear search for small trees.
  for (; First != Last; ++First)
    if (Name == getNameFromNode(Tree, First))
      return First;

  return None;
}

Expected<TreeNodeProxy> TreeSchema::loadTree(ObjectRef Object) const {
  auto TreeNode = CAS.loadNode(Object);
  if (!TreeNode)
    return TreeNode.takeError();

  return loadTree(*TreeNode);
}

Expected<TreeNodeProxy> TreeSchema::loadTree(NodeHandle Object) const {
  if (!isNode(Object))
    return createStringError(inconvertibleErrorCode(), "not a tree object");

  return TreeNodeProxy::get(*this, NodeProxy::load(CAS, Object));
}

Expected<TreeNodeProxy>
TreeSchema::storeTree(ArrayRef<NamedTreeEntry> Entries) {
  return TreeNodeProxy::create(*this, Entries);
}

Expected<ObjectRef> TreeSchema::storeTreeNodeName(StringRef Name) {
  auto Data = arrayRefFromStringRef<char>(Name);
  auto Node = CAS.storeNode({}, Data, false);
  if (!Node)
    return Node.takeError();

  return NodeProxy::load(CAS, *Node).getRef();
}

Expected<TreeNodeProxy> TreeNodeProxy::get(const TreeSchema &Schema,
                                           Expected<NodeProxy> Ref) {
  if (!Ref)
    return Ref.takeError();
  return TreeNodeProxy(Schema, *Ref);
}

Expected<TreeNodeProxy>
TreeNodeProxy::create(TreeSchema &Schema, ArrayRef<NamedTreeEntry> Entries) {
  // Ensure a stable order for tree entries and ignore name collisions.
  SmallVector<NamedTreeEntry> Sorted(Entries.begin(), Entries.end());
  std::stable_sort(Sorted.begin(), Sorted.end());
  Sorted.erase(std::unique(Sorted.begin(), Sorted.end()), Sorted.end());

  auto B = Builder::startNode(Schema);
  if (!B)
    return B.takeError();

  return B->build(Sorted);
}

Expected<TreeNodeProxy::Builder>
TreeNodeProxy::Builder::startNode(TreeSchema &Schema) {
  Builder B(Schema);
  B.IDs.push_back(Schema.getKindID());
  return std::move(B);
}

Expected<TreeNodeProxy>
TreeNodeProxy::Builder::build(ArrayRef<NamedTreeEntry> SortedEntries) {
  raw_svector_ostream OS(Data);
  support::endian::Writer Writer(OS, support::endianness::little);
  // Write Name.
  // The start of the string table index.
  uint32_t StrIdx = sizeof(uint8_t) * SortedEntries.size() +
                    sizeof(uint32_t) * (SortedEntries.size() + 1);
  for (auto &Entry : SortedEntries) {
    Writer.write(StrIdx);
    StrIdx += Entry.getName().size();

    // Append refs.
    IDs.push_back(Schema->CAS.getObjectID(Entry.getRef()));
  }
  Writer.write(StrIdx);

  // Write Kind.
  for (auto &Entry : SortedEntries)
    Writer.write((uint8_t)Entry.getKind());

  // Write names in the end of the block.
  for (auto &Entry : SortedEntries)
    OS << Entry.getName();

  return TreeNodeProxy::get(*Schema, Schema->CAS.createNode(IDs, Data));
}
