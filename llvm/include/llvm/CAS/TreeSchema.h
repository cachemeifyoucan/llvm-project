//===- llvm/CAS/TreeSchema.h ------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CAS_TREESCHEMA_H
#define LLVM_CAS_TREESCHEMA_H

#include "llvm/CAS/CASDB.h"
#include "llvm/CAS/CASNodeSchema.h"
#include "llvm/CAS/TreeEntry.h"

namespace llvm {
namespace cas {

class TreeNodeProxy;

class TreeSchema : public RTTIExtends<TreeSchema, NodeSchema> {
  void anchor() override;

public:
  static char ID;
  bool isRootNode(const NodeHandle &Node) const final {
    return false; // TreeSchema doesn't have a root node.
  }
  bool isNode(const NodeHandle &Node) const final;

  TreeSchema(CASDB &CAS);

  // Tree Schema.
  CASID getKindID();

  size_t getNumTreeEntries(TreeNodeProxy Tree) const;

  Error
  forEachTreeEntry(TreeNodeProxy Tree,
                   function_ref<Error(const NamedTreeEntry &)> Callback) const;

  Optional<size_t> lookupTreeEntry(TreeNodeProxy Tree, StringRef Name) const;

  NamedTreeEntry loadTreeEntry(TreeNodeProxy Tree, size_t I) const;

  Expected<TreeNodeProxy> loadTree(ObjectRef Object) const;
  Expected<TreeNodeProxy> loadTree(NodeHandle Object) const;

  Expected<TreeNodeProxy> storeTree(ArrayRef<NamedTreeEntry> Entries = None);

  Expected<ObjectRef> storeTreeNodeName(StringRef Name);

private:
  static constexpr StringLiteral SchemaName = "llvm::cas::schema::tree::v1";
  Optional<CASID> TreeKindID;
};

class TreeNodeProxy : public NodeProxy {
public:
  static Expected<TreeNodeProxy> get(const TreeSchema &Schema,
                                     Expected<NodeProxy> Ref);

  static Expected<TreeNodeProxy> create(TreeSchema &Schema,
                                        ArrayRef<NamedTreeEntry> Entries);

  const TreeSchema &getSchema() const { return *Schema; }

  bool operator==(const TreeNodeProxy &RHS) const {
    return Schema == RHS.Schema && cas::CASID(*this) == cas::CASID(RHS);
  }

  Error
  forEachEntry(function_ref<Error(const NamedTreeEntry &)> Callback) const {
    return Schema->forEachTreeEntry(*this, Callback);
  }

  bool empty() const { return size() == 0; }
  size_t size() const { return Schema->getNumTreeEntries(*this); }

  Optional<NamedTreeEntry> lookup(StringRef Name) const {
    if (auto I = Schema->lookupTreeEntry(*this, Name))
      return get(*I);
    return None;
  }

  NamedTreeEntry get(size_t I) const { return Schema->loadTreeEntry(*this, I); }

  TreeNodeProxy() = delete;

private:
  TreeNodeProxy(const TreeSchema &Schema, const NodeProxy &Node)
      : NodeProxy(Node), Schema(&Schema) {}

  class Builder {
  public:
    static Expected<Builder> startNode(TreeSchema &Schema);

    Expected<TreeNodeProxy> build();

  private:
    Builder(const TreeSchema &Schema) : Schema(&Schema) {}
    const TreeSchema *Schema;

  public:
    SmallString<256> Data;
    SmallVector<CASID, 16> IDs;
  };
  const TreeSchema *Schema;
};

} // namespace cas
} // namespace llvm

#endif // LLVM_CAS_TREESCHEMA_H
