//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/CAS/CASConfiguration.h"
#include "CASTestConfig.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/CAS/ActionCache.h"
#include "llvm/CAS/ObjectStore.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/VirtualFileSystem.h"
#include "llvm/Testing/Support/Error.h"
#include "llvm/Testing/Support/SupportHelpers.h"
#include "gtest/gtest.h"

using namespace llvm;
using namespace llvm::cas;

using CASDatabases =
    std::pair<std::shared_ptr<ObjectStore>, std::shared_ptr<ActionCache>>;

TEST(CASConfigurationTest, isConfigured) {
  CASConfiguration Config;
  EXPECT_FALSE(Config);

  Config.CASPath = "/tmp";
  EXPECT_TRUE(Config);

  Config.CASPath.clear();
  Config.PluginPath = "/test.plug";
  EXPECT_TRUE(Config);
}

TEST(CASConfigurationTest, equality) {
  CASConfiguration A, B;
  EXPECT_EQ(A, B);

  A.CASPath = "/tmp";
  EXPECT_NE(A, B);
  B.CASPath = "/tmp";
  EXPECT_EQ(A, B);

  A.PluginOptions.emplace_back("a", "b");
  EXPECT_NE(A, B);
  B.PluginOptions.emplace_back("a", "b");
  EXPECT_EQ(A, B);
}

TEST(CASConfigurationTest, createInMemory) {
  CASConfiguration Config;
  std::optional<CASDatabases> DBs;
  ASSERT_THAT_ERROR(Config.createDatabases().moveInto(DBs), Succeeded());
  ASSERT_TRUE(DBs->first);
  ASSERT_TRUE(DBs->second);

  std::optional<CASID> ID;
  ASSERT_THAT_ERROR(DBs->first->createProxy({}, "data").moveInto(ID),
                    Succeeded());
  EXPECT_TRUE(DBs->first->getReference(*ID));
}

TEST(CASConfigurationTest, createOnDisk) {
  unittest::cas::setMaxOnDiskCASMappingSize();
  unittest::TempDir Temp("cas-configuration", /*Unique=*/true);
  CASConfiguration Config;
  Config.CASPath = std::string(Temp.path("cas"));

  std::optional<CASID> ID;
  {
    std::optional<CASDatabases> DBs;
    ASSERT_THAT_ERROR(Config.createDatabases().moveInto(DBs), Succeeded());
    ASSERT_THAT_ERROR(DBs->first->createProxy({}, "data").moveInto(ID),
                      Succeeded());
  }

  // Reopen and check that the object was persisted at CASPath.
  std::optional<CASDatabases> DBs;
  ASSERT_THAT_ERROR(Config.createDatabases().moveInto(DBs), Succeeded());
  std::optional<ObjectRef> Ref = DBs->first->getReference(*ID);
  ASSERT_TRUE(Ref);
  bool IsMaterialized = false;
  ASSERT_THAT_ERROR(DBs->first->isMaterialized(*Ref).moveInto(IsMaterialized),
                    Succeeded());
  EXPECT_TRUE(IsMaterialized);
}

TEST(CASConfigurationTest, resolveAutoPath) {
  SmallString<128> DefaultPath;
  ASSERT_THAT_ERROR(getDefaultOnDiskCASPath(DefaultPath), Succeeded());
  EXPECT_FALSE(DefaultPath.empty());

  std::optional<std::string> DefaultPathStr;
  ASSERT_THAT_ERROR(getDefaultOnDiskCASPath().moveInto(DefaultPathStr),
                    Succeeded());
  EXPECT_EQ(DefaultPath, *DefaultPathStr);

  CASConfiguration Config;
  Config.CASPath = "auto";
  SmallString<128> Resolved;
  ASSERT_THAT_ERROR(Config.getResolvedCASPath(Resolved), Succeeded());
  EXPECT_EQ(DefaultPath, Resolved);

  Config.CASPath = "/some/path";
  ASSERT_THAT_ERROR(Config.getResolvedCASPath(Resolved), Succeeded());
  EXPECT_EQ(Resolved, "/some/path");
}

TEST(CASConfigurationTest, roundTrips) {
  auto roundTripConfig = [](CASConfiguration &Config) {
    std::string Serialized;
    raw_string_ostream OS(Serialized);
    Config.writeConfigurationFile(OS);

    std::optional<CASConfiguration> NewConfig;
    ASSERT_THAT_ERROR(
        CASConfiguration::createFromConfig(Serialized).moveInto(NewConfig),
        Succeeded());
    EXPECT_EQ(Config, *NewConfig);
  };

  CASConfiguration Config;
  roundTripConfig(Config);

  Config.CASPath = "/tmp";
  roundTripConfig(Config);

  Config.PluginPath = "/test.plug";
  roundTripConfig(Config);

  Config.PluginOptions.emplace_back("a", "b");
  roundTripConfig(Config);

  Config.PluginOptions.emplace_back("c", "d");
  roundTripConfig(Config);
}

TEST(CASConfigurationTest, configFileSearch) {
  // /a/b/c
  SmallString<261> PathToC = sys::path::get_separator();
  sys::path::append(PathToC, "a", "b", "c");
  // /a/b/c/d
  SmallString<261> PathToD = PathToC;
  sys::path::append(PathToD, "d");
  // /a/b/c/d/e
  SmallString<261> PathToE = PathToD;
  sys::path::append(PathToE, "e");
  // /a/b/c/.cas-config
  SmallString<261> PathToCConfig = PathToC;
  sys::path::append(PathToCConfig, ".cas-config");
  // /a/b/c/d/.cas-config
  SmallString<261> PathToDConfig = PathToD;
  sys::path::append(PathToDConfig, ".cas-config");

  auto VFS = makeIntrusiveRefCnt<vfs::InMemoryFileSystem>();
  ASSERT_FALSE(
      CASConfiguration::createFromSearchConfigFile(PathToE.str(), VFS));

  // Add an empty file.
  VFS->addFile(PathToCConfig.str(), 0,
               llvm::MemoryBuffer::getMemBufferCopy(""));
  ASSERT_FALSE(
      CASConfiguration::createFromSearchConfigFile(PathToE.str(), VFS));

  VFS->addFile(PathToDConfig.str(), 0,
#ifndef _WIN32
               llvm::MemoryBuffer::getMemBufferCopy("{\"CASPath\": \"/tmp\"}"));
#else
               llvm::MemoryBuffer::getMemBufferCopy(
                   "{\"CASPath\": \"\\\\tmp\"}"));
#endif

  CASConfiguration Config;
  SmallString<261> CASPath = sys::path::get_separator();
  sys::path::append(CASPath, "tmp");
  Config.CASPath = CASPath.str();
  auto NewConfig =
      CASConfiguration::createFromSearchConfigFile(PathToE.str(), VFS);
  ASSERT_TRUE(NewConfig);
  EXPECT_EQ(NewConfig->first, PathToDConfig.str());
  EXPECT_EQ(Config, NewConfig->second);
}

TEST(CASConfigurationTest, denseMapKey) {
  CASConfiguration B, C;
  B.CASPath = "/tmp";
  C.PluginPath = "/test.plug";
  DenseSet<CASConfiguration> Set;
  EXPECT_TRUE(Set.insert(B).second);
  EXPECT_TRUE(Set.insert(C).second);
  EXPECT_FALSE(Set.insert(B).second);
  EXPECT_TRUE(Set.contains(B));
  EXPECT_TRUE(Set.contains(C));
}
