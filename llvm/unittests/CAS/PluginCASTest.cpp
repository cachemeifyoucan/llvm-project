//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Tests the plugin-backed \c ObjectStore and \c ActionCache against the mock
/// plugin implementation in \c llvm/unittests/CAS/CASPluginTest.
///
//===----------------------------------------------------------------------===//

#include "CASTestConfig.h"
#include "OnDiskCommonUtils.h"
#include "llvm/CAS/ActionCache.h"
#include "llvm/CAS/CASConfiguration.h"
#include "llvm/CAS/ObjectStore.h"
#include "llvm/Config/llvm-config.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/Path.h"
#include "llvm/Testing/Support/Error.h"
#include "llvm/Testing/Support/SupportHelpers.h"
#include "gtest/gtest.h"

using namespace llvm;
using namespace llvm::cas;
using namespace llvm::unittest::cas;

// HWASan does not tag the globals of a dlopen'ed library with glibc, so the
// plugin faults as soon as it touches one of its own globals.
// FIXME: Re-enable once https://github.com/llvm/llvm-project/issues/57206 is
// fixed.
#if !LLVM_HWADDRESS_SANITIZER_BUILD

/// \returns the path of the CASPluginTest dynamic library, which implements
/// the CAS plugin API for testing purposes.
static std::string getCASPluginPath() { return CAS_PLUGIN_PATH; }

static CASTestingEnv createPlugin(int I) {
  unittest::TempDir Temp("plugin-cas", /*Unique=*/true);
  std::optional<
      std::pair<std::shared_ptr<ObjectStore>, std::shared_ptr<ActionCache>>>
      DBs;
  EXPECT_THAT_ERROR(createPluginCASDatabases(getCASPluginPath(), Temp.path(),
                                             /*PluginArgs=*/{})
                        .moveInto(DBs),
                    Succeeded());
  if (!DBs)
    return CASTestingEnv{nullptr, nullptr, std::move(Temp)};
  return CASTestingEnv{std::move(DBs->first), std::move(DBs->second),
                       std::move(Temp)};
}
INSTANTIATE_TEST_SUITE_P(PluginCAS, CASTest, ::testing::Values(createPlugin));

TEST(PluginCASTest, isMaterialized) {
  unittest::TempDir Temp("plugin-cas", /*Unique=*/true);
  std::string UpDir(Temp.path("up"));
  std::string DownDir(Temp.path("down"));
  std::pair<std::string, std::string> PluginOpts[] = {
      {"upstream-path", std::string(UpDir)}};

  {
    std::optional<
        std::pair<std::shared_ptr<ObjectStore>, std::shared_ptr<ActionCache>>>
        DBs;
    ASSERT_THAT_ERROR(
        createPluginCASDatabases(getCASPluginPath(), DownDir, PluginOpts)
            .moveInto(DBs),
        Succeeded());
    std::shared_ptr<ObjectStore> CAS;
    std::shared_ptr<ActionCache> AC;
    std::tie(CAS, AC) = std::move(*DBs);

    std::optional<CASID> ID1, ID2;
    ASSERT_THAT_ERROR(CAS->createProxy({}, "1").moveInto(ID1), Succeeded());
    ASSERT_THAT_ERROR(CAS->createProxy({}, "2").moveInto(ID2), Succeeded());
    std::optional<ObjectRef> ID2Ref = CAS->getReference(*ID2);
    ASSERT_TRUE(ID2Ref);
    bool IsMaterialized = false;
    ASSERT_THAT_ERROR(CAS->isMaterialized(*ID2Ref).moveInto(IsMaterialized),
                      Succeeded());
    EXPECT_TRUE(IsMaterialized);
    ASSERT_THAT_ERROR(AC->put(*ID1, *ID2, /*CanBeDistributed=*/true),
                      Succeeded());
  }

  // Clear "local" cache.
  sys::fs::remove_directories(DownDir);

  {
    std::optional<
        std::pair<std::shared_ptr<ObjectStore>, std::shared_ptr<ActionCache>>>
        DBs;
    ASSERT_THAT_ERROR(
        createPluginCASDatabases(getCASPluginPath(), DownDir, PluginOpts)
            .moveInto(DBs),
        Succeeded());
    std::shared_ptr<ObjectStore> CAS;
    std::shared_ptr<ActionCache> AC;
    std::tie(CAS, AC) = std::move(*DBs);

    std::optional<CASID> ID1, ID2;
    ASSERT_THAT_ERROR(CAS->createProxy({}, "1").moveInto(ID1), Succeeded());
    ASSERT_THAT_ERROR(AC->get(*ID1, /*CanBeDistributed=*/true).moveInto(ID2),
                      Succeeded());
    std::optional<ObjectRef> ID2Ref = CAS->getReference(*ID2);
    ASSERT_TRUE(ID2Ref);
    bool IsMaterialized = false;
    ASSERT_THAT_ERROR(CAS->isMaterialized(*ID2Ref).moveInto(IsMaterialized),
                      Succeeded());
    EXPECT_FALSE(IsMaterialized);

    std::optional<ObjectProxy> Obj;
    ASSERT_THAT_ERROR(CAS->getProxy(*ID2Ref).moveInto(Obj), Succeeded());
    ASSERT_THAT_ERROR(CAS->isMaterialized(*ID2Ref).moveInto(IsMaterialized),
                      Succeeded());
    EXPECT_TRUE(IsMaterialized);
  }
}

TEST(PluginCASTest, parseInvalidID) {
  unittest::TempDir Temp("plugin-cas", /*Unique=*/true);
  std::pair<std::string, std::string> PluginOpts[] = {
      {"first-prefix", "first~"}, {"second-prefix", "second~"}};

  std::optional<
      std::pair<std::shared_ptr<ObjectStore>, std::shared_ptr<ActionCache>>>
      DBs;
  ASSERT_THAT_ERROR(
      createPluginCASDatabases(getCASPluginPath(), Temp.path(), PluginOpts)
          .moveInto(DBs),
      Succeeded());
  std::shared_ptr<ObjectStore> CAS = DBs->first;

  std::optional<CASID> ID;
  ASSERT_THAT_ERROR(CAS->createProxy({}, "1").moveInto(ID), Succeeded());
  std::string PrintedID = ID->toString();
  ASSERT_TRUE(StringRef(PrintedID).starts_with("first~second~"));

  std::optional<CASID> ParsedID;
  ASSERT_THAT_ERROR(CAS->parseID(PrintedID).moveInto(ParsedID), Succeeded());
  EXPECT_EQ(ID, ParsedID);

  // Missing or mismatched prefixes are reported as errors.
  StringRef Digest = StringRef(PrintedID).drop_front(strlen("first~second~"));
  EXPECT_THAT_EXPECTED(CAS->parseID(Digest), Failed());
  EXPECT_THAT_EXPECTED(CAS->parseID(("first~" + Digest).str()), Failed());
  EXPECT_THAT_EXPECTED(CAS->parseID(("second~" + Digest).str()), Failed());
  EXPECT_THAT_EXPECTED(CAS->parseID(("second~first~" + Digest).str()),
                       Failed());
  EXPECT_THAT_EXPECTED(CAS->parseID(""), Failed());
}

TEST(PluginCASTest, validate) {
  unittest::TempDir Temp("plugin-cas", /*Unique=*/true);

  auto validateIfNeeded = [&](bool Force) {
    return validatePluginCASDatabasesIfNeeded(getCASPluginPath(), Temp.path(),
                                              /*PluginArgs=*/{},
                                              /*CheckHash=*/true, Force);
  };
  auto recover = [&]() {
    return recoverPluginCASDatabases(getCASPluginPath(), Temp.path(),
                                     /*PluginArgs=*/{});
  };
  auto openCAS = [&]() {
    std::optional<
        std::pair<std::shared_ptr<ObjectStore>, std::shared_ptr<ActionCache>>>
        DBs;
    EXPECT_THAT_ERROR(createPluginCASDatabases(getCASPluginPath(), Temp.path(),
                                               /*PluginArgs=*/{})
                          .moveInto(DBs),
                      Succeeded());
    return DBs;
  };

  std::optional<ValidationResult> Result;
  ASSERT_THAT_ERROR(validateIfNeeded(/*Force=*/false).moveInto(Result),
                    Succeeded());
  EXPECT_EQ(Result, ValidationResult::Valid);

  {
    auto DBs = openCAS();
    ASSERT_TRUE(DBs);
    auto &[CAS, AC] = *DBs;

    std::optional<CASID> ID1, ID2;
    ASSERT_THAT_ERROR(CAS->createProxy({}, "1").moveInto(ID1), Succeeded());
    ASSERT_THAT_ERROR(CAS->createProxy({}, "2").moveInto(ID2), Succeeded());
    ASSERT_THAT_ERROR(AC->put(*ID1, *ID2), Succeeded());

    EXPECT_THAT_ERROR(CAS->validate(/*CheckHash=*/true), Succeeded());
    EXPECT_THAT_ERROR(AC->validate(), Succeeded());
  }

  // Already validated since boot.
  const ValidationResult ValidatedSinceBoot =
      isBootTimeKnown() ? ValidationResult::Skipped : ValidationResult::Valid;
  ASSERT_THAT_ERROR(validateIfNeeded(/*Force=*/false).moveInto(Result),
                    Succeeded());
  EXPECT_EQ(Result, ValidatedSinceBoot);

  ASSERT_THAT_ERROR(validateIfNeeded(/*Force=*/true).moveInto(Result),
                    Succeeded());
  EXPECT_EQ(Result, ValidationResult::Valid);

  // Nothing to recover from after a successful validation.
  ASSERT_THAT_ERROR(recover().moveInto(Result), Succeeded());
  EXPECT_EQ(Result, ValidationResult::Skipped);
  EXPECT_TRUE(sys::fs::exists(Temp.path("v1.1")));

  // Invalidate the data.
  ASSERT_FALSE(sys::fs::remove(Temp.path("v1.1/data.v1")));
  EXPECT_THAT_EXPECTED(validateIfNeeded(/*Force=*/true), Failed());

  // Recovery requires exclusive access, and fails rather than waiting for it.
  {
    auto DBs = openCAS();
    ASSERT_TRUE(DBs);
    EXPECT_THAT_EXPECTED(recover(), Failed());
  }

  ASSERT_THAT_ERROR(recover().moveInto(Result), Succeeded());
  EXPECT_EQ(Result, ValidationResult::Recovered);
  EXPECT_FALSE(sys::fs::exists(Temp.path("v1.1")));

  // A concurrent recovery for the same failed validation is skipped.
  ASSERT_THAT_ERROR(recover().moveInto(Result), Succeeded());
  EXPECT_EQ(Result, ValidationResult::Skipped);

  // Recovery counts as validation for this boot.
  ASSERT_THAT_ERROR(validateIfNeeded(/*Force=*/false).moveInto(Result),
                    Succeeded());
  EXPECT_EQ(Result, ValidatedSinceBoot);

  std::pair<std::string, std::string> BadOpts[] = {{"bogus", ""}};
  EXPECT_THAT_EXPECTED(
      validatePluginCASDatabasesIfNeeded(getCASPluginPath(), Temp.path(),
                                         BadOpts, /*CheckHash=*/false,
                                         /*ForceValidation=*/true),
      Failed());
  EXPECT_THAT_EXPECTED(
      recoverPluginCASDatabases(getCASPluginPath(), Temp.path(), BadOpts),
      Failed());
}

TEST(PluginCASTest, CASConfigurationEmptyPath) {
  // An empty CASPath creates an in-memory CAS even when a plugin is set, so
  // the plugin is never loaded.
  CASConfiguration Config;
  Config.PluginPath = "/does/not/exist";
  std::optional<
      std::pair<std::shared_ptr<ObjectStore>, std::shared_ptr<ActionCache>>>
      DBs;
  ASSERT_THAT_ERROR(Config.createDatabases().moveInto(DBs), Succeeded());
  EXPECT_TRUE(DBs->first);
  EXPECT_TRUE(DBs->second);
}

TEST(PluginCASTest, CASConfigurationOnDiskPath) {
  unittest::TempDir Temp("plugin-cas", /*Unique=*/true);
  CASConfiguration Config;
  Config.CASPath = std::string(Temp.path("cas"));
  Config.PluginPath = getCASPluginPath();

  std::optional<CASID> ID;
  {
    std::optional<
        std::pair<std::shared_ptr<ObjectStore>, std::shared_ptr<ActionCache>>>
        DBs;
    ASSERT_THAT_ERROR(Config.createDatabases().moveInto(DBs), Succeeded());
    ASSERT_THAT_ERROR(DBs->first->createProxy({}, "data").moveInto(ID),
                      Succeeded());
  }

  // Reopen the plugin CAS at CASPath and check the object was stored there.
  std::optional<
      std::pair<std::shared_ptr<ObjectStore>, std::shared_ptr<ActionCache>>>
      DBs;
  ASSERT_THAT_ERROR(createPluginCASDatabases(getCASPluginPath(),
                                             Temp.path("cas"),
                                             /*PluginArgs=*/{})
                        .moveInto(DBs),
                    Succeeded());
  std::optional<ObjectRef> Ref = DBs->first->getReference(*ID);
  ASSERT_TRUE(Ref);
  bool IsMaterialized = false;
  ASSERT_THAT_ERROR(DBs->first->isMaterialized(*Ref).moveInto(IsMaterialized),
                    Succeeded());
  EXPECT_TRUE(IsMaterialized);
}

TEST(PluginCASTest, asyncDownloadAndCancel) {
  unittest::TempDir Temp("plugin-cas", /*Unique=*/true);
  std::string UpDir(Temp.path("up"));
  std::string DownDir(Temp.path("down"));
  std::pair<std::string, std::string> PluginOpts[] = {
      {"upstream-path", std::string(UpDir)}, {"no-logging", "true"}};
  using CASDatabases =
      std::pair<std::shared_ptr<ObjectStore>, std::shared_ptr<ActionCache>>;
  auto OpenDBs = [&]() -> CASDatabases {
    std::optional<CASDatabases> DBs;
    EXPECT_THAT_ERROR(
        createPluginCASDatabases(getCASPluginPath(), DownDir, PluginOpts)
            .moveInto(DBs),
        Succeeded());
    return std::move(*DBs);
  };
  auto GetAsync = [](ActionCache &AC, const CASID &Key,
                     std::unique_ptr<Cancellable> *CancelObj) {
    std::promise<AsyncCASIDValue> Promise;
    auto Future = Promise.get_future();
    AC.getAsync(
        Key, /*CanBeDistributed=*/true,
        [Promise =
             std::move(Promise)](Expected<std::optional<CASID>> Value) mutable {
          Promise.set_value(std::move(Value));
        },
        CancelObj);
    return Future;
  };
  auto LoadAsync = [](ObjectStore &CAS, ObjectRef Ref,
                      std::unique_ptr<Cancellable> *CancelObj) {
    std::promise<AsyncProxyValue> Promise;
    auto Future = Promise.get_future();
    CAS.getProxyAsync(
        Ref,
        [Promise = std::move(Promise)](
            Expected<std::optional<ObjectProxy>> Obj) mutable {
          Promise.set_value(std::move(Obj));
        },
        CancelObj);
    return Future;
  };

  // Populate the "upstream" CAS. A CASID refers to the CAS that created it,
  // so keep the IDs as strings across re-opening the CAS.
  std::string KeyStr, ValueStr;
  {
    auto [CAS, AC] = OpenDBs();
    std::optional<CASID> Key, Value;
    ASSERT_THAT_ERROR(CAS->createProxy({}, "key").moveInto(Key), Succeeded());
    ASSERT_THAT_ERROR(CAS->createProxy({}, "value").moveInto(Value),
                      Succeeded());
    ASSERT_THAT_ERROR(
        AC->putFuture(*Key, *Value, /*CanBeDistributed=*/true).get().take(),
        Succeeded());
    KeyStr = Key->toString();
    ValueStr = Value->toString();
  }

  // Download the result and the object asynchronously from upstream.
  sys::fs::remove_directories(DownDir);
  {
    auto [CAS, AC] = OpenDBs();
    CASID Key = cantFail(CAS->parseID(KeyStr));
    CASID Value = cantFail(CAS->parseID(ValueStr));
    std::optional<CASID> Result;
    ASSERT_THAT_ERROR(GetAsync(*AC, Key, nullptr).get().take().moveInto(Result),
                      Succeeded());
    ASSERT_TRUE(Result);
    EXPECT_EQ(*Result, Value);

    std::optional<ObjectRef> Ref = CAS->getReference(Value);
    ASSERT_TRUE(Ref);
    std::optional<ObjectProxy> Obj;
    ASSERT_THAT_ERROR(LoadAsync(*CAS, *Ref, nullptr).get().take().moveInto(Obj),
                      Succeeded());
    ASSERT_TRUE(Obj);
    EXPECT_EQ(Obj->getData(), "value");
  }

#if LLVM_ENABLE_THREADS
  // Cancelled requests report the result as missing.
  sys::fs::remove_directories(DownDir);
  {
    auto [CAS, AC] = OpenDBs();
    CASID Key = cantFail(CAS->parseID(KeyStr));
    CASID Value = cantFail(CAS->parseID(ValueStr));
    std::unique_ptr<Cancellable> CancelGet;
    auto GetFuture = GetAsync(*AC, Key, &CancelGet);
    ASSERT_TRUE(CancelGet);
    CancelGet->cancel();
    std::optional<CASID> Result;
    ASSERT_THAT_ERROR(GetFuture.get().take().moveInto(Result), Succeeded());
    EXPECT_FALSE(Result);

    std::optional<ObjectRef> Ref = CAS->getReference(Value);
    ASSERT_TRUE(Ref);
    std::unique_ptr<Cancellable> CancelLoad;
    auto LoadFuture = LoadAsync(*CAS, *Ref, &CancelLoad);
    ASSERT_TRUE(CancelLoad);
    CancelLoad->cancel();
    std::optional<ObjectProxy> Obj;
    ASSERT_THAT_ERROR(LoadFuture.get().take().moveInto(Obj), Succeeded());
    EXPECT_FALSE(Obj);
  }
#endif
}

#endif /* !LLVM_HWADDRESS_SANITIZER_BUILD */
