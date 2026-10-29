//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/Support/ScopedDurationTimer.h"
#include "gtest/gtest.h"
#include <thread>

using namespace llvm;

TEST(ScopedDurationTimerTest, CallsHandlerOnDestruction) {
  unsigned Calls = 0;
  double Elapsed = -1;
  {
    ScopedDurationTimer Timer([&](double Seconds) {
      ++Calls;
      Elapsed = Seconds;
    });
    EXPECT_EQ(Calls, 0u);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(Calls, 1u);
  EXPECT_GE(Elapsed, 0.010);
}

TEST(ScopedDurationTimerTest, NestedTimers) {
  double Outer = -1, Inner = -1;
  {
    ScopedDurationTimer OuterTimer([&](double Seconds) { Outer = Seconds; });
    {
      ScopedDurationTimer InnerTimer([&](double Seconds) { Inner = Seconds; });
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_GE(Inner, 0.0);
    EXPECT_LT(Outer, 0.0);
  }
  EXPECT_GE(Outer, Inner);
}
