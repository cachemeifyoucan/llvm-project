// Check that `clang-cache -depscan-server` starts a depscan daemon that
// clang-cache can be directed to with CLANG_CACHE_SCAN_DAEMON_SOCKET_PATH.
//
// REQUIRES: clang-cc1daemon
// UNSUPPORTED: system-windows

// RUN: rm -rf %t && mkdir -p %t
// RUN: %python %S/Inputs/clang-cache-depscan-server.py %clang-cache %clang \
// RUN:   %{clang-daemon-dir}/%basename_t %t/cas %s | FileCheck %s

// CHECK: remark: compile job cache miss
// CHECK: ok

// RUN: not %clang-cache -depscan-server 2>&1 \
// RUN:   | FileCheck %s -check-prefix=MISSING
// MISSING: error: missing arguments for dep-scanning server mode

int x;
