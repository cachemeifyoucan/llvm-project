// Check that a daemon removes its socket when it is terminated, so that a
// new daemon can be started on the same path.
//
// REQUIRES: clang-cc1daemon
// UNSUPPORTED: system-windows

// RUN: rm -rf %t && mkdir -p %t
// RUN: %python %S/Inputs/terminate-depscan-daemon.py %clang \
// RUN:   %{clang-daemon-dir}/%basename_t %t/cas | FileCheck %s

// CHECK: ok
