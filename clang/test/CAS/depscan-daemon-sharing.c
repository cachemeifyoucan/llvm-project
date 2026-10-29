// Check which -cc1depscan invocations share a daemon by counting the daemons
// that are running for each CAS.
//
// REQUIRES: clang-cc1daemon
// UNSUPPORTED: system-windows

// DEFINE: %{scan} = %clang -cc1depscan -fdepscan=daemon -o %t/out.rsp
// DEFINE: %{cc1} = -cc1-args -cc1 -triple x86_64-apple-macos11 -fsyntax-only %s
// DEFINE: %{daemons} = cc1depscand -run .* -cas-args -fcas-path

// Stop daemons left behind by a previous run of this test.
// RUN: pkill -f "%{daemons} %t/" || true
// RUN: rm -rf %t && mkdir -p %t

// Scans with the same identifier share a daemon.
// RUN: %{scan} -fdepscan-share-identifier=%basename_t-a %{cc1} -fcas-path %t/id
// RUN: %{scan} -fdepscan-share-identifier=%basename_t-a %{cc1} -fcas-path %t/id
// RUN: pgrep -f "%{daemons} %t/id$" | count 1
// RUN: %{scan} -fdepscan-share-identifier=%basename_t-b %{cc1} -fcas-path %t/id
// RUN: pgrep -f "%{daemons} %t/id$" | count 2

// Scans with the same parent process share a daemon.
// RUN: bash -c '%{scan} -fdepscan-share-parent %{cc1} -fcas-path %t/parent; \
// RUN:          %{scan} -fdepscan-share-parent %{cc1} -fcas-path %t/parent; true'
// RUN: pgrep -f "%{daemons} %t/parent$" | count 1

// Scans that don't share get their own daemon.
// RUN: bash -c '%{scan} %{cc1} -fcas-path %t/none; \
// RUN:          %{scan} %{cc1} -fcas-path %t/none; true'
// RUN: pgrep -f "%{daemons} %t/none$" | count 2

// RUN: pkill -f "%{daemons} %t/"

int x;
