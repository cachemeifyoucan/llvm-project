// Check that daemons are shared based on the names of ancestor processes. The
// scans run under bash, with `not not` adding two ancestors named "not".
//
// REQUIRES: clang-cc1daemon
// UNSUPPORTED: system-windows

// DEFINE: %{scan} = %clang -cc1depscan -fdepscan=daemon -o %t/out.rsp
// DEFINE: %{cc1} = -cc1-args -cc1 -triple x86_64-apple-macos11 -fsyntax-only %s
// DEFINE: %{daemons} = cc1depscand -run .* -cas-args -fcas-path

// Stop daemons left behind by a previous run of this test.
// RUN: pkill -f "%{daemons} %t/" || true
// RUN: rm -rf %t && mkdir -p %t

// Scans whose parent has the given name share a daemon.
// RUN: bash -c '%{scan} -fdepscan-share-parent=bash %{cc1} -fcas-path %t/parent; \
// RUN:          %{scan} -fdepscan-share-parent=bash %{cc1} -fcas-path %t/parent; true'
// RUN: pgrep -f "%{daemons} %t/parent$" | count 1

// Only the parent is checked for -fdepscan-share-parent=, so each scan gets
// its own daemon.
// RUN: bash -c 'not not %{scan} -fdepscan-share-parent=bash %{cc1} -fcas-path %t/not-parent; \
// RUN:          not not %{scan} -fdepscan-share-parent=bash %{cc1} -fcas-path %t/not-parent; true'
// RUN: pgrep -f "%{daemons} %t/not-parent$" | count 2

// Scans with an ancestor that has the given name share a daemon.
// RUN: bash -c 'not not %{scan} -fdepscan-share=bash %{cc1} -fcas-path %t/ancestor; \
// RUN:          not not %{scan} -fdepscan-share=bash %{cc1} -fcas-path %t/ancestor; true'
// RUN: pgrep -f "%{daemons} %t/ancestor$" | count 1

// The search stops at a process named by -fdepscan-share-stop=, so each scan
// gets its own daemon.
// RUN: bash -c 'not not %{scan} -fdepscan-share=bash -fdepscan-share-stop=not %{cc1} -fcas-path %t/stop; \
// RUN:          not not %{scan} -fdepscan-share=bash -fdepscan-share-stop=not %{cc1} -fcas-path %t/stop; true'
// RUN: pgrep -f "%{daemons} %t/stop$" | count 2

// RUN: pkill -f "%{daemons} %t/"

int x;
