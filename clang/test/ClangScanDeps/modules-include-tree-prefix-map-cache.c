// REQUIRES: ondisk_cas

// RUN: rm -rf %t
// RUN: split-file %s %t/src

// Populate the implicit module cache before enabling prefix mapping.
// RUN: clang-scan-deps -format experimental-include-tree-full -cas-path %t/cas \
// RUN:   -module-files-dir %t/outputs -o %t/no-map.json -- \
// RUN:   %clang -c %t/src/tu.c -o %t/tu.o -I %t/src -fmodules \
// RUN:   -fimplicit-modules -fimplicit-module-maps -fmodules-cache-path=%t/cache

// Reusing that cache must produce the same mapped include tree as a fresh scan.
// RUN: clang-scan-deps -format experimental-include-tree-full -cas-path %t/cas \
// RUN:   -module-files-dir %t/outputs -prefix-map=%t/src=%/root^src -o %t/warm.json -- \
// RUN:   %clang -c %t/src/tu.c -o %t/tu.o -I %t/src -fmodules \
// RUN:   -fimplicit-modules -fimplicit-module-maps -fmodules-cache-path=%t/cache
// RUN: clang-scan-deps -format experimental-include-tree-full -cas-path %t/cas \
// RUN:   -module-files-dir %t/outputs -prefix-map=%t/src=%/root^src -o %t/fresh.json -- \
// RUN:   %clang -c %t/src/tu.c -o %t/tu.o -I %t/src -fmodules \
// RUN:   -fimplicit-modules -fimplicit-module-maps -fmodules-cache-path=%t/fresh-cache

// RUN: %deps-to-rsp %t/warm.json --module-name M > %t/warm.rsp
// RUN: %deps-to-rsp %t/fresh.json --module-name M > %t/fresh.rsp
// RUN: sed -E 's|.*"-fcas-include-tree" "(llvmcas://[[:xdigit:]]+)".*|\1|' %t/warm.rsp > %t/warm.casid
// RUN: sed -E 's|.*"-fcas-include-tree" "(llvmcas://[[:xdigit:]]+)".*|\1|' %t/fresh.rsp > %t/fresh.casid
// RUN: diff %t/warm.casid %t/fresh.casid
// RUN: clang-cas-test -cas %t/cas -print-include-tree @%t/fresh.casid | \
// RUN:   sed 's:\\\\\?:/:g' | FileCheck %s -DMAPPED=%/root^src

// Changing the mapping's value must also invalidate the implicit module cache.
// RUN: clang-scan-deps -format experimental-include-tree-full -cas-path %t/cas \
// RUN:   -module-files-dir %t/outputs -prefix-map=%t/src=%/root^other -o %t/remapped.json -- \
// RUN:   %clang -c %t/src/tu.c -o %t/tu.o -I %t/src -fmodules \
// RUN:   -fimplicit-modules -fimplicit-module-maps -fmodules-cache-path=%t/cache
// RUN: %deps-to-rsp %t/remapped.json --module-name M > %t/remapped.rsp
// RUN: sed -E 's|.*"-fcas-include-tree" "(llvmcas://[[:xdigit:]]+)".*|\1|' %t/remapped.rsp > %t/remapped.casid
// RUN: clang-cas-test -cas %t/cas -print-include-tree @%t/remapped.casid | \
// RUN:   sed 's:\\\\\?:/:g' | FileCheck %s -DMAPPED=%/root^other

// CHECK: [[MAPPED]]{{/|\\}}M.h

//--- module.modulemap
module M { header "M.h" export * }

//--- M.h
#pragma once
struct M { int value; };

//--- tu.c
#include "M.h"
struct M value;
