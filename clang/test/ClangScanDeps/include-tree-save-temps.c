// Test caching when the driver requires multiple jobs. With -save-temps there
// are separate -E, -emit-llvm-bc, -S and -cc1as jobs. The include-tree is only
// used by the -E job; each later job takes the cached output of the previous
// one as its input, referenced by cache key.

// REQUIRES: ondisk_cas

// We use an x86_64-apple-darwin target to avoid host-dependent behaviour in
// the driver. Platforms without an integrated assembler have different commands
// REQUIRES: x86-registered-target

// RUN: rm -rf %t
// RUN: split-file %s %t

// RUN: clang-scan-deps -format experimental-include-tree-full -cas-path %t/cas \
// RUN:   -- %clang -target x86_64-apple-darwin -c %t/src/tu.c -save-temps=obj -o %t/dst0/tu.o -I %t/include \
// RUN:   > %t/deps.0.json

// RUN: cat %t/deps.0.json | sed 's:\\\\\?:/:g' | FileCheck %s -DPREFIX=%/t -DDST=%/t/dst0

// RUN: mkdir %t/dst0
// RUN: %deps-to-rsp %t/deps.0.json --tu-index 0 --tu-cmd-index 0 > %t/tu-cpp.0.rsp
// RUN: %deps-to-rsp %t/deps.0.json --tu-index 0 --tu-cmd-index 1 > %t/tu-emit-ir.0.rsp
// RUN: %deps-to-rsp %t/deps.0.json --tu-index 0 --tu-cmd-index 2 > %t/tu-emit-asm.0.rsp
// RUN: %deps-to-rsp %t/deps.0.json --tu-index 0 --tu-cmd-index 3 > %t/tu-cc1as.0.rsp
// RUN: %clang @%t/tu-cpp.0.rsp      -Rcompile-job-cache 2>&1 | FileCheck %s -check-prefix=CACHE-MISS
// RUN: %clang @%t/tu-emit-ir.0.rsp  -Rcompile-job-cache 2>&1 | FileCheck %s -check-prefix=CACHE-MISS
// RUN: %clang @%t/tu-emit-asm.0.rsp -Rcompile-job-cache 2>&1 | FileCheck %s -check-prefix=CACHE-MISS
// RUN: %clang @%t/tu-cc1as.0.rsp

// A second build with different output paths gets cache hits for all the
// cached jobs.
// RUN: clang-scan-deps -format experimental-include-tree-full -cas-path %t/cas \
// RUN:   -- %clang -target x86_64-apple-darwin -c %t/src/tu.c -save-temps=obj -o %t/dst1/tu.o -I %t/include \
// RUN:   > %t/deps.1.json

// RUN: cat %t/deps.1.json | sed 's:\\\\\?:/:g' | FileCheck %s -DPREFIX=%/t -DDST=%/t/dst1

// RUN: mkdir %t/dst1
// RUN: %deps-to-rsp %t/deps.1.json --tu-index 0 --tu-cmd-index 0 > %t/tu-cpp.1.rsp
// RUN: %deps-to-rsp %t/deps.1.json --tu-index 0 --tu-cmd-index 1 > %t/tu-emit-ir.1.rsp
// RUN: %deps-to-rsp %t/deps.1.json --tu-index 0 --tu-cmd-index 2 > %t/tu-emit-asm.1.rsp
// RUN: %deps-to-rsp %t/deps.1.json --tu-index 0 --tu-cmd-index 3 > %t/tu-cc1as.1.rsp
// RUN: %clang @%t/tu-cpp.1.rsp      -Rcompile-job-cache 2>&1 | FileCheck %s -check-prefix=CACHE-HIT
// RUN: %clang @%t/tu-emit-ir.1.rsp  -Rcompile-job-cache 2>&1 | FileCheck %s -check-prefix=CACHE-HIT
// RUN: %clang @%t/tu-emit-asm.1.rsp -Rcompile-job-cache 2>&1 | FileCheck %s -check-prefix=CACHE-HIT
// RUN: %clang @%t/tu-cc1as.1.rsp

// RUN: diff %t/dst1/tu.i  %t/dst0/tu.i
// RUN: diff %t/dst1/tu.bc %t/dst0/tu.bc
// RUN: diff %t/dst1/tu.s  %t/dst0/tu.s
// RUN: diff %t/dst1/tu.o  %t/dst0/tu.o

// CACHE-HIT: remark: compile job cache hit
// CACHE-MISS: remark: compile job cache miss

// CHECK:      "translation-units": [
// CHECK-NEXT:   {
// CHECK:          "commands": [
// CHECK-NEXT:       {
// CHECK-NEXT:         "cache-key": "[[CPP_CACHE_KEY:llvmcas://[[:xdigit:]]+]]"
// CHECK-NEXT:         "cas-include-tree-id": "[[CPP_INCLUDE_TREE:llvmcas://[[:xdigit:]]+]]"
// CHECK:              "command-line": [
// CHECK-NEXT:           "-cc1"
// CHECK:                "-o"
// CHECK-NEXT:           "[[DST]]/tu.i"
// CHECK:                "-fcas-include-tree"
// CHECK-NEXT:           "[[CPP_INCLUDE_TREE]]"
// CHECK-NOT:            "-fcas-input-file-cache-key"
// CHECK:                "-E"
// CHECK:                "-x"
// CHECK-NEXT:           "c"
// CHECK-NOT:            "{{.*}}/tu.c"
// CHECK:              ]
// CHECK:              "file-deps": [
// CHECK-NEXT:           "[[PREFIX]]/src/tu.c"
// CHECK-NEXT:           "[[PREFIX]]/include/header.h"
// CHECK-NEXT:         ]
// CHECK:              "input-file": "[[PREFIX]]/src/tu.c"
// CHECK-NEXT:       }
// CHECK-NEXT:       {
// CHECK-NEXT:         "cache-key": "[[COMPILER_CACHE_KEY:llvmcas://[[:xdigit:]]+]]"
// FIXME: This should be empty.
// CHECK-NEXT:         "cas-include-tree-id": "{{.*}}"
// CHECK:              "command-line": [
// CHECK-NEXT:           "-cc1"
// CHECK:                "-o"
// CHECK-NEXT:           "[[DST]]/tu.bc"
// CHECK-NOT:            "-fcas-include-tree"
// CHECK:                "-fcas-input-file-cache-key"
// CHECK-NEXT:           "[[CPP_CACHE_KEY]]"
// CHECK:                "-emit-llvm-bc"
// CHECK:                "-x"
// CHECK-NEXT:           "c-cpp-output"
// CHECK-NOT:            "{{.*}}/tu.i"
// CHECK:              ]
// CHECK:              "input-file": "[[PREFIX]]/src/tu.c"
// CHECK-NEXT:       }
// CHECK-NEXT:       {
// CHECK-NEXT:         "cache-key": "[[BACKEND_CACHE_KEY:llvmcas://[[:xdigit:]]+]]"
// FIXME: This should be empty.
// CHECK-NEXT:         "cas-include-tree-id": "{{.*}}"
// CHECK:              "command-line": [
// CHECK-NEXT:           "-cc1"
// CHECK:                "-o"
// CHECK-NEXT:           "[[DST]]/tu.s"
// CHECK:                "-fcas-input-file-cache-key"
// CHECK-NEXT:           "[[COMPILER_CACHE_KEY]]"
// CHECK:                "-S"
// CHECK:                "-x"
// CHECK-NEXT:           "ir"
// CHECK:              ]
// CHECK:              "input-file": "[[PREFIX]]/src/tu.c"
// CHECK-NEXT:       }
// CHECK-NEXT:       {
// FIXME: This should be empty.
// CHECK-NEXT:         "cas-include-tree-id": "{{.*}}"
// CHECK:              "command-line": [
// CHECK-NEXT:           "-cc1as"
// CHECK:                "-o"
// CHECK-NEXT:           "[[DST]]/tu.o"
// FIXME: The integrated assembler should support caching too.
// CHECK:                "[[DST]]/tu.s"
// CHECK:              ]
// CHECK:              "input-file": "[[PREFIX]]/src/tu.c"
// CHECK-NEXT:       }
// CHECK-NEXT:     ]
// CHECK-NEXT:   }
// CHECK-NEXT: ]

//--- src/tu.c
#include "header.h"

int tu(void) {
  return header();
}

//--- include/header.h
static inline int header(void) {
  return 0;
}
