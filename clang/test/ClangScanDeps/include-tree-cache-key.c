// Check that include-tree scanning reports a compile job cache key for each
// translation unit command, that the key does not depend on the output path,
// and that it can be printed with clang-cas-test.

// REQUIRES: ondisk_cas
// RUN: rm -rf %t
// RUN: split-file %s %t
// RUN: sed -e "s|DIR|%/t|g" %t/cdb.json.template > %t/cdb.json

// RUN: clang-scan-deps -compilation-database %t/cdb.json -j 1 \
// RUN:   -format experimental-include-tree-full -cas-path %t/cas > %t/deps.json
// RUN: sed -n -e 's/.*"cache-key": "\(.*\)".*/\1/p' %t/deps.json > %t/keys.txt
// RUN: FileCheck %s -input-file %t/keys.txt -check-prefix=KEYS

// The first two commands only differ in their output path; the third one uses
// a different optimization level.
// KEYS:      [[KEY:llvmcas://[[:xdigit:]]+]]
// KEYS-NEXT: [[KEY]]
// KEYS-NEXT: llvmcas://
// RUN: sort -u %t/keys.txt | count 2

// RUN: head -n 1 %t/keys.txt > %t/key.rsp
// RUN: clang-cas-test -print-compile-job-cache-key -cas %t/cas @%t/key.rsp \
// RUN:   | FileCheck %s -DPREFIX=%/t

// CHECK:      command-line: llvmcas://
// CHECK-NOT:    a.o
// CHECK:        -o \
// CHECK-NEXT:   - \
// CHECK-NOT:    a.o
// CHECK:        -fcas-include-tree llvmcas://
// CHECK-NOT:    a.o
// CHECK:      computation: llvmcas://
// CHECK-NEXT:   -cc1
// CHECK:      include-tree: llvmcas://
// CHECK-NEXT: [[PREFIX]]{{[/\\]}}t.c llvmcas://
// CHECK:      Files:
// CHECK-NEXT: [[PREFIX]]{{[/\\]}}t.c llvmcas://
// CHECK:      version: llvmcas://
// CHECK-NEXT:   {{.*}}clang version

// RUN: not clang-cas-test -print-compile-job-cache-key -cas %t/cas \
// RUN:   llvmcas://0000000000000000000000000000000000000000000000000000000000000000 \
// RUN:   2>&1 | FileCheck %s -check-prefix=MISSING
// MISSING: clang-cas-test:

//--- cdb.json.template
[
  {
    "directory": "DIR",
    "command": "clang -target x86_64-apple-macos12 -c DIR/t.c -o DIR/a.o",
    "file": "DIR/t.c"
  },
  {
    "directory": "DIR",
    "command": "clang -target x86_64-apple-macos12 -c DIR/t.c -o DIR/b.o",
    "file": "DIR/t.c"
  },
  {
    "directory": "DIR",
    "command": "clang -target x86_64-apple-macos12 -c DIR/t.c -o DIR/c.o -O2",
    "file": "DIR/t.c"
  }
]

//--- t.c
int x = 1;
