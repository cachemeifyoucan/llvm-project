// Check the error when an imported module file is referred to by a CAS ID
// that is not in the CAS.

// REQUIRES: ondisk_cas

// RUN: rm -rf %t
// RUN: split-file %s %t
// RUN: sed "s|DIR|%/t|g" %t/cdb.json.template > %t/cdb.json

// RUN: clang-scan-deps -compilation-database %t/cdb.json \
// RUN:   -cas-path %t/cas -module-files-dir %t/outputs \
// RUN:   -format experimental-include-tree-full -mode preprocess-dependency-directives \
// RUN:   > %t/deps.json

// RUN: %deps-to-rsp %t/deps.json --module-name A > %t/A.rsp
// RUN: %deps-to-rsp %t/deps.json --module-name B > %t/B.rsp
// RUN: %clang @%t/A.rsp
// RUN: %clang @%t/B.rsp

// B records its import of A by the CAS ID of A's module file. Load B with
// a different CAS that has B's include-tree but not A's module file.
// RUN: cp %t/outputs/*/B-*.pcm %t/B.pcm
// RUN: cat %t/B.rsp | sed -E 's|.*"-fcas-include-tree" "(llvmcas://[[:xdigit:]]+)".*|\1|' > %t/B.casid
// RUN: llvm-cas --cas %t/cas2 --upstream-cas %t/cas --import @%t/B.casid
// RUN: not %clang_cc1 -fmodules -fno-implicit-modules -fimplicit-module-maps \
// RUN:   -fcas-path %t/cas2 -fmodule-file=B=%t/B.pcm \
// RUN:   -fsyntax-only %t/tu.c 2>&1 \
// RUN:   | FileCheck %s
// CHECK: error: module file '{{.*}}A-{{.*}}.pcm' not found: unloadable casid llvmcas://{{[[:xdigit:]]+}}: does not exist in CAS

//--- cdb.json.template
[{
  "file": "DIR/tu.c",
  "directory": "DIR",
  "command": "clang -fsyntax-only DIR/tu.c -fmodules -fimplicit-module-maps -fmodules-cache-path=DIR/module-cache"
}]

//--- module.modulemap
module A { header "A.h" }
module B { header "B.h" }

//--- A.h
void a(void);

//--- B.h
#include "A.h"
void b(void);

//--- tu.c
#include "B.h"
void tu(void) { a(); b(); }
