// Check that the module cache keys recorded in a module file are resolved with
// a CAS configured with -fcas-path, even if the compilation does not use the
// CAS otherwise, and that -fmodule-load-ignore-cas makes them ignored.

// REQUIRES: ondisk_cas

// RUN: rm -rf %t
// RUN: split-file %s %t

// RUN: sed -e "s|DIR|%/t|g" %t/cdb.json.template > %t/cdb.json
// RUN: clang-scan-deps -compilation-database %t/cdb.json \
// RUN:   -format experimental-include-tree-full -cas-path %t/cas > %t/deps.json

// RUN: %deps-to-rsp %t/deps.json --module-name Inner > %t/Inner.rsp
// RUN: %deps-to-rsp %t/deps.json --module-name Outer > %t/Outer.rsp
// RUN: %clang @%t/Inner.rsp
// RUN: %clang @%t/Outer.rsp -o %t/Outer.pcm

// Outer refers to Inner by the file name of Inner's module file, so Inner
// can only be loaded out of the CAS.
// RUN: %clang -fsyntax-only -fmodules -fimplicit-module-maps \
// RUN:   -fmodules-cache-path=%t/mcp -I %t -fmodule-file=Outer=%t/Outer.pcm \
// RUN:   -Xclang -fcas-path -Xclang %t/cas %t/tu.c
// RUN: not %clang -fsyntax-only -fmodules -fimplicit-module-maps \
// RUN:   -fmodules-cache-path=%t/mcp -I %t -fmodule-file=Outer=%t/Outer.pcm \
// RUN:   -Xclang -fcas-path -Xclang %t/cas -Xclang -fmodule-load-ignore-cas %t/tu.c 2>&1 \
// RUN:   | FileCheck %s --check-prefix=MISSING
// RUN: not %clang -fsyntax-only -fmodules -fimplicit-module-maps \
// RUN:   -fmodules-cache-path=%t/mcp -I %t -fmodule-file=Outer=%t/Outer.pcm \
// RUN:   %t/tu.c 2>&1 | FileCheck %s --check-prefix=MISSING

// MISSING: fatal error: module file '{{.*}}Inner-{{.*}}.pcm' not found

//--- cdb.json.template
[
  {
    "directory": "DIR",
    "command": "clang -fsyntax-only -fmodules DIR/tu.c -I DIR -fmodules-cache-path=DIR/module-cache",
    "file": "DIR/tu.c"
  }
]

//--- module.modulemap
module Inner { header "inner.h" }
module Outer { header "outer.h" export * }

//--- inner.h
extern int inner_var;
struct InnerType { int x; };

//--- outer.h
#include "inner.h"
extern int outer_var;

//--- tu.c
#include "outer.h"
int use(struct InnerType *p) { return inner_var + outer_var + p->x; }
