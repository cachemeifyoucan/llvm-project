// Include-tree scanning of translation units that use clang modules or module
// map files is diagnosed.

// REQUIRES: ondisk_cas
// RUN: rm -rf %t
// RUN: split-file %s %t
// RUN: sed -e "s|DIR|%/t|g" %t/cdb.json.template > %t/cdb.json
// RUN: not clang-scan-deps -compilation-database %t/cdb.json \
// RUN:   -format experimental-include-tree-full -cas-path %t/cas 2>&1 \
// RUN:   | FileCheck %s

// CHECK: fatal error: CAS-based dependency scan failed: include-tree scanning does not support clang modules

//--- cdb.json.template
[{
  "directory": "DIR",
  "command": "clang -fsyntax-only DIR/tu.c -fmodules -fmodules-cache-path=DIR/cache",
  "file": "DIR/tu.c"
}]

//--- tu.c
