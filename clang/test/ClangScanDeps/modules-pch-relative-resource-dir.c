// Check that scanning with a PCH that imports explicitly built modules
// accepts modules that record the same relative resource directory as the
// translation unit while there are stable directories.

// RUN: rm -rf %t
// RUN: split-file %s %t
// RUN: sed "s|DIR|%/t|g" %t/cdb_pch.json.template > %t/cdb_pch.json
// RUN: sed "s|DIR|%/t|g" %t/cdb.json.template > %t/cdb.json

// RUN: clang-scan-deps -compilation-database %t/cdb_pch.json \
// RUN:   -format experimental-full -module-files-dir %t/outputs \
// RUN:   > %t/deps_pch.json
// RUN: %deps-to-rsp %t/deps_pch.json --module-name A > %t/A.rsp
// RUN: %deps-to-rsp %t/deps_pch.json --tu-index 0 > %t/pch.rsp
// RUN: %clang @%t/A.rsp
// RUN: %clang @%t/pch.rsp

// RUN: clang-scan-deps -compilation-database %t/cdb.json \
// RUN:   -format experimental-full -module-files-dir %t/outputs \
// RUN:   > %t/deps.json
// RUN: cat %t/deps.json | sed 's:\\\\\?:/:g' | FileCheck %s -DPREFIX=%/t

// CHECK:      "translation-units": [
// CHECK:        "input-file": "[[PREFIX]]/tu.c"

//--- cdb_pch.json.template
[{
  "directory": "DIR",
  "command": "clang -x c-header DIR/prefix.h -o DIR/prefix.h.pch -isysroot DIR/sysroot -resource-dir lib/clang/fake -I DIR -fmodules -fimplicit-modules -fimplicit-module-maps -fmodules-cache-path=DIR/module-cache",
  "file": "DIR/prefix.h"
}]

//--- cdb.json.template
[{
  "directory": "DIR",
  "command": "clang -fsyntax-only DIR/tu.c -include DIR/prefix.h -isysroot DIR/sysroot -resource-dir lib/clang/fake -I DIR -fmodules -fimplicit-modules -fimplicit-module-maps -fmodules-cache-path=DIR/module-cache",
  "file": "DIR/tu.c"
}]

//--- sysroot/.keep

//--- module.modulemap
module A { header "A.h" }

//--- A.h
int a;

//--- prefix.h
#include "A.h"

//--- tu.c
int b = a;
