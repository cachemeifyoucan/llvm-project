// REQUIRES: ondisk_cas

// RUN: rm -rf %t
// RUN: split-file %s %t/dir
// RUN: sed -e "s|DIR|%/t/dir|g" -e "s|CLANG|%clang|g" %t/dir/cdb1.json.template > %t/cdb1.json
// RUN: sed -e "s|DIR|%/t/dir|g" -e "s|CLANG|%clang|g" %t/dir/cdb2.json.template > %t/cdb2.json

// RUN: mkdir -p %t/dir/sdk
// RUN: echo "{}" > %t/dir/sdk/SDKSettings.json

/// Scan some of the modules with current SDKSettings.json.
// RUN: clang-scan-deps -compilation-database %t/cdb2.json \
// RUN:   -cas-path %t/cas \
// RUN:   -format experimental-include-tree-full -mode preprocess-dependency-directives

/// Modify SDKSettings.json and scan the rest of the modules.
// RUN: echo "" >> %t/dir/sdk/SDKSettings.json

// RUN: not clang-scan-deps -compilation-database %t/cdb2.json \
// RUN:   -cas-path %t/cas \
// RUN:   -format experimental-include-tree-full -mode preprocess-dependency-directives \
// RUN:   2>&1 | FileCheck %s
// CHECK: fatal error:
// CHECK-SAME: is changed during the build but its dependencies are not rebuilt
// CHECK-NEXT: note: when building
// CHECK-NEXT: note: when building


//--- cdb1.json.template
[{
  "file": "DIR/tu1.m",
  "directory": "DIR",
  "command": "CLANG -target x86_64-apple-darwin10 -fsyntax-only DIR/tu1.m -I DIR -isysroot DIR/sdk -fmodules -fimplicit-modules -fimplicit-module-maps -fmodules-cache-path=DIR/module-cache"
}]

//--- cdb2.json.template
[{
  "file": "DIR/tu2.m",
  "directory": "DIR",
  "command": "CLANG -target x86_64-apple-darwin10 -fsyntax-only DIR/tu2.m -I DIR -isysroot DIR/sdk -fmodules -fimplicit-modules -fimplicit-module-maps -fmodules-cache-path=DIR/module-cache"
}]

//--- module.modulemap
module Top { header "Top.h" export *}
module Left { header "Left.h" export *}
module Right { header "Right.h" export *}

//--- Top.h
#pragma once
struct Top {
  int x;
};
void top(void);

//--- Left.h
#include "Top.h"

//--- Right.h
#include "Top.h"
void right(void);

//--- tu1.m
#import "Left.h"

//--- tu2.m
#import "Left.h"
#import "Right.h"

void tu(void) {
  top();
  left();
  right();
}
