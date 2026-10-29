// RUN: rm -rf %t
// RUN: split-file %s %t

// RUN: %clang -cc1depscan -o %t/inline.rsp -fdepscan=inline -cc1-args -cc1 -triple x86_64-apple-macos11.0 \
// RUN:     -emit-obj %t/t.c -o %t/t.o -dwarf-ext-refs -fmodule-format=obj  \
// RUN:     -coverage-notes-file=%t/t.gcno -coverage-data-file=%t/t.gcda \
// RUN:     -I %t/includes -isysroot %S/Inputs/SDK -fcas-path %t/cas -DSOME_MACRO -dependency-file %t/inline.d -MT deps

// RUN: cat %t/inline.rsp | sed 's:\\\\\?:/:g' | FileCheck %s -DPREFIX=%/t
// RUN: cat %t/inline.rsp | sed 's:\\\\\?:/:g' | FileCheck %s -DPREFIX=%/t -check-prefix=SHOULD

// RUN: %clang @%t/inline.rsp

// CHECK: "-fcas-path" "[[PREFIX]]/cas"
// CHECK: "-fcas-include-tree"
// CHECK: "-isysroot"
// CHECK: "-coverage-data-file=[[PREFIX]]/t.gcda"
// CHECK: "-coverage-notes-file=[[PREFIX]]/t.gcno"
// SHOULD-NOT: "-I"
// SHOULD-NOT: "[[PREFIX]]/t.c"
// SHOULD-NOT: "-D"

// RUN: cat %t/inline.d | sed 's:\\\\\?:/:g' | FileCheck %s -DPREFIX=%/t -check-prefix=DEPS

// DEPS: deps:
// DEPS: [[PREFIX]]/t.c
// DEPS: [[PREFIX]]/includes{{/|\\}}t.h

//--- t.c
#include "t.h"

int test(struct S *s) {
  return s->x;
}

//--- includes/t.h
struct S {
  int x;
};
