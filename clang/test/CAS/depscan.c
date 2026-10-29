// RUN: rm -rf %t && mkdir -p %t
// RUN: %clang -cc1depscan -fdepscan=inline -cc1-args -cc1 -triple x86_64-apple-macos11.0 -x c %s -o %s.o -MT %s.o -dependency-file %t.d -fcas-path %t/cas 2>&1 | sed 's:\\\\\?:/:g' | FileCheck %s -DPREFIX=%/t
// RUN: %clang -cc1depscan -fdepscan=inline -cc1-args -triple x86_64-apple-macos11.0 -x c %s -o %s.o -MT %s.o -dependency-file %t.d -fcas-path %t/cas 2>&1 | sed 's:\\\\\?:/:g' | FileCheck %s -DPREFIX=%/t

// CHECK: {{^}}"-cc1"
// CHECK: "-fcas-path" "[[PREFIX]]/cas"
// CHECK-NOT: dependency-file
// CHECK-NOT: [[PREFIX]].d

int test() { return 0; }
