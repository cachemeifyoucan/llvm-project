// Check the error when -cc1depscan cannot write its output.

// RUN: rm -rf %t && mkdir -p %t && touch %t/file
// RUN: not %clang -cc1depscan -fdepscan=inline -o %t/file/out.rsp \
// RUN:   -cc1-args -cc1 -triple x86_64-apple-macos11 -fsyntax-only %s \
// RUN:   -fcas-path %t/cas 2>&1 | FileCheck %s

// CHECK: error: unable to open output file '{{.*}}out.rsp'

int x;
