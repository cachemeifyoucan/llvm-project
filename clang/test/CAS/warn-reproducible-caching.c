// RUN: rm -rf %t && mkdir -p %t
// RUN: split-file %s %t

// No warning in normal invocation.
// RUN: %clang_cc1 -triple x86_64-apple-macos11 -fsyntax-only %t/t.c -isystem %t/sys 2>&1 | FileCheck %s --check-prefix=NOWARN --allow-empty
// NOWARN-NOT: warning

// Warning in cached invocation even if coming from system header. -Werror does not affect it.
// RUN: %clang -cc1depscan -fdepscan=inline -o %t/t.rsp -cc1-args \
// RUN:   -cc1 -triple x86_64-apple-macos11 -emit-obj %t/t.c -o %t/t.o \
// RUN:   -isystem %t/sys -fcas-path %t/cas -Werror
// RUN: %clang @%t/t.rsp 2>&1 | FileCheck %s --check-prefix=WARN
// WARN: warning: encountered non-reproducible token

// The result is not cached.
// RUN: %clang @%t/t.rsp -Rcompile-job-cache 2>&1 | FileCheck %s --check-prefix=SKIPPED
// SKIPPED: remark: compile job cache miss
// SKIPPED: warning: encountered non-reproducible token
// SKIPPED: remark: compile job cache skipped

// Error if explicitly turned into error.
// RUN: %clang -cc1depscan -fdepscan=inline -o %t/t-error.rsp -cc1-args \
// RUN:   -cc1 -triple x86_64-apple-macos11 -emit-obj %t/t.c -o %t/t.o \
// RUN:   -isystem %t/sys -fcas-path %t/cas -Werror=reproducible-caching
// RUN: not %clang @%t/t-error.rsp 2>&1 | FileCheck %s --check-prefix=ERROR
// ERROR: error: encountered non-reproducible token

// Always an error when generating a PCH.
// RUN: %clang -cc1depscan -fdepscan=inline -o %t/pch.rsp -cc1-args \
// RUN:   -cc1 -triple x86_64-apple-macos11 -x c-header %t/sys/sys.h \
// RUN:   -emit-pch -o %t/sys.pch -fcas-path %t/cas
// RUN: not %clang @%t/pch.rsp 2>&1 | FileCheck %s --check-prefix=PCH-ERROR
// PCH-ERROR: error: encountered non-reproducible token, caching failed

//--- t.c
#include <sys.h>

//--- sys/sys.h
const char *foo() {
  return __DATE__;
}
