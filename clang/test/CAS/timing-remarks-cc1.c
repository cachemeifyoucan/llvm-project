// Check -Rcompile-job-cache-timing at the -cc1 level. The same remarks are
// tested through clang-cache in timing-remarks.c.

// RUN: rm -rf %t && mkdir -p %t

// RUN: %clang -cc1depscan -fdepscan=inline -o %t/t.rsp -cc1-args \
// RUN:   -cc1 -triple x86_64-apple-macos11 -emit-obj %s -o %t/t.o \
// RUN:   -fcas-path %t/cas -Rcompile-job-cache -Rcompile-job-cache-timing \
// RUN:   2>&1 | FileCheck %s --check-prefix=DEPSCAN
// RUN: %clang @%t/t.rsp 2>&1 | FileCheck %s --check-prefix=CACHE-MISS
// RUN: %clang @%t/t.rsp 2>&1 | FileCheck %s --check-prefix=CACHE-HIT

// DEPSCAN: remark: compile job dependency scanning time:

// CACHE-MISS: remark: compile job cache backend key query time:
// CACHE-MISS: remark: compile job cache miss
// CACHE-MISS: remark: compile job cache backend store artifacts time:
// CACHE-MISS: remark: compile job cache backend key update time:

// CACHE-HIT: remark: compile job cache backend key query time:
// CACHE-HIT: remark: compile job cache hit
// CACHE-HIT: remark: compile job cache backend load artifacts time:

int x;
