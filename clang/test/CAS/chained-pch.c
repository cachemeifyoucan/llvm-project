// Check that CAS caching works when a parent PCH is created, then a child
// PCH is created that includes the parent PCH, then a source file includes
// the child PCH.

// RUN: rm -rf %t
// RUN: split-file %s %t
// RUN: mkdir %t/cas

// RUN: %clang -cc1depscan -fdepscan=inline -o %t/parent.rsp -cc1-args \
// RUN:   -cc1 -triple x86_64-apple-macos11 -x c-header %t/parent.h \
// RUN:   -emit-pch -o %t/parent.pch -Rcompile-job-cache -fcas-path %t/cas
// RUN: %clang @%t/parent.rsp 2>&1 | FileCheck %s --check-prefix=MISS

// RUN: %clang -cc1depscan -fdepscan=inline -o %t/child.rsp -cc1-args \
// RUN:   -cc1 -triple x86_64-apple-macos11 -x c-header %t/child.h \
// RUN:   -include-pch %t/parent.pch -emit-pch -o %t/child.pch \
// RUN:   -Rcompile-job-cache -fcas-path %t/cas
// RUN: %clang @%t/child.rsp 2>&1 | FileCheck %s --check-prefix=MISS

// RUN: %clang -cc1depscan -fdepscan=inline -o %t/tu.rsp -cc1-args \
// RUN:   -cc1 -triple x86_64-apple-macos11 -emit-obj %t/t.c \
// RUN:   -include-pch %t/child.pch -o %t/t.o \
// RUN:   -Rcompile-job-cache -fcas-path %t/cas
// RUN: %clang @%t/tu.rsp 2>&1 | FileCheck %s --check-prefix=MISS
// RUN: ls %t/t.o && rm %t/t.o

// Redo the whole scenario from scratch. Since the CAS already has all the
// inputs cached, everything should be a cache hit this time.
// RUN: rm %t/parent.pch %t/child.pch

// RUN: %clang -cc1depscan -fdepscan=inline -o %t/parent.rsp -cc1-args \
// RUN:   -cc1 -triple x86_64-apple-macos11 -x c-header %t/parent.h \
// RUN:   -emit-pch -o %t/parent.pch -Rcompile-job-cache -fcas-path %t/cas
// RUN: %clang @%t/parent.rsp 2>&1 | FileCheck %s --check-prefix=HIT

// RUN: %clang -cc1depscan -fdepscan=inline -o %t/child.rsp -cc1-args \
// RUN:   -cc1 -triple x86_64-apple-macos11 -x c-header %t/child.h \
// RUN:   -include-pch %t/parent.pch -emit-pch -o %t/child.pch \
// RUN:   -Rcompile-job-cache -fcas-path %t/cas
// RUN: %clang @%t/child.rsp 2>&1 | FileCheck %s --check-prefix=HIT

// RUN: %clang -cc1depscan -fdepscan=inline -o %t/tu.rsp -cc1-args \
// RUN:   -cc1 -triple x86_64-apple-macos11 -emit-obj %t/t.c \
// RUN:   -include-pch %t/child.pch -o %t/t.o \
// RUN:   -Rcompile-job-cache -fcas-path %t/cas
// RUN: %clang @%t/tu.rsp 2>&1 | FileCheck %s --check-prefix=HIT
// RUN: ls %t/t.o

// MISS: remark: compile job cache miss
// MISS-NOT: error:
// HIT: remark: compile job cache hit
// HIT-NOT: error:

//--- parent.h
#define PARENT_MACRO 1
struct Parent {
  int x;
};

//--- child.h
#define CHILD_MACRO 2
struct Child {
  int y;
};

//--- t.c
int test(struct Parent *p, struct Child *c) {
  return p->x + c->y + PARENT_MACRO + CHILD_MACRO;
}
