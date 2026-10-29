// Test running -fdepscan.

// RUN: rm -rf %t-*.d %t.cas
// RUN: %clang -target x86_64-apple-macos11 -I %S/Inputs -fdepscan=inline \
// RUN:   -E -MD -MF %t-inline.d -x c %s -Xclang -fcas-path -Xclang %t.cas >/dev/null
// RUN: %clang -target x86_64-apple-macos11 -I %S/Inputs -fdepscan=auto \
// RUN:   -E -MD -MF %t-auto.d -x c %s -Xclang -fcas-path -Xclang %t.cas >/dev/null
// RUN: %clang -target x86_64-apple-macos11 -I %S/Inputs -fdepscan=off \
// RUN:   -E -MD -MF %t-off.d -x c %s -Xclang -fcas-path -Xclang %t.cas >/dev/null
//
// Check that the dependency files match.
// RUN: diff %t-off.d %t-inline.d
// RUN: diff %t-off.d %t-auto.d

#include "test.h"

int func(void);
