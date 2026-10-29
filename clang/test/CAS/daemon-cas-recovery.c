// REQUIRES: clang-cc1daemon

// RUN: rm -rf %t && mkdir -p %t

/// Construct a malformed CAS to recovery from.
// RUN: echo "abc" | llvm-cas --cas %t/cas --make-blob --data -
// RUN: rm %t/cas/v1.1/data.v1
// RUN: not llvm-cas --cas %t/cas --validate --check-hash

// RUN: env LLVM_CAS_FORCE_VALIDATION=1 %clang -target x86_64-apple-macos11 \
// RUN:   -fdepscan=daemon -Xclang -fcas-path -Xclang %t/cas \
// RUN:   -fsyntax-only -x c %s
// RUN: llvm-cas --cas %t/cas --validate --check-hash

int func(void);
