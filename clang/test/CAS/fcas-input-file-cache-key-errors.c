// Check errors for -fcas-input-file-cache-key.

// RUN: rm -rf %t && mkdir -p %t

// RUN: not %clang_cc1 -triple x86_64-apple-macos11 -fcas-path %t/cas \
// RUN:   -fcache-compile-job -fcas-input-file-cache-key not-a-key \
// RUN:   -x c-cpp-output -emit-obj -o %t/t.o 2>&1 \
// RUN:   | FileCheck %s --check-prefix=INVALID
// INVALID: fatal error: CAS cannot parse root-id 'not-a-key'

// A valid CAS ID that is not a cache key in the action cache.
// RUN: echo "data" > %t/blob
// RUN: llvm-cas --cas %t/cas --make-blob --data %t/blob > %t/blob.casid
// RUN: not %clang_cc1 -triple x86_64-apple-macos11 -fcas-path %t/cas \
// RUN:   -fcache-compile-job -fcas-input-file-cache-key @%t/blob.casid \
// RUN:   -x c-cpp-output -emit-obj -o %t/t.o 2>&1 \
// RUN:   | FileCheck %s --check-prefix=MISSING
// MISSING: fatal error: CAS missing expected input cache key

// RUN: not %clang_cc1 -triple x86_64-apple-macos11 -fcas-path %t/cas \
// RUN:   -fcache-compile-job -fcas-input-file-cache-key @%t/blob.casid \
// RUN:   -x c-cpp-output -emit-obj -o %t/t.o %t/blob 2>&1 \
// RUN:   | FileCheck %s --check-prefix=INPUTS
// INPUTS: error: passing input files is incompatible with '-fcas-include-tree' and '-fcas-input-file-cache-key'

// The cache key resolves to something that is not a compile job result.
// RUN: echo "key" > %t/key
// RUN: llvm-cas --cas %t/cas --make-blob --data %t/key > %t/key.casid
// RUN: llvm-cas --cas %t/cas --put-cache-key @%t/key.casid @%t/blob.casid
// RUN: not %clang_cc1 -triple x86_64-apple-macos11 -fcas-path %t/cas \
// RUN:   -fcache-compile-job -fcas-input-file-cache-key @%t/key.casid \
// RUN:   -x c-cpp-output -emit-obj -o %t/t.o 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NOT-RESULT
// NOT-RESULT: error: unable to load the CAS input 'llvmcas://{{[[:xdigit:]]+}}': 'not a compile job result'
