// RUN: rm -rf %t && mkdir -p %t

// This compiles twice with replay disabled, ensuring that we get the same outputs for the same key.

// Under clang driver

// RUN: env LLVM_CACHE_CAS_PATH=%t/cas CLANG_CACHE_TEST_DETERMINISTIC_OUTPUTS=1 CLANG_CACHE_REDACT_TIME_MACROS=1 \
// RUN: %clang -target x86_64-apple-macos11 -c %s -o %t/t.o -Rcompile-job-cache \
// RUN:   -fdepscan=inline -Xclang -fcas-path -Xclang %t/cas 2> %t/out_driver.txt
// RUN: FileCheck %s --check-prefix=CACHE-SKIPPED --input-file=%t/out_driver.txt

// CACHE-SKIPPED: remark: compile job cache skipped
// CACHE-SKIPPED: remark: compile job cache skipped

/// Check that reproducible caching issues can be made errors.
// RUN: not env CLANG_CACHE_CHECK_REPRODUCIBLE_CACHING_ISSUES=1 \
// RUN: %clang -target x86_64-apple-macos11 -c %s -o %t/t.o -Rcompile-job-cache \
// RUN:   -fdepscan=inline -Xclang -fcas-path -Xclang %t/cas 2> %t/out.txt
// RUN: FileCheck %s --check-prefix=ERROR --input-file=%t/out.txt

// ERROR: error: encountered non-reproducible token, caching will be skipped

// Verify we don't double output without caching enabled.
// RUN: env CLANG_CACHE_TEST_DETERMINISTIC_OUTPUTS=1 %clang -E %s | FileCheck %s -check-prefix=NO_CACHE
// NO_CACHE: getit
// NO_CACHE-NOT: getit

void getit(const char **p1, const char **p2, const char **p3) {
  *p1 = __DATE__;
  *p2 = __TIMESTAMP__;
  *p3 = __TIME__;
}
