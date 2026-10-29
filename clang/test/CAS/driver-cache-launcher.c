// UNSUPPORTED: system-windows

// RUN: rm -rf %t && mkdir %t
// RUN: echo "#!/bin/sh"              > %t/clang
// RUN: echo "echo run some compiler with opts \$*" >> %t/clang
// RUN: chmod +x %t/clang
// RUN: ln -s %clang %t/clang-symlink-outside-bindir

// 'clang-cache' launcher invokes itself, enables caching.
// RUN: env LLVM_CACHE_CAS_PATH=%t/cas %clang-cache %clang -c %s -o %t.o -### 2>&1 | FileCheck %s -check-prefix=CLANG -DPREFIX=%t
// RUN: env LLVM_CACHE_CAS_PATH=%t/cas %clang-cache %clang++ -c %s -o %t.o -### 2>&1 | FileCheck %s -check-prefix=CLANGPP -DPREFIX=%t
// RUN: env LLVM_CACHE_CAS_PATH=%t/cas %clang-cache %t/clang-symlink-outside-bindir -c %s -o %t.o -### 2>&1 | FileCheck %s -check-prefix=CLANG -DPREFIX=%t
// RUN: env LLVM_CACHE_CAS_PATH=%t/cas PATH="%t:$PATH" %clang-cache clang-symlink-outside-bindir -c %s -o %t.o -### 2>&1 | FileCheck %s -check-prefix=CLANG -DPREFIX=%t

// CLANG: "-cc1depscan" "-fdepscan=auto"
// CLANG: "-fcas-path" "[[PREFIX]]/cas"
// CLANG: "-x" "c"

// CLANGPP: "-cc1depscan" "-fdepscan=auto"
// CLANGPP: "-fcas-path" "[[PREFIX]]/cas"
// CLANGPP: "-x" "c++"

// RUN: env LLVM_CACHE_CAS_PATH=%t/cas %clang-cache %clang -c %s -o %t.o -### 2>&1 | FileCheck %s -check-prefix=INCLUDE-TREE -DPREFIX=%t
// INCLUDE-TREE: "-cc1depscan" "-fdepscan=auto"

// The launcher passes -greproducible, which makes the Darwin link reproducible.
// RUN: env LLVM_CACHE_CAS_PATH=%t/cas %clang-cache %clang -target arm64-apple-macosx12 %s -o %t/exe -### 2>&1 | FileCheck %s -check-prefix=REPRODUCIBLE-LINK
// REPRODUCIBLE-LINK: "-cc1depscan" "-fdepscan=auto"
// REPRODUCIBLE-LINK: env "ZERO_AR_DATE=1" "{{[^"]*}}ld"

// RUN: env LLVM_CACHE_CAS_PATH=%t/cas LLVM_CACHE_PLUGIN_PATH=%t/plugin LLVM_CACHE_PLUGIN_OPTIONS=some-opt=value:opt2=val2 \
// RUN:   %clang-cache %clang -c %s -o %t.o -### 2>&1 | FileCheck %s -check-prefix=PLUGIN -DPREFIX=%t
// PLUGIN: "-fcas-path" "[[PREFIX]]/cas"
// PLUGIN: "-fcas-plugin-path" "[[PREFIX]]/plugin"
// PLUGIN: "-fcas-plugin-option" "some-opt=value"
// PLUGIN: "-fcas-plugin-option" "opt2=val2"

// Using multi-arch invocation.
// RUN: env LLVM_CACHE_CAS_PATH=%t/cas %clang-cache %clang -target x86_64-apple-macos12 -arch x86_64 -arch arm64 -c %s -o %t.o -### 2>&1 | FileCheck %s -check-prefix=MULTIARCH
// MULTIARCH: "-cc1depscan" "-fdepscan=auto"
// MULTIARCH: "-triple" "x86_64-apple-macosx12.0.0"
// MULTIARCH: "-cc1depscan" "-fdepscan=auto"
// MULTIARCH: "-triple" "arm64-apple-macosx12.0.0"

// 'clang-cache' launcher invokes a different clang, does normal non-caching launch.
// RUN: env LLVM_CACHE_CAS_PATH=%t/cas %clang-cache %t/clang -c %s -o %t.o 2>&1 | FileCheck %s -check-prefix=OTHERCLANG -DSRC=%s -DPREFIX=%t
// OTHERCLANG: warning: caching disabled because clang-cache invokes a different clang binary than itself
// OTHERCLANG-NEXT: run some compiler with opts -c [[SRC]] -o [[PREFIX]].o

// RUN: %clang-cache %clang -x objective-c++ -fmodules -fmodules-cache-path=%t/mcp -fno-cxx-modules -c %s -o %t.o -### 2>&1 | FileCheck %s -check-prefix=NONMOD -DPREFIX=%t
// RUN: %clang-cache %clang -x c++ -fmodules -fmodules-cache-path=%t/mcp -fno-cxx-modules -c %s -o %t.o -### 2>&1 | FileCheck %s -check-prefix=NONMOD -DPREFIX=%t
// RUN: %clang-cache %clang -x objective-c -fmodules -fmodules-cache-path=%t/mcp -c %s -o %t.o -### 2>&1 | FileCheck %s -check-prefix=MOD -DPREFIX=%t
// RUN: %clang-cache %clang -x objective-c++ -fmodules -fmodules-cache-path=%t/mcp -fcxx-modules -c %s -o %t.o -### 2>&1 | FileCheck %s -check-prefix=MOD -DPREFIX=%t
// RUN: %clang-cache %clang -x c++ -fmodules -fmodules-cache-path=%t/mcp -fcxx-modules -c %s -o %t.o -### 2>&1 | FileCheck %s -check-prefix=MOD -DPREFIX=%t
// NONMOD: "-cc1depscan"
// MOD: warning: caching disabled because -fmodules is enabled
// MOD-NOT: "-cc1depscan"

// RUN: touch %t/t.s
// RUN: %clang-cache %clang -target arm64-apple-macosx12 -c %t/t.s -o %t.o -### 2>&1 | FileCheck %s -check-prefix=ASM
// ASM: warning: caching disabled because assembler language mode is enabled
// ASM-NOT: "-cc1depscan"

// RUN: env AS_SECURE_LOG_FILE=%t/log %clang-cache %clang -target arm64-apple-macosx12 -c %s -o %t.o -### 2>&1 | FileCheck %s -check-prefix=AS_SECURE_LOG_FILE
// AS_SECURE_LOG_FILE: warning: caching disabled because AS_SECURE_LOG_FILE is set
// AS_SECURE_LOG_FILE-NOT: "-cc1depscan"

// RUN: env LLVM_CACHE_WARNINGS=-Wno-clang-cache %clang-cache %clang -x c++ -fmodules -fmodules-cache-path=%t/mcp -fcxx-modules -c %s -o %t.o -### 2>&1 | FileCheck %s -check-prefix=MOD_HIDE -DPREFIX=%t
// MOD_HIDE-NOT: warning: caching disabled
// MOD_HIDE-NOT: "-cc1depscan"

// RUN: env LLVM_CACHE_WARNINGS=-Werror=clang-cache not %clang-cache %clang -x c++ -fmodules -fmodules-cache-path=%t/mcp -fcxx-modules -c %s -o %t.o -### 2>&1 | FileCheck %s -check-prefix=MOD_ERR -DPREFIX=%t
// MOD_ERR: error: caching disabled because -fmodules is enabled

// RUN: not %clang-cache %t/nonexistent -c %s -o %t.o 2>&1 | FileCheck %s -check-prefix=NONEXISTENT
// NONEXISTENT: error: clang-cache failed to execute compiler

// RUN: not %clang-cache 2>&1 | FileCheck %s -check-prefix=NOCOMMAND
// NOCOMMAND: error: missing compiler command for clang-cache

// Link-only job should not attempt to cache.
// RUN: touch %t.o
// RUN: %clang-cache %clang -target arm64-apple-macosx12 %t.o -o %t -Wl,-ObjC -### 2>&1 | FileCheck %s -check-prefix=STATIC_LINK
// STATIC_LINK-NOT: warning:
// STATIC_LINK-NOT: "-cc1depscan"
// STATIC_LINK: "{{[^"]*}}ld"

// Unused option warning should only be emitted once.
// RUN: touch %t.o
// RUN: env LLVM_CACHE_CAS_PATH=%t/cas %clang-cache %clang -target arm64-apple-macosx12 -fsyntax-only %s -Wl,-ObjC 2>&1 | FileCheck %s -check-prefix=UNUSED_OPT
// UNUSED_OPT-NOT: warning:
// UNUSED_OPT: warning: -Wl,-ObjC: 'linker' input unused
// UNUSED_OPT-NOT: warning:

// -E with caching
// RUN: env LLVM_CACHE_CAS_PATH=%t/cas %clang-cache %clang -target x86_64-apple-darwin10 -E %s -### 2>&1 | FileCheck %s -check-prefix=CLANG_E -DPREFIX=%t
// RUN: env LLVM_CACHE_CAS_PATH=%t/cas %clang-cache %clang -target x86_64-unknown-linux-gnu -E %s -### 2>&1 | FileCheck %s -check-prefix=CLANG_E -DPREFIX=%t
// RUN: env LLVM_CACHE_CAS_PATH=%t/cas %clang-cache %clang -target 86_64-unknown-windows-msvc -E %s -### 2>&1 | FileCheck %s -check-prefix=CLANG_E -DPREFIX=%t

// CLANG_E: "-cc1depscan" "-fdepscan=auto"
// CLANG_E: "-fcas-path" "[[PREFIX]]/cas"
// CLANG_E: "{{.*}}clang{{.*}}" "-cc1" "@{{.*}}.rsp" "-o" "-"
