// Check that -fdepscan scans the header when clang-cl builds a PCH (/Yc).

// RUN: rm -rf %t
// RUN: split-file %s %t

// RUN: %clang_cl /clang:-fdepscan=inline -Xclang -fcas-path -Xclang %t/cas /Yc%t/cmake_pch.hh /Fp%t/cmake_pch.pch /FI%t/cmake_pch.hh /Fo%t/cmake_pch.cc.obj /Fd%t -Xclang -Rcompile-job-cache -c -- %t/cmake_pch.cc 2>&1 | FileCheck %s -check-prefix=CACHE-MISS
// RUN: %clang_cl /clang:-fdepscan=inline -Xclang -fcas-path -Xclang %t/cas /Yc%t/cmake_pch.hh /Fp%t/cmake_pch.pch /FI%t/cmake_pch.hh /Fo%t/cmake_pch.cc.obj /Fd%t -Xclang -Rcompile-job-cache -c -- %t/cmake_pch.cc 2>&1 | FileCheck %s -check-prefix=CACHE-HIT
// RUN: %clang_cl /clang:-fdepscan=inline -Xclang -fcas-path -Xclang %t/cas /Yu%t/cmake_pch.hh /Fp%t/cmake_pch.pch /FI%t/cmake_pch.hh /Fo%t/test.cc.obj /Fd%t -Xclang -Rcompile-job-cache -c -- %t/test.cc 2>&1 | FileCheck %s -check-prefix=CACHE-MISS

// CACHE-HIT: remark: compile job cache hit
// CACHE-MISS: remark: compile job cache miss

//--- test.cc
#include "header.h"

//--- header.h
#pragma once

//--- cmake_pch.cc
extern int _;

//--- cmake_pch.hh
#pragma clang system_header
#include "header.h"
