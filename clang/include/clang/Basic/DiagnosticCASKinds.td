//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// CAS diagnostics.
///
//===----------------------------------------------------------------------===//

let Component = "CAS" in {

def err_cas_cannot_be_initialized : Error<
  "CAS cannot be initialized from the specified '-fcas-*' options: %0">,
  DefaultFatal;
def err_cas_cannot_parse_root_id
    : Error<"CAS cannot parse root-id '%0' specified by '-fcas-*' options">,
      DefaultFatal;
def err_cas_filesystem_cannot_be_initialized : Error<
  "CAS filesystem cannot be initialized from root-id '%0': %1">, DefaultFatal;
def err_cas_depscan_failed: Error<
  "CAS-based dependency scan failed: %0">, DefaultFatal;
def err_cas_store: Error<"failed to store to CAS: %0">, DefaultFatal;
def err_cas_missing_root_id : Error<
  "CAS missing expected root-id '%0'">, DefaultFatal;

def err_caching_backend_fail: Error<
  "caching backend error: %0">, DefaultFatal;

def remark_compile_job_cache_hit : Remark<
  "compile job cache hit for '%0' => '%1'">, InGroup<CompileJobCacheHit>;
def remark_compile_job_cache_miss : Remark<
  "compile job cache miss for '%0'">, InGroup<CompileJobCacheMiss>;
def remark_compile_job_cache_miss_result_not_found : Remark<
  "compile job cache miss for '%0' (result not found: '%1')">,
  InGroup<CompileJobCacheMiss>;
def remark_compile_job_cache_backend_output_not_found : Remark<
  "compile job cache backend did not find output '%0' for key '%1' (output ID: '%2')">,
  InGroup<CompileJobCacheMiss>;
def remark_compile_job_cache_skipped : Remark<
  "compile job cache skipped for '%0'">, InGroup<CompileJobCache>;

} // let Component = "CAS" in
