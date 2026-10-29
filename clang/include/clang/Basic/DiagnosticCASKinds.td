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

} // let Component = "CAS" in
