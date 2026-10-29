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

} // let Component = "CAS" in
