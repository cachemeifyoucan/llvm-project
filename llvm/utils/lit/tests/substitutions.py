# Basic test for substitutions.
#
# RUN: echo %{s:basename} | FileCheck %s --check-prefix=BASENAME
# RUN: echo %{s:stem} | FileCheck %s --check-prefix=STEM
# RUN: echo %{t:stem} %basename_t | FileCheck %s --check-prefix=TMPBASENAME

# BASENAME: substitutions.py
# STEM: substitutions
# TMPBASENAME: [[FIRST:[^[:space:]]+]] [[FIRST]]

# Path substitutions used by tests that need Windows-friendly paths.
#
# RUN: echo %{fs-sep-yaml} | FileCheck %s --check-prefix=FSSEPYAML
# RUN: echo %{t-lower} %t | FileCheck %s --ignore-case --check-prefix=TLOWER
# RUN: echo %/root | FileCheck %s --check-prefix=ROOT

# FSSEPYAML: {{^(/|\\+)$}}
# TLOWER: [[TMP:[^[:space:]]+]] [[TMP]]
# ROOT: {{^([A-Za-z]:)?/$}}
