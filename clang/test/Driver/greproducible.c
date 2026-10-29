// -greproducible is passed to -cc1 and stops -grecord-command-line from
// recording the command line in the debug info.
// RUN: %clang -### -target x86_64-apple-macos11.0 -g -grecord-command-line \
// RUN:   -c %s 2>&1 | FileCheck %s --check-prefix=RECORD
// RUN: %clang -### -target x86_64-apple-macos11.0 -g -grecord-command-line \
// RUN:   -greproducible -c %s 2>&1 | FileCheck %s --check-prefix=REPRO
// RECORD: "-dwarf-debug-flags"
// REPRO: "-greproducible"
// REPRO-NOT: "-dwarf-debug-flags"

// On Darwin, the link sets ZERO_AR_DATE so that ld64 does not record object
// file timestamps.
// RUN: %clang -### -target x86_64-apple-macos11.0 -greproducible %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=LINK
// RUN: %clang -### -target x86_64-apple-macos11.0 %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NO-ENV
// LINK: env "ZERO_AR_DATE=1" "{{[^"]*}}ld"
// NO-ENV-NOT: ZERO_AR_DATE
