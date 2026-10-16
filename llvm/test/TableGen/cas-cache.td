// Caching llvm-tblgen invocations in a CAS with --cas-path.
// REQUIRES: ondisk_cas

// RUN: rm -rf %t && split-file %s %t && cd %t

// A cache miss stores the result, and a hit replays it. Both match an
// uncached run, including the dependency file.
// RUN: llvm-tblgen -I inc main.td -o ref.inc -d ref.d
// RUN: llvm-tblgen --cas-path=cas --cas-remarks -I inc main.td -o ref.inc \
// RUN:   -d miss.d 2>&1 | FileCheck %s --check-prefix=MISS
// RUN: llvm-tblgen -I inc main.td -o out.inc
// RUN: diff ref.inc out.inc
// RUN: diff ref.d miss.d
// RUN: rm ref.inc
// RUN: llvm-tblgen --cas-path=cas --cas-remarks -I inc main.td -o ref.inc \
// RUN:   -d hit.d 2>&1 | FileCheck %s --check-prefix=HIT
// RUN: diff out.inc ref.inc
// RUN: diff ref.d hit.d
// MISS: remark: cache miss for '[[KEY:llvmcas://[0-9a-f]+]]'
// HIT: remark: cache hit for 'llvmcas://{{[0-9a-f]+}}'

// The key only depends on what determines the outputs: not on the dependency
// file or --write-if-changed, but on the output file (whose stem backends can
// see), the command line, and the contents of every included file.
// RUN: llvm-tblgen --cas-path=cas --cas-print-cache-key -I inc main.td \
// RUN:   -o ref.inc > key.1
// RUN: llvm-tblgen --cas-path=cas --cas-print-cache-key -I inc main.td \
// RUN:   -o ref.inc -d other.d --write-if-changed > key.2
// RUN: diff key.1 key.2
// RUN: llvm-tblgen --cas-path=cas --cas-print-cache-key -I inc main.td \
// RUN:   -o other.inc > key.out
// RUN: not diff key.1 key.out
// RUN: llvm-tblgen --cas-path=cas --cas-print-cache-key -I inc main.td \
// RUN:   -o ref.inc -DEXTRA > key.def
// RUN: not diff key.1 key.def
// RUN: cp -R inc inc.orig
// RUN: echo "def Changed;" >> inc/deeper/b.td
// RUN: llvm-tblgen --cas-path=cas --cas-print-cache-key -I inc main.td \
// RUN:   -o ref.inc > key.changed
// RUN: not diff key.1 key.changed
// RUN: rm -rf inc && mv inc.orig inc

// The scan parses the input, so an include in an inactive preprocessor region
// is not an input: changing the file it names only changes the key when the
// region is active.
// RUN: llvm-tblgen --cas-path=cas --cas-print-cache-key -I inc main.td \
// RUN:   -o ref.inc -DEXTRA > key.extra
// RUN: echo "def Changed;" >> inc/extra.td
// RUN: llvm-tblgen --cas-path=cas --cas-print-cache-key -I inc main.td \
// RUN:   -o ref.inc > key.extra.off
// RUN: diff key.1 key.extra.off
// RUN: llvm-tblgen --cas-path=cas --cas-print-cache-key -I inc main.td \
// RUN:   -o ref.inc -DEXTRA > key.extra.on
// RUN: not diff key.extra key.extra.on
// RUN: llvm-tblgen --cas-path=cas -I inc main.td -o extra.inc -DEXTRA
// RUN: FileCheck %s --check-prefix=EXTRA < extra.inc
// EXTRA: def Changed
// EXTRA: def Extra

// Round trip from a key back to the invocation: the key records the command
// line, working directory, and a snapshot of the inputs, so it can be run
// without the original files. The key recreated from the recorded invocation
// must match. Running the key misses (-o differs from the cached run above)
// and computes the result from the snapshot alone; running it again hits.
// RUN: llvm-tblgen --cas-path=cas --cas-print-cache-key -I inc main.td \
// RUN:   -o roundtrip.inc -d roundtrip.d > key.rt
// RUN: mkdir saved && mv main.td inc saved
// RUN: llvm-tblgen --cas-path=cas --cas-remarks \
// RUN:   --cas-run-cache-key=%{readfile:key.rt} -d roundtrip.d 2>&1 \
// RUN:   | FileCheck %s --check-prefix=RT-MISS
// RUN: diff out.inc roundtrip.inc
// RUN: sed 's/^roundtrip.inc:/ref.inc:/' roundtrip.d | diff ref.d -
// RUN: rm roundtrip.inc
// RUN: llvm-tblgen --cas-path=cas --cas-remarks \
// RUN:   --cas-run-cache-key=%{readfile:key.rt} 2>&1 \
// RUN:   | FileCheck %s --check-prefix=RT-HIT
// RUN: diff out.inc roundtrip.inc
// RT-MISS: remark: cache miss for '[[KEY:llvmcas://[0-9a-f]+]]'
// RT-HIT: remark: cache hit for 'llvmcas://{{[0-9a-f]+}}'
// RUN: mv saved/main.td saved/inc .

// A run that prints warnings is cached, but warnings are not part of the
// result, so a cache hit does not replay them. The scan does not report
// warnings itself, so a miss prints them once.
// RUN: llvm-tblgen --cas-path=cas --cas-remarks -I inc warn.td -o warn.inc \
// RUN:   2>&1 | FileCheck %s --check-prefix=WARN-MISS
// RUN: llvm-tblgen -I inc warn.td -o warn.ref 2>/dev/null
// RUN: diff warn.ref warn.inc
// RUN: rm warn.inc
// RUN: llvm-tblgen --cas-path=cas --cas-remarks -I inc warn.td -o warn.inc \
// RUN:   2>&1 | FileCheck %s --check-prefix=WARN-HIT --implicit-check-not=warning:
// RUN: diff warn.ref warn.inc
// WARN-MISS: remark: cache miss for 'llvmcas://{{[0-9a-f]+}}'
// WARN-MISS-NEXT: warning: unused template argument
// WARN-MISS-NOT: warning:
// WARN-HIT: remark: cache hit for 'llvmcas://{{[0-9a-f]+}}'

// An error found by the scan is reported by the scan, once, with the include
// stack.
// RUN: not llvm-tblgen --cas-path=cas -I inc error.td -o error.inc 2>&1 \
// RUN:   | FileCheck %s --check-prefix=ERROR --implicit-check-not=error:
// ERROR: Included from error.td:1:
// ERROR: bad.td:2:9: error: Couldn't find class 'Missing'

// Errors.
// RUN: not llvm-tblgen --cas-path=cas < main.td 2>&1 \
// RUN:   | FileCheck %s --check-prefix=STDIN
// STDIN: --cas-path requires an input file
// RUN: not llvm-tblgen --cas-path=cas --cas-run-cache-key=%{readfile:key.rt} \
// RUN:   main.td 2>&1 | FileCheck %s --check-prefix=EXTRA-ARGS
// EXTRA-ARGS: --cas-run-cache-key cannot be combined with 'main.td'
// RUN: not llvm-tblgen --cas-print-cache-key main.td 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NO-CAS
// NO-CAS: --cas-print-cache-key and --cas-run-cache-key require --cas-path

//--- main.td
include "a.td"
// A commented-out include of a missing file is ignored by the scan:
// include "missing.td"
#ifdef EXTRA
include "extra.td"
#endif
def Main : A;

//--- inc/a.td
include "deeper/b.td"
class A : B;

//--- inc/deeper/b.td
class B {
  int Value = 42;
}

//--- inc/extra.td
def Extra;

//--- error.td
include "bad.td"

//--- inc/bad.td
class Bad;
def X : Missing;

//--- warn.td
class Unused<int X>;
def W : Unused<1>;
