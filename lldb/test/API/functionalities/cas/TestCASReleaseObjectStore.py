"""
Test that CAS object stores are released once nothing references the modules
that were loaded out of them.
"""

import re

import lldb
from lldbsuite.test.decorators import *
from lldbsuite.test.lldbtest import *
import lldbsuite.test.lldbutil as lldbutil


class TestCASReleaseObjectStore(TestBase):
    NO_DEBUG_INFO_TESTCASE = True

    def load_both_modules(self, target):
        """Stop in each dylib and evaluate an expression there, so that both
        SymbolFiles import their clang module -- one out of the CAS, one from a
        .pcm on disk."""
        for source, pattern, expected in [
            ("cached.c", "BREAK CACHED", "x = 41"),
            ("uncached.c", "BREAK UNCACHED", "y = 17"),
        ]:
            bkpt = target.BreakpointCreateBySourceRegex(
                pattern, lldb.SBFileSpec(source)
            )
            self.assertEqual(bkpt.GetNumLocations(), 1, source)

        process = target.LaunchSimple(None, None, self.get_process_working_directory())
        self.assertState(process.GetState(), lldb.eStateStopped)

        for variable, expected in [("c", "x = 41"), ("u", "y = 17")]:
            frame = process.GetSelectedThread().GetFrameAtIndex(0)
            value = frame.EvaluateExpression(variable)
            self.assertSuccess(value.GetError(), "evaluating '%s'" % variable)
            self.assertIn(expected, str(value))
            process.Continue()

        return process

    def debug_in_a_fresh_debugger(self, executable, log_path):
        """Debug `executable` in a debugger of our own -- the test's primary
        debugger would keep the modules alive -- capturing the module log,
        then destroy that debugger. Returns the log contents."""
        debugger = lldb.SBDebugger.Create()
        try:
            debugger.SetAsync(False)
            result = lldb.SBCommandReturnObject()
            debugger.GetCommandInterpreter().HandleCommand(
                'log enable lldb module -f "%s"' % log_path, result
            )
            self.assertTrue(result.Succeeded(), result.GetError())

            target = debugger.CreateTarget(executable)
            self.assertTrue(target.IsValid())
            self.load_both_modules(target)
        finally:
            lldb.SBDebugger.Destroy(debugger)

        with open(log_path, "r") as f:
            return f.read()

    def assertModuleConstructed(self, log_contents, path, msg=None):
        pattern = r"Module::Module\(\(.*?\) '%s'\)" % re.escape(path)
        self.assertRegex(log_contents, pattern, msg)

    def assertModuleNotConstructed(self, log_contents, path, msg=None):
        pattern = r"Module::Module\(\(.*?\) '%s'\)" % re.escape(path)
        self.assertNotRegex(log_contents, pattern, msg)

    @skipUnlessDarwin
    def test_release_on_api_call(self):
        """SBDebugger.ReleaseCASObjectStores() reclaims the store once the
        target that used it is gone, and reports nothing left referencing it."""
        self.build()
        log = self.getBuildArtifact("api_call.log")
        result = lldb.SBCommandReturnObject()
        self.dbg.GetCommandInterpreter().HandleCommand(
            'log enable lldb module -f "%s"' % log, result
        )
        self.assertTrue(result.Succeeded(), result.GetError())

        target = self.dbg.CreateTarget(self.getBuildArtifact("a.out"))
        self.assertTrue(target.IsValid())
        self.load_both_modules(target)

        # While the target is alive it still references the CAS-backed modules,
        # so the store cannot be released yet, and nothing is removed.
        self.assertEqual(lldb.SBDebugger.ReleaseCASObjectStores(), 1)

        self.dbg.DeleteTarget(target)
        del target

        # Now nothing references them, so the store is reclaimed. Two
        # modules are removed: the module loaded out of the CAS, and the .o
        # that held it.
        self.assertEqual(lldb.SBDebugger.ReleaseCASObjectStores(), 0)

        with open(log, "r") as f:
            contents = f.read()
        self.assertIn(
            "Released object stores: 0 module(s) removed, 0 released, "
            "1 still referenced",
            contents,
        )
        self.assertIn(
            "Released object stores: 2 module(s) removed, 1 released, "
            "0 still referenced",
            contents,
        )

    @skipUnlessDarwin
    def test_release_on_debugger_destroy(self):
        """Destroying a debugger releases its CAS object stores, and only drags
        along the modules that were keeping them alive. Debugging the same
        binary again works, and reloads and reparses only the modules that
        were released -- everything else is reused."""
        self.build()
        executable = self.getBuildArtifact("a.out")

        first_log = self.debug_in_a_fresh_debugger(
            executable, self.getBuildArtifact("first_session.log")
        )
        self.assertIn("Released CAS at", first_log)
        self.assertIn(
            "Released object stores: 2 module(s) removed, 1 released, "
            "0 still referenced",
            first_log,
        )

        # Debug the same binary again in a new debugger. This must still
        # work, and the log tells us what got rebuilt in the process.
        second_log = self.debug_in_a_fresh_debugger(
            executable, self.getBuildArtifact("second_session.log")
        )

        cached_dylib = self.getBuildArtifact("cached/libcached.dylib")
        uncached_dylib = self.getBuildArtifact("uncached/libuncached.dylib")

        # The cached dylib's Module, and the clang module loaded out of the
        # CAS, were both released, so both are constructed afresh and their
        # debug info is reparsed to satisfy the new breakpoint and expression.
        self.assertModuleConstructed(second_log, cached_dylib)
        self.assertIn("Loading module 'Cached' from CAS at", second_log)
        self.assertIn("Initialized CAS at", second_log)

        # The uncached dylib and the executable were never released. Their
        # Module objects, and the SymbolFile that already parsed their debug
        # info the first time, are reused rather than rebuilt from scratch.
        self.assertModuleNotConstructed(second_log, uncached_dylib)
        self.assertModuleNotConstructed(second_log, executable)
