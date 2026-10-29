"""Start `clang-cache -depscan-server` and compile through it.

Check that the server gets the CAS from LLVM_CACHE_CAS_PATH, and that
clang-cache connects to it through CLANG_CACHE_SCAN_DAEMON_SOCKET_PATH.

Usage: clang-cache-depscan-server.py <clang-cache> <clang> <base-path>
           <cas-path> <source>
"""

import os
import signal
import subprocess
import sys
import time

clang_cache, clang, base, cas, source = sys.argv[1:]
env = dict(os.environ, LLVM_CACHE_CAS_PATH=cas)

server = subprocess.Popen([clang_cache, "-depscan-server", base], env=env)
sock = base + ".sock"
deadline = time.time() + 60
while not os.path.exists(sock):
    if server.poll() is not None:
        sys.exit("server exited early with %d" % server.returncode)
    if time.time() > deadline:
        server.kill()
        sys.exit("timed out waiting for " + sock)
    time.sleep(0.05)

try:
    # The client has not run yet, so only the server can create the CAS.
    while not os.path.isdir(cas):
        if time.time() > deadline:
            sys.exit("server did not create the CAS at " + cas)
        time.sleep(0.05)
    env["CLANG_CACHE_SCAN_DAEMON_SOCKET_PATH"] = base
    compile = subprocess.run(
        [clang_cache, clang, "-fsyntax-only", source, "-Rcompile-job-cache"],
        env=env,
        stderr=subprocess.PIPE,
        universal_newlines=True,
    )
    sys.stdout.write(compile.stderr)
    if compile.returncode != 0:
        sys.exit("compilation failed with %d" % compile.returncode)
finally:
    server.send_signal(signal.SIGTERM)
    server.wait(timeout=60)
print("ok")
