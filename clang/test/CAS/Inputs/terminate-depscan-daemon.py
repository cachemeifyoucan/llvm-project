"""Start `clang -cc1depscand -run` and terminate it with a signal.

Check that the daemon removes its socket file when it is terminated, so
that a new daemon can be started on the same path afterwards.

Usage: terminate-depscan-daemon.py <clang> <base-path> <cas-path>
"""

import os
import signal
import subprocess
import sys
import time


def start_daemon(clang, base, cas):
    daemon = subprocess.Popen(
        [clang, "-cc1depscand", "-run", base, "-cas-args", "-fcas-path", cas]
    )
    sock = base + ".sock"
    deadline = time.time() + 60
    while not os.path.exists(sock):
        if daemon.poll() is not None:
            sys.exit("daemon exited early with %d" % daemon.returncode)
        if time.time() > deadline:
            daemon.kill()
            sys.exit("timed out waiting for " + sock)
        time.sleep(0.05)
    return daemon


def terminate(daemon, sig, base):
    daemon.send_signal(sig)
    daemon.wait(timeout=60)
    if os.path.exists(base + ".sock"):
        os.unlink(base + ".sock")
        sys.exit("socket left behind after signal %d" % sig)


clang, base, cas = sys.argv[1:]
for sig in [signal.SIGTERM, signal.SIGINT]:
    terminate(start_daemon(clang, base, cas), sig, base)
print("ok")
