"""Run a reviewed Bash script over existing SSH authentication; preserve its log."""
import argparse
from pathlib import Path
import shlex
import subprocess
import sys


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("script", type=Path)
    parser.add_argument("--host", required=True)
    parser.add_argument("--cwd")
    parser.add_argument("--log", required=True, type=Path)
    args = parser.parse_args()
    command = "bash -se"
    if args.cwd:
        command = "cd " + shlex.quote(args.cwd) + " && " + command
    args.log.parent.mkdir(parents=True, exist_ok=True)
    with args.log.open("xb") as log:
        child = subprocess.Popen(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=15",
                                  args.host, command], stdin=subprocess.PIPE,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        child.stdin.write(args.script.read_bytes().replace(b"\r\n", b"\n"))
        child.stdin.close()
        for line in iter(child.stdout.readline, b""):
            log.write(line)
            log.flush()
            print(line.decode("utf-8", errors="replace"), end="", flush=True)
        code = child.wait()
        log.write(f"\nSSH_EXIT_CODE={code}\n".encode())
    raise SystemExit(code)


if __name__ == "__main__":
    main()
