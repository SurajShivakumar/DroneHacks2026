"""Run a reviewed Bash file in WSL using LF bytes, with a persistent output log.

Avoids Windows native-argument quoting and CRLF stdin errors. No shell command
is constructed from user text. This helper is for local laptop setup only.
"""

import argparse
from pathlib import Path
import subprocess
import sys


def main():
    # Native Windows terminals may default to cp1252; pip emits Unicode bars.
    # Never terminate a Linux build merely because its progress cannot be printed.
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("script", type=Path)
    parser.add_argument("--distro", default="Ubuntu-22.04")
    parser.add_argument("--user")
    parser.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()
    command = ["wsl", "-d", args.distro]
    if args.user:
        command += ["-u", args.user]
    command += ["--", "bash", "-se"]
    contents = args.script.read_bytes().replace(b"\r\n", b"\n")
    args.log.parent.mkdir(parents=True, exist_ok=True)
    with args.log.open("xb") as log:
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT)
        try:
            process.stdin.write(contents)
            process.stdin.close()
            for line in iter(process.stdout.readline, b""):
                log.write(line)
                log.flush()
                print(line.decode("utf-8", errors="replace"), end="", flush=True)
            result = process.wait()
            log.write(f"\nWSL_EXIT_CODE={result}\n".encode())
        except BaseException:
            process.terminate()
            process.wait()
            raise
    raise SystemExit(result)


if __name__ == "__main__":
    main()
