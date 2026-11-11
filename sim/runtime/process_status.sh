#!/usr/bin/env bash
set -euo pipefail
ps -eo pid,ppid,comm,args | grep -E '[p]ip install|[n]inja|[c]make|[p]x4|[M]icroXRCE|[g]z sim' || true
