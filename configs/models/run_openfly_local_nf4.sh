#!/usr/bin/env bash
# Laptop NF4 language-only variant; recorded sim pixels, no flight/control.
set -euo pipefail
root="$HOME/dronehacks-models-local"
export HF_HOME="$root/hf-home"
export HF_HUB_DISABLE_IMPLICIT_TOKEN=1
"$root/venv/bin/python" "$PWD/tools/model_benchmark.py" \
    --strategy openfly --precision nf4 \
    --observations "$root/observations-course-rig-v1.json" \
    --upstream "$root/OpenFly-Platform" \
    --checkpoint IPEC-COMMUNITY/openfly-agent-7b \
    --revision 21dcce235f1f2d40fc23c76998051abc5434cc99 \
    --local-checkpoint "$root/checkpoints/openfly-21dcce" \
    --cache "$root/cache" --unnorm-key vlnv1 --warmup 1 --repetitions 3 \
    --hash-cache "$root/checkpoint-hashes.json" \
    --output "$root/results/openfly-course-rig-nf4-v3"
