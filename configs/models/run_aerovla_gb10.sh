#!/usr/bin/env bash
# Extra downward RGB diagnostic only, incompatible with supplied hardware suite.
set -euo pipefail
root="$HOME/dronehacks-models"
export HF_HOME="$root/hf-home"
export HF_HUB_DISABLE_IMPLICIT_TOKEN=1
"$root/venv/bin/python" "$root/model_benchmark.py" \
    --strategy aerovla \
    --observations "$root/observations-course-rig-v1.json" \
    --upstream "$root/AeroVLA" \
    --checkpoint XuPeng23/AerialVLA \
    --revision 196f2f3253b69df6e90ac10b6ae041c7b3a9569e \
    --local-checkpoint "$root/checkpoints/aerovla-196f2f" \
    --base-revision 47a0ec7fc4ec123775a391911046cf33cf9ed83f \
    --local-base "$root/checkpoints/openvla-47a0ec" \
    --cache "$root/cache" --warmup 1 --repetitions 3 \
    --output "$root/results/aerovla-course-rig-v1"
