# Aerial model benchmark status

This report covers offline policy inference on recorded Gazebo pixels. It does
not report flown course success, simulation finish time or hardware readiness.
Those require a validated control adapter and actual closed-loop runs.

November 5 release recheck: the official
[OpenFlyV2 repository](https://github.com/Ideal-ljl/AeroBrain#release-status)
still lists code, checkpoints and usage instructions as forthcoming. It
cannot currently supply the third runnable released policy in this comparison.

**Completed GB10 BF16 benchmark:** OpenFly-Agent7B loaded the fully verified
released checkpoint without missing/unexpected weights. Three synchronized
inference samples after one warm-up took **0.741909 / 0.738210 / 0.742602 s**;
median **0.741909 s**. Warm-up was 1.533888 s and model load 104.03 s. Peak
allocated GPU memory during inference was **15,289,178,624 bytes (14.24 GiB)**.
All four outputs are saved, with the recorded input hashes, package versions,
source/checkpoint revisions and an empty loading-error report in
[result](artifacts/openfly-gb10-bf16-v2/result.json).
This uses GB10, three recorded front frames and the native BF16 model; no
fine-tuning or flight commands. Its action-to-flight mapping remains unqualified.
It does not fit the Orin Nano's 8 GB in this form.

Latest local NF4 attempt v2 returned four inference samples, then failed result
export because Transformers loading metadata contained a set. The runner now
exports those sets as sorted lists, covered by a regression test. Fresh v3
stopped before loading: `nvidia-smi` returned permission denied, both in WSL and
on Windows. No latency/result record or GPU availability is fabricated. The
Gazebo/PX4 work on the separate GB10 continues independently.

| Model | Public release checked 2026-11-03 | Current task status | Course time / model flight video |
| --- | --- | --- | --- |
| OpenFly-Agent7B | Full 15.086GB checkpoint verified on GB10 | BF16 offline inference completed, median 0.741909 s | No course flight |
| AeroVLA | LoRA462.7MB plus OpenVLA7B base available | Concrete dual-view adapter implemented; separate15.085GB base not staged | Not run; physical sensor suite incompatible with native dual-view input |
| OpenFlyV2 | Code/checkpoint still coming soon | Blocked upstream | Unavailable |

No latency number is claimed before successful inference. Download time, model
load time, warm-up and synchronized GPU inference wall latency are separate
fields. The runner writes `finish_time_sim_s: null`, never derives it from video
length, and never publishes any flight controls. No fine-tuning is performed here.
Full BF16 weights alone exceed the supplied Jetson Orin Nano8GB memory. A GB10
run therefore does not establish onboard deployability or Jetson latency;
quantized/compressed variants need their own measured evaluation and labels.

Pinned model and source revisions are recorded in `configs/models/*.json`.
OpenFly uses the pinned upstream registered custom model, processor and
`predict_action`, with `vlnv1` normalization matching its evaluation script. The
README example uses another key, `vln_norm`; it is not silently interchanged.
Its upstream macro action interpreter uses metre-scale translations and 30-degree
turns. A dense indoor course needs an explicitly qualified translation from those
macros; changing the planner must be disclosed as a hybrid rather than model-only
control. [OpenFly source](https://github.com/SHAILAB-IPEC/OpenFly-Platform).

AeroVLA independently resizes front/down images to224x224 and vertically stacks
them, then generates three text bins for forward/down/yaw and a landing signal.
The native code silently substitutes zero actions on malformed text; this runner
retains the text and marks that output invalid. The supplied Sky Beta hardware
has a front OAK and downward rangefinder, not downward RGB. A simulated downward
camera can test the model offline but adds a sensor and cannot establish hardware
compatibility. [AeroVLA source](https://github.com/XuPeng23/AeroVLA).

OpenFlyV2's repository still contains assets/README and explicitly lists code,
checkpoints and instructions as forthcoming. Its reported10Hz is an upstream
claim on Orin NX, not a measurement here. [Release status](https://github.com/Ideal-ljl/AeroBrain#release-status).

## Recorded input and runtime preparation

The GB10 source run `gb10-course-rig-v1` supplies three actual separately received
front frames at73.062488310,73.129154966,73.195821622 simulated seconds. The
independent downward frame is timestamped73.195821622, exactly matching the last
front frame. Front frame hashes are identical because the rig had stopped;
timestamps remain genuinely distinct. No front frame is used as a down view.
The v1 down CameraInfo topic inherited the front topic; the offline inference
runner does not consume CameraInfo. This limitation remains explicit in the
input manifest, and these are diagnostic sensor recordings rather than flight.

At initial model setup, GB10 GPU utilization was0% and available RAM about115GiB.
It had no system PyTorch. A separate user-owned environment under
`~/dronehacks-models/venv` installed torch2.9.1 and torchvision0.24.1 from the
official CUDA13 ARM64 index. No system packages or existing environments are
modified. Package/checkpoint download progress is not inference timing. A bounded
public-weight HTTP Range probe read8MiB in31.486 seconds on GB10 (0.254MiB/s,
including connection setup). That is only a short transfer sample, not sustained
full-model throughput. The parent task measured a much faster laptop route and
is staging SHA256-verified weights separately. The slow GB10 snapshot download
was stopped explicitly; CUDA dependency installation subsequently completed.
The original NVIDIA cuSPARSE route later measured only0.0283MiB/s. The identical
public PyPI wheel was downloaded and SHA256-verified through a faster route.
Completed CUDA wheels were preserved, and the normal pip installer now consumes
explicit local wheel paths to avoid re-downloading them. The core torch CUDA13
wheel is staged and verified (512448274bytes, SHA256
`6e7f84cb10c7e7d9f862c318f056d64840544ab4f0bcbf8cf7ed6047fe04051f`).
Native runtime dependencies have now installed successfully. A genuine CUDA
matrix operation and pinned OpenFly processor on the three recorded frames passed:
pixel_values shape3x6x224x224. PyTorch warns that GB10 capability12.1 exceeds its
advertised maximum12.0; the simple CUDA operation passed, but full model kernels
remain untested. The15GB model transfer still precedes actual inference; none of
this download/setup work counts as a policy timing.

BF16 matrix multiplication and scaled-dot-product-attention kernel smoke checks
also passed on GB10; these use synthetic tensors and are not model benchmarks.
The smallest OpenFly shard was received, SHA256-verified and inspected through
SafeTensors on CPU:23tensor names match the released index exactly, with BF16
metadata. The first two growing uploads have not been loaded or renamed.

Separate official Hugging Face Hub2.4.0/hf_xet1.8.0 high-performance download tests
were bounded at120seconds direct and90seconds through the laptop proxy. Neither
completed a verified weight; only metadata-size cache allocations were observed.
Both processes were stopped at their bounds; normal uploads continue. Inference
keeps its separate pinned Hub0.34.4 environment. AeroVLA's base has the same shard
sizes but different hashes from OpenFly; substituting those weights would be
incorrect.

Static architecture inspection found two redundant `timm.create_model` calls
with `pretrained=True` in the released OpenFly constructor, which download
independent DINO/SigLIP initializers even when loading a full OpenFly checkpoint.
The runner suppresses only those initialization downloads, after verifying all
three released weight hashes and both vision branches in the index. It then
requires empty missing/unexpected/mismatched/error loading lists before any
CUDA forward. This does not modify released weights or train the model. A
separate meta-device shape-construction check passed with7,541,237,184 parameters
and action dimension8; that check has no model outputs or inference latency.
The tokenizer is pinned to0.21.1, exactly as the reviewed upstream code expects.

## Laptop fallback variant

While GB10 weight uploads continue, an isolated WSL environment under
`~/dronehacks-models-local` is running a separate OpenFly NF4 attempt on the
RTX5070Ti Laptop GPU12GB. It uses the same SHA256-verified released files through
read-only links, same three recorded images, normalization and instruction. The
language model's linear layers use NF4 double quantization with BF16 compute;
vision, projector and output head remain unquantized. The runner verifies the
actual quantized module set and retains complete loading information. It is
explicitly a different precision/device variant, not a GB10 BF16 result or Jetson
qualification. No weight updates are performed. Exact dependencies and launch
script are `configs/models/requirements-local-nf4.txt` and
`configs/models/run_openfly_local_nf4.sh`. The first attempt verified all three
weight hashes, then failed before model loading because forcing `use_fast=False`
selected the checkpoint's unusable76-byte slow-tokenizer asset. Reverting to the
native fast-tokenizer default passed an independent real-input processor check.
The failed result remains under the isolated local runtime. The second attempt
is running. A full-hash cache now validates file identity/size/timestamps before
reuse; its first creation still recomputes all hashes. Checkpoint hashing/loading
time must not be reported as inference.

The concrete runner is `tools/model_benchmark.py`. It rejects missing/replayed
history, image hash mismatch, duplicated front/down paths and badly aligned
dual-view timestamps. It checks GPU load before inference, pins reviewed upstream
source, saves model file hashes and exact runtime, separates warm-up from repeated
measurements, and preserves failure evidence. Six focused tests pass, including
rejection of incomplete/growing weight uploads before any CUDA invocation. Real model
outputs and measured latency remain pending complete weight transfer.
