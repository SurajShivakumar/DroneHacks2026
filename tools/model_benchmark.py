"""Offline aerial-policy inference on recorded images; never publishes controls.

Timing is synchronized GPU wall latency, not simulator flight or finish time.
Pinned upstream code is executable code: review it before supplying --upstream.
"""
import argparse
from datetime import datetime, timezone
from hashlib import sha256
import json
import math
from pathlib import Path
import platform
import re
import statistics
import subprocess
import sys
import time
import shutil
import os


OPENFLY_WEIGHTS = {
    'model-00001-of-00003.safetensors': 'a40777127f28bb99a18107c8145057b9ca3762f8e65da5ed67a3bb8bd84b171d',
    'model-00002-of-00003.safetensors': 'e6b9e972ba51d6920403670dd54e0b305f34315a492ce874120792bbd7a8c7a3',
    'model-00003-of-00003.safetensors': '4a33348f85fc6c25d5f4fef6abb4000593bb2bb5d9b99cfdd9d8869bb04e43c1'}


def metadata_json_default(value):
    """Transform Transformers loading-info sets without discarding their keys."""
    if isinstance(value, (set, frozenset)) and all(isinstance(item,str) for item in value):
        return sorted(value)
    raise TypeError('Unsupported benchmark metadata type: '+type(value).__name__)


def digest(path):
    value = sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def checkpoint_hashes(checkpoint, cache_path=None):
    """Reuse full hashes only while file identity, size and timestamps match."""
    root = Path(checkpoint).resolve()
    cache = {}
    if cache_path and cache_path.exists():
        saved = json.loads(cache_path.read_text())
        if saved.get('root') == str(root):
            cache = saved.get('files', {})
    values, entries, reused = {}, {}, 0
    for path in root.rglob('*'):
        if not path.is_file() or '.upload' in path.name or '.assembling' in path.name:
            continue
        name = str(path.relative_to(root))
        def identity():
            s = path.stat()
            return [s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns]
        before = identity()
        old = cache.get(name, {})
        if old.get('identity') == before and re.fullmatch(r'[a-f0-9]{64}', old.get('sha256', '')):
            value = old['sha256']; reused += 1
        else:
            value = digest(path)
        if identity() != before:
            raise RuntimeError('checkpoint changed during verification')
        values[name] = value
        entries[name] = {'identity': before, 'sha256': value}
    if cache_path:
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = cache_path.with_name(cache_path.name + f'.tmp-{os.getpid()}')
        with temporary.open('x') as stream:
            json.dump({'root': str(root), 'files': entries}, stream, indent=2)
        temporary.replace(cache_path)
    return values, {'fresh_full_hashes': len(values) - reused, 'reused_full_hashes': reused,
                    'reuse_condition': 'same device/inode/size/mtime_ns/ctime_ns'}


def validate_observations(data, strategy):
    if not isinstance(data.get('instruction'), str) or not data['instruction'].strip():
        raise ValueError('nonempty instruction required')
    frames = data.get('front_history', [])
    if len(frames) != 3:
        raise ValueError('three actual front history frames required')
    stamps = [frame['stamp_ns'] for frame in frames]
    if any(type(v) is not int or v < 0 for v in stamps) or any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError('strictly increasing actual source timestamps required')
    needed = list(frames)
    if strategy == 'aerovla':
        down = data.get('down')
        if not down or not data.get('extra_down_camera_diagnostic'):
            raise ValueError('AeroVLA requires real downward RGB; extra sensor diagnostic must be declared')
        if abs(down['stamp_ns'] - stamps[-1]) > 20_000_000:
            raise ValueError('front/down source timestamps differ by more than 20 ms')
        if Path(down['path']).resolve() == Path(frames[-1]['path']).resolve():
            raise ValueError('duplicating front image as down is invalid')
        needed.append(down)
    for frame in needed:
        if digest(frame['path']) != frame['sha256']:
            raise ValueError('input image hash mismatch')
    return needed


def decode_aerovla(text):
    """Expose malformed output instead of upstream's silent zero/landing fallback."""
    suffix = text.split('Action:')[-1]
    bins = [int(v) for v in re.findall(r'-?\d+', suffix)]
    land = 'LAND' in suffix
    if len(bins) < 3:
        return {'valid_numeric_action': False, 'explicit_land_token': land, 'raw': suffix}
    bins = bins[-3:]
    if any(v < 0 or v > 99 for v in bins):
        return {'valid_numeric_action': False, 'explicit_land_token': land, 'raw': suffix}
    values = [bins[0] / 99 * 5, bins[1] / 99 * 10 - 5, bins[2] / 99 * 2.2 - 1.1]
    return {'valid_numeric_action': True, 'bins': bins, 'forward': values[0],
            'down': values[1], 'yaw': values[2], 'explicit_land_token': land,
            'upstream_numeric_land_heuristic': values[0] < .01 and abs(values[1]) < .01 and abs(values[2]) < .01}


def gpu_load():
    executable = shutil.which('nvidia-smi') or '/usr/lib/wsl/lib/nvidia-smi'
    result = subprocess.run([executable, '--query-gpu=utilization.gpu', '--format=csv,noheader,nounits'],
                            capture_output=True, text=True, check=True)
    return [int(v.strip()) for v in result.stdout.splitlines() if v.strip()]


def run(args):
    args.output.mkdir(parents=True, exist_ok=False)
    precision = getattr(args, 'precision', 'bf16')
    result = {'kind': 'offline_policy_inference_not_flight', 'strategy': args.strategy,
              'checkpoint_repository': args.checkpoint, 'dtype_requested': 'bfloat16',
              'attention_implementation_requested': 'sdpa', 'quantized': precision == 'nf4',
              'variant': 'nf4_language_only' if precision == 'nf4' else 'bf16',
              'created_utc': datetime.now(timezone.utc).isoformat(), 'finetuned_here': False,
              'finish_time_sim_s': None, 'commands_published': False,
              'script_sha256': digest(__file__), 'status': 'starting'}
    try:
        data = json.loads(args.observations.read_text())
        if precision == 'nf4' and args.strategy != 'openfly':
            raise ValueError('NF4 is implemented only for the explicitly labeled OpenFly variant')
        needed = validate_observations(data, args.strategy)
        if args.local_checkpoint and not args.local_checkpoint.is_dir():
            raise ValueError('staged checkpoint directory is missing')
        if args.local_checkpoint and args.strategy == 'openfly' and any(
                not (args.local_checkpoint / name).is_file() for name in OPENFLY_WEIGHTS):
            raise ValueError('staged OpenFly weights incomplete; do not read growing uploads')
        if args.local_base and not args.local_base.is_dir():
            raise ValueError('staged base directory is missing')
        result['input_manifest_sha256'] = digest(args.observations)
        result['inputs'] = needed
        print('Validated recorded input timestamps and image hashes.', flush=True)
        result['instruction'] = data['instruction']
        result['extra_down_camera_diagnostic'] = args.strategy == 'aerovla' and bool(data.get('extra_down_camera_diagnostic'))
        result['gpu_load_before_percent'] = gpu_load()
        if max(result['gpu_load_before_percent'], default=0) > args.max_gpu_util:
            raise RuntimeError('GPU busy; run at a separate time for meaningful latency')
        import torch
        from PIL import Image
        from huggingface_hub import snapshot_download
        from transformers import AutoConfig, AutoImageProcessor, AutoModelForVision2Seq, AutoProcessor, AutoTokenizer
        from importlib.metadata import version, PackageNotFoundError
        result['package_versions'] = {}
        for package in ('transformers', 'timm', 'accelerate', 'peft', 'torchvision', 'huggingface_hub', 'sentencepiece', 'bitsandbytes'):
            try:
                result['package_versions'][package] = version(package)
            except PackageNotFoundError:
                result['package_versions'][package] = None
        if not torch.cuda.is_available():
            raise RuntimeError('CUDA unavailable; no CPU timing silently substituted')
        result['runtime'] = {'python': platform.python_version(), 'machine': platform.machine(),
                             'torch': torch.__version__, 'cuda': torch.version.cuda,
                             'gpu': torch.cuda.get_device_name(), 'capability': torch.cuda.get_device_capability(),
                             'torch_cuda_arch_list': torch.cuda.get_arch_list()}
        source_commit = subprocess.check_output(['git', '-C', str(args.upstream), 'rev-parse', 'HEAD'], text=True).strip()
        expected_source = {'openfly': 'c075075497a7122bad82f5b76b9be926ad5a81b3',
                           'aerovla': '3ba3e244262cb5ff6bcf4c24755d3b4bb19a1d27'}[args.strategy]
        if source_commit != expected_source:
            raise ValueError('upstream source revision differs from reviewed adapter contract')
        result['upstream_commit'] = source_commit
        result['upstream_diff'] = subprocess.check_output(['git', '-C', str(args.upstream), 'diff'], text=True)
        result['model_revision'] = args.revision
        started = time.perf_counter()
        print('Verifying complete checkpoint file hashes; this is preparation, not inference.', flush=True)
        checkpoint = str(args.local_checkpoint.resolve()) if args.local_checkpoint else snapshot_download(
            args.checkpoint, revision=args.revision, cache_dir=str(args.cache),
            ignore_patterns=['*.jsonl', '*.parquet', '*dataset.json'])
        result['checkpoint_files'], result['checkpoint_hash_verification'] = checkpoint_hashes(
            checkpoint, getattr(args, 'hash_cache', None))
        if args.strategy == 'openfly':
            if any(result['checkpoint_files'].get(name) != value for name, value in OPENFLY_WEIGHTS.items()):
                raise ValueError('OpenFly weights missing or hash differs from pinned public release')
            index = json.loads((Path(checkpoint) / 'model.safetensors.index.json').read_text())
            prefixes = ('vision_backbone.featurizer.', 'vision_backbone.fused_featurizer.')
            result['indexed_vision_tensor_counts'] = {
                prefix: sum(name.startswith(prefix) for name in index['weight_map']) for prefix in prefixes}
            if not all(result['indexed_vision_tensor_counts'].values()):
                raise ValueError('full checkpoint must contain both native vision branches')
            print('All released OpenFly shard hashes and both vision branches verified.', flush=True)
        result['download_and_hash_wall_s'] = time.perf_counter() - started
        if args.strategy == 'aerovla':
            if not args.base_revision:
                raise ValueError('pinned AeroVLA base revision required')
            started = time.perf_counter()
            base = str(args.local_base.resolve()) if args.local_base else snapshot_download(
                'openvla/openvla-7b', revision=args.base_revision, cache_dir=str(args.cache))
            result['base_revision'] = args.base_revision
            result['base_checkpoint_files'] = {str(p.relative_to(base)): digest(p) for p in Path(base).rglob('*') if p.is_file()}
            result['base_download_and_hash_wall_s'] = time.perf_counter() - started
        result['gpu_load_before_model_load_percent'] = gpu_load()
        if max(result['gpu_load_before_model_load_percent'], default=0) > args.max_gpu_util:
            raise RuntimeError('GPU became busy during preparation; defer timed inference')
        started = time.perf_counter()
        print(f'Loading {args.strategy} variant {result["variant"]}.', flush=True)
        if args.strategy == 'openfly':
            sys.path.insert(0, str(args.upstream / 'train'))
            from extern.hf.configuration_prismatic import OpenFlyConfig
            from extern.hf.modeling_prismatic import OpenVLAForActionPrediction
            from extern.hf.processing_prismatic import PrismaticImageProcessor, PrismaticProcessor
            AutoConfig.register('openvla', OpenFlyConfig)
            AutoImageProcessor.register(OpenFlyConfig, PrismaticImageProcessor)
            AutoProcessor.register(OpenFlyConfig, PrismaticProcessor)
            AutoModelForVision2Seq.register(OpenFlyConfig, OpenVLAForActionPrediction)
            # The checkpoint supplies a valid fast tokenizer.json; forcing
            # use_fast=False selects its unusable tiny tokenizer.model instead.
            processor = AutoProcessor.from_pretrained(checkpoint)
            result['tokenizer_is_fast'] = bool(getattr(processor.tokenizer, 'is_fast', False))
            # The reviewed OpenFly constructor downloads pretrained DINO/SigLIP
            # again. Full checkpoint vision tensors replace those initializers.
            # Suppress only that redundant initialization, then require complete
            # checkpoint coverage so no random/default tensor becomes a result.
            import timm
            original_create = timm.create_model
            def checkpoint_vision(*values, **options):
                options['pretrained'] = False
                return original_create(*values, **options)
            timm.create_model = checkpoint_vision
            loading_kwargs = {}
            if precision == 'nf4':
                from transformers import BitsAndBytesConfig
                result['quantization'] = {'method': 'bitsandbytes NF4', 'double_quant': True,
                    'compute_dtype': 'bfloat16', 'skip_modules': ['vision_backbone', 'projector', 'lm_head']}
                loading_kwargs.update(quantization_config=BitsAndBytesConfig(
                    load_in_4bit=True, bnb_4bit_quant_type='nf4', bnb_4bit_use_double_quant=True,
                    bnb_4bit_compute_dtype=torch.bfloat16,
                    llm_int8_skip_modules=['vision_backbone', 'projector', 'lm_head']), device_map={'': 0})
            try:
                model, loading = AutoModelForVision2Seq.from_pretrained(
                    checkpoint, torch_dtype=torch.bfloat16, low_cpu_mem_usage=True,
                    attn_implementation='sdpa', output_loading_info=True, **loading_kwargs)
            finally:
                timm.create_model = original_create
            result['checkpoint_loading_info'] = loading
            result['redundant_vision_pretraining_download_suppressed'] = True
            if any(loading.get(key) for key in ('missing_keys', 'unexpected_keys', 'mismatched_keys', 'error_msgs')):
                raise RuntimeError('checkpoint does not fully cover constructed model; refusing initialized substitutes')
            model = model.eval() if precision == 'nf4' else model.to('cuda').eval()
            result['hf_device_map'] = getattr(model, 'hf_device_map', None)
            if precision == 'nf4':
                import bitsandbytes as bnb
                quantized_names = [name for name, module in model.named_modules()
                                   if isinstance(module, bnb.nn.Linear4bit)]
                result['quantized_linear_layers'] = len(quantized_names)
                result['quantized_layer_name_examples'] = quantized_names[:3]
                if not quantized_names or any(not name.startswith('language_model.') for name in quantized_names):
                    raise RuntimeError('actual quantized module set differs from declared language-only variant')
            images = [Image.open(f['path']).convert('RGB') for f in data['front_history']]
            def predict():
                inputs = processor(data['instruction'], images).to('cuda', dtype=torch.bfloat16)
                action = model.predict_action(**inputs, unnorm_key=args.unnorm_key, do_sample=False)
                if not all(math.isfinite(float(x)) for x in action):
                    raise RuntimeError('model returned nonfinite action')
                return {'raw_unnormalized_action': action.tolist(), 'unnorm_key': args.unnorm_key,
                        'flight_action_mapping_validated': False}
        else:
            from peft import PeftModel
            tokenizer = AutoTokenizer.from_pretrained(base, trust_remote_code=True)
            processor = AutoImageProcessor.from_pretrained(base, trust_remote_code=True, use_fast=False)
            model = AutoModelForVision2Seq.from_pretrained(base, torch_dtype=torch.bfloat16,
                       low_cpu_mem_usage=True, trust_remote_code=True, attn_implementation='sdpa')
            model.resize_token_embeddings(len(tokenizer))
            model = PeftModel.from_pretrained(model, str(Path(checkpoint) / 'aero_vla')).to('cuda').eval()
            front = Image.open(data['front_history'][-1]['path']).convert('RGB').resize((224,224), Image.Resampling.BICUBIC)
            down = Image.open(data['down']['path']).convert('RGB').resize((224,224), Image.Resampling.BICUBIC)
            mosaic = Image.new('RGB', (224,448)); mosaic.paste(front,(0,0)); mosaic.paste(down,(0,224))
            def predict():
                prompt = '<image>\n' + data['instruction'].strip() + '\nAction: '
                inputs = tokenizer([prompt], return_tensors='pt', padding=True).to('cuda')
                inputs['pixel_values'] = processor(images=mosaic, return_tensors='pt')['pixel_values'].to('cuda', dtype=torch.bfloat16)
                output = model.generate(**inputs, max_new_tokens=20, do_sample=False,
                                        eos_token_id=[tokenizer.eos_token_id])
                text = tokenizer.decode(output[0], skip_special_tokens=False)
                return {'text': text, **decode_aerovla(text), 'flight_action_mapping_validated': False}
        torch.cuda.synchronize()
        result['model_load_wall_s'] = time.perf_counter() - started
        result['model_load_torch_cuda_peak_allocated_bytes'] = torch.cuda.max_memory_allocated()
        result['model_load_torch_cuda_peak_reserved_bytes'] = torch.cuda.max_memory_reserved()
        result['model_loaded_torch_cuda_allocated_bytes'] = torch.cuda.memory_allocated()
        torch.cuda.reset_peak_memory_stats()
        outputs, latencies = [], []
        result['samples'] = outputs
        with torch.inference_mode():
            for index in range(args.warmup + args.repetitions):
                torch.cuda.synchronize(); started = time.perf_counter()
                prediction = predict()
                torch.cuda.synchronize(); elapsed = time.perf_counter() - started
                outputs.append({'warmup': index < args.warmup, 'wall_s': elapsed, 'prediction': prediction})
                print(f'Inference sample {index}: {elapsed:.6f} wall seconds; warmup={index < args.warmup}.', flush=True)
                if index >= args.warmup: latencies.append(elapsed)
        result.update(status='offline_inference_completed_not_flown', samples=outputs,
                      latency_wall_s={'median': statistics.median(latencies), 'min': min(latencies), 'max': max(latencies)},
                      peak_allocated_bytes=torch.cuda.max_memory_allocated(), peak_reserved_bytes=torch.cuda.max_memory_reserved())
    except Exception as exc:
        result.update(status='failed_not_flown', error=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        (args.output / 'result.json').write_text(json.dumps(result, indent=2, allow_nan=False,
            default=metadata_json_default), encoding='utf-8')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--strategy', choices=['openfly','aerovla'], required=True)
    parser.add_argument('--precision', choices=['bf16', 'nf4'], default='bf16')
    parser.add_argument('--observations', type=Path, required=True)
    parser.add_argument('--upstream', type=Path, required=True)
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--local-checkpoint', type=Path,
                        help='Use separately staged reviewed weights; hashes are still recorded')
    parser.add_argument('--hash-cache', type=Path,
                        help='Optional cache of full hashes with file identity/size/timestamp validation')
    parser.add_argument('--revision', required=True)
    parser.add_argument('--base-revision')
    parser.add_argument('--local-base', type=Path)
    parser.add_argument('--cache', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--unnorm-key', default='vlnv1')
    parser.add_argument('--warmup', type=int, default=1)
    parser.add_argument('--repetitions', type=int, default=3)
    parser.add_argument('--max-gpu-util', type=int, default=20)
    args = parser.parse_args()
    if args.warmup < 0 or args.repetitions < 1: parser.error('invalid repeat counts')
    run(args)
