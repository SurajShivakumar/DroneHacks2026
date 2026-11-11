#!/usr/bin/env bash
# Finish already completed public-weight transfers only after full SHA256 checks.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
from hashlib import sha256
import json
root=Path.home()/'dronehacks-models/checkpoints/openfly-21dcce'
expected={
 'model-00001-of-00003.safetensors':(6948961960,'a40777127f28bb99a18107c8145057b9ca3762f8e65da5ed67a3bb8bd84b171d'),
 'model-00002-of-00003.safetensors':(6971232040,'e6b9e972ba51d6920403670dd54e0b305f34315a492ce874120792bbd7a8c7a3'),
 'model-00003-of-00003.safetensors':(1162406824,'4a33348f85fc6c25d5f4fef6abb4000593bb2bb5d9b99cfdd9d8869bb04e43c1')}
records=[]
for name,(size,wanted) in expected.items():
 final=root/name
 source=final if final.exists() else root/(name+'.upload-v1')
 # The third shard was previously finalized and verified; still hash it again.
 if source.stat().st_size!=size:
  raise RuntimeError('Incomplete transfer: '+name)
 digest=sha256()
 with source.open('rb') as stream:
  for chunk in iter(lambda:stream.read(8*1024*1024),b''):digest.update(chunk)
 if digest.hexdigest()!=wanted:raise RuntimeError('Full hash mismatch: '+name)
 records.append({'name':name,'size':source.stat().st_size,'sha256':wanted})
 if source!=final:
  if final.exists():raise RuntimeError('Final destination appeared; preserve both files')
  source.rename(final)
 print('Verified '+name,flush=True)
output=Path.home()/'dronehacks-2026-gb10/artifacts/model-preparation/gb10-openfly-verified-v1.json'
output.parent.mkdir(parents=True,exist_ok=True)
with output.open('x') as stream:json.dump({'kind':'full_public_checkpoint_hash_verification_not_inference','files':records},stream,indent=2)
PY
