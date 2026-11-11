#!/usr/bin/env bash
set -euo pipefail
"$HOME/dronehacks-models-local/venv/bin/python" - <<'PY'
from pathlib import Path
import sys,json
from PIL import Image
root=Path.home()/'dronehacks-models-local'
sys.path.insert(0,str(root/'OpenFly-Platform/train'))
from extern.hf.configuration_prismatic import OpenFlyConfig
from extern.hf.processing_prismatic import PrismaticImageProcessor,PrismaticProcessor
from transformers import AutoConfig,AutoImageProcessor,AutoProcessor
AutoConfig.register('openvla',OpenFlyConfig)
AutoImageProcessor.register(OpenFlyConfig,PrismaticImageProcessor)
AutoProcessor.register(OpenFlyConfig,PrismaticProcessor)
processor=AutoProcessor.from_pretrained(root/'checkpoints/openfly-21dcce')
data=json.loads((root/'observations-course-rig-v1.json').read_text())
inputs=processor(data['instruction'],[Image.open(f['path']).convert('RGB') for f in data['front_history']])
result={'kind':'processor_only_not_model_inference','tokenizer_is_fast':processor.tokenizer.is_fast,
        'input_shapes':{k:list(v.shape) for k,v in inputs.items()},
        'last_input_token_id':int(inputs['input_ids'][0,-1])}
assert result['tokenizer_is_fast']
(root/'processor-default-fast-v1.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
PY
