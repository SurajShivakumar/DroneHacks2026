#!/usr/bin/env bash
set -euo pipefail
HTTPS_PROXY=http://127.0.0.1:18080 python3 - <<'PY'
import urllib.request,time,json
url='https://huggingface.co/IPEC-COMMUNITY/openfly-agent-7b/resolve/21dcce235f1f2d40fc23c76998051abc5434cc99/model-00001-of-00003.safetensors'
start=time.monotonic()
request=urllib.request.Request(url,headers={'Range':'bytes=16777216-25165823'})
with urllib.request.urlopen(request,timeout=45) as response:
    data=response.read(8388608)
    result={'status':response.status,'range':response.headers.get('Content-Range'),'bytes':len(data),'wall_s':time.monotonic()-start}
result['mib_per_s']=len(data)/1048576/result['wall_s']
print(json.dumps(result))
PY
