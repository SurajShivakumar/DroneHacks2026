"""Resumable public pinned-weight download with exact range and SHA256 checks."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from hashlib import sha256
import json
from pathlib import Path
import threading
import time
import urllib.request


def digest(path):
    h = sha256()
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(8*1024*1024), b''):
            h.update(data)
    return h.hexdigest()


def download(manifest, output, workers=4, chunk_bytes=32*1024*1024):
    output.mkdir(parents=True, exist_ok=True)
    lock = threading.Lock()
    for item in manifest['files']:
        name = item['name']
        if Path(name).name != name or item['bytes'] <= 0 or len(item['lfs_sha256']) != 64:
            raise ValueError('Expected flat filenames, sizes and SHA256 from pinned source metadata')
        target = output / name
        if target.exists():
            if target.stat().st_size != item['bytes'] or digest(target) != item['lfs_sha256']:
                raise ValueError('Existing checkpoint differs; refusing replacement')
            continue
        part = output / (name + '.part')
        state_path = output / (name + '.ranges.json')
        signature = {'sha256': item['lfs_sha256'], 'bytes': item['bytes'], 'chunk_bytes': chunk_bytes}
        if part.exists():
            state = json.loads(state_path.read_text())
            if any(state[k] != v for k, v in signature.items()) or part.stat().st_size != item['bytes']:
                raise ValueError('Partial download metadata differs')
        else:
            state = dict(signature, completed=[])
            with part.open('xb') as stream:
                stream.truncate(item['bytes'])
            state_path.write_text(json.dumps(state))
        done = set(state['completed'])

        def fetch(index):
            start = index * chunk_bytes
            end = min(item['bytes'], start + chunk_bytes) - 1
            for attempt in range(4):
                try:
                    request = urllib.request.Request(item['url'] + f'?download=true&range_start={start}',
                                                      headers={'Range': f'bytes={start}-{end}'})
                    with urllib.request.urlopen(request, timeout=45) as response:
                        expected = f'bytes {start}-{end}/{item["bytes"]}'
                        if response.status != 206 or response.headers.get('Content-Range') != expected:
                            raise ValueError('Server did not honor exact requested range')
                        payload = response.read(end-start+2)
                    if len(payload) != end-start+1:
                        raise ValueError('Truncated or oversized range')
                    with part.open('r+b') as stream:
                        stream.seek(start)
                        stream.write(payload)
                    with lock:
                        done.add(index)
                        state['completed'] = sorted(done)
                        state_path.write_text(json.dumps(state))
                        if len(done) % 16 == 0:
                            print(name, len(done), '/', (item['bytes']+chunk_bytes-1)//chunk_bytes, flush=True)
                    return
                except Exception:
                    if attempt == 3:
                        raise
                    time.sleep(attempt+1)
        pending = [i for i in range((item['bytes']+chunk_bytes-1)//chunk_bytes) if i not in done]
        with ThreadPoolExecutor(max_workers=workers) as pool:
            list(pool.map(fetch, pending))
        if digest(part) != item['lfs_sha256']:
            raise ValueError('Final SHA256 mismatch; partial evidence retained')
        part.rename(target)
        print('Verified', name, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--workers', type=int, default=4)
    args = parser.parse_args()
    if not 1 <= args.workers <= 8:
        parser.error('workers must be between1 and8')
    download(json.loads(args.manifest.read_text()), args.output, args.workers)
