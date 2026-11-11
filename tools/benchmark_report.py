"""Generate an exclusive benchmark report from measured JSON records.

Input is {"schema_version": 1, "runs": [...]}; paths are relative to --root.
No simulation, model or flight is started. Original input/failures stay untouched.
"""
import argparse
import json
from pathlib import Path
import shutil
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dronebench.benchmark_results import markdown_report, strategy_summary, validate_records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--root', type=Path, default=Path.cwd())
    args = parser.parse_args()
    data = json.loads(args.input.read_text(encoding='utf-8'))
    if data.get('schema_version') != 1 or not isinstance(data.get('runs'), list):
        raise ValueError('expected schema_version 1 and runs list')
    records = validate_records(data['runs'], args.root)
    args.output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(args.input, args.output / 'input.json')
    (args.output / 'validated.json').write_text(json.dumps({'schema_version': 1, 'runs': records,
                                                         'summary': strategy_summary(records)}, indent=2), encoding='utf-8')
    # Links stay relative to the evidence root, including when output is elsewhere.
    import os
    link_records = json.loads(json.dumps(records))
    for row in link_records:
        for item in row['videos'] + row['provenance']:
            item['path'] = Path(os.path.relpath((args.root / item['path']).resolve(), args.output.resolve())).as_posix()
    (args.output / 'report.md').write_text(markdown_report(link_records), encoding='utf-8')
    print(args.output / 'report.md')


if __name__ == '__main__':
    main()
