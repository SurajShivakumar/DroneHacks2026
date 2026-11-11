"""Verify actual PX4 initial ULog parameters against a reviewed physical model."""
import argparse
from hashlib import sha256
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ulog',type=Path,required=True)
    parser.add_argument('--contract',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    from pyulog import ULog
    data=json.loads(args.contract.read_text())
    actual=ULog(str(args.ulog),message_name_filter_list=[]).initial_parameters
    wanted=dict(data['px4_parameters'])
    if 'px4_autostart_id' in data:
        wanted['SYS_AUTOSTART']=data['px4_autostart_id']
    differences={key:{'expected':value,'actual':actual.get(key)} for key,value in wanted.items()
                 if actual.get(key) is None or abs(actual[key]-value)>1e-6}
    result={'kind':'physical_airframe_parameter_audit_not_flight_qualification',
            'ulog_sha256':sha256(args.ulog.read_bytes()).hexdigest(),
            'contract_sha256':sha256(args.contract.read_bytes()).hexdigest(),
            'matches':not differences,'differences':differences,'actual':{key:actual.get(key) for key in wanted}}
    with args.output.open('x',encoding='utf-8') as output:
        json.dump(result,output,indent=2)
    print(json.dumps({'matches':not differences,'differences':differences},indent=2))
    return 1 if differences else 0


if __name__=='__main__':
    raise SystemExit(main())
