# bag2csv.py
import argparse, csv, math
from pathlib import Path
from dataclasses import is_dataclass, asdict
from rosbags.highlevel import AnyReader

def flatten(obj, prefix='', out=None, max_list=16):
    if out is None:
        out = {}
    key = lambda name: f'{prefix}.{name}' if prefix else name

    if is_dataclass(obj):
        d = asdict(obj)
        return flatten(d, prefix, out, max_list)

    if isinstance(obj, dict):
        for k, v in obj.items():
            flatten(v, key(k), out, max_list)
        return out

    if isinstance(obj, (list, tuple)):
        n = min(len(obj), max_list)
        for i in range(n):
            flatten(obj[i], key(str(i)), out, max_list)
        if len(obj) > n:
            out[key('_truncated')] = len(obj) - n
        return out

    if isinstance(obj, (int, float, bool, str)):
        if isinstance(obj, float) and (math.isnan(obj) or math.isinf(obj)):
            out[prefix] = ''
        else:
            out[prefix] = obj
        return out

    return out  # skip unknowns

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True, help='Path to bag folder (contains .db3 + metadata)')
    ap.add_argument('--topic', required=True, help='Topic to export')
    ap.add_argument('--out', required=True, help='Output CSV file')
    ap.add_argument('--time-unit', choices=['ns', 's'], default='ns', help='Timestamp unit')
    ap.add_argument('--max', type=int, default=0, help='Max messages (0=all)')
    args = ap.parse_args()

    with AnyReader([Path(args.bag)]) as reader:
        conns = [c for c in reader.connections if c.topic == args.topic]
        if not conns:
            raise SystemExit(f'Fant ikke topic: {args.topic}')

        keys = set()
        rows = []
        count = 0
        for conn, ts, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            row = {'t': ts if args.time_unit == 'ns' else ts / 1e9}
            data = {}
            flatten(msg, out=data)
            row.update(data)
            rows.append(row)
            keys.update(row.keys())
            count += 1
            if args.max and count >= args.max:
                break

        keys = ['t'] + sorted(k for k in keys if k != 't')
        with open(args.out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for r in rows:
                w.writerow({k: r.get(k, '') for k in keys})

        print(f'Skrev {len(rows)} rader til {args.out}')
        print('Kolonner:', ', '.join(keys[:10]), '...' if len(keys) > 10 else '')


if __name__ == '__main__':
    main()
