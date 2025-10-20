#!/usr/bin/env python3
import argparse, csv, math, sys
from pathlib import Path
from dataclasses import is_dataclass, asdict

from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg, get_types_from_idl

from rosbags.typesys.store import Typestore


# ----------------------------- Utilities ---------------------------------- #

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

    return out  # ukjente typer droppes stille


# ------------------------ Type registration -------------------------------- #

def infer_pkg_name_from_msg_dir(msg_dir: Path) -> str | None:
    if msg_dir.name != 'msg':
        return None
    parent = msg_dir.parent
    return parent.name if parent and parent.name else None


def register_all_types_in_dir_into(typestore: Typestore, msg_dir: Path, pkg_name: str | None = None) -> int:
    if not msg_dir.exists() or not msg_dir.is_dir():
        print(f"[type-reg] Advarsel: msg-dir finnes ikke: {msg_dir}", file=sys.stderr)
        return 0

    if not pkg_name:
        pkg_name = infer_pkg_name_from_msg_dir(msg_dir)
    if not pkg_name:
        print(f"[type-reg] Klarte ikke å utlede pakkenavn fra {msg_dir}. Oppgi --pkg-name ved behov.",
              file=sys.stderr)

    msg_files = sorted(msg_dir.glob("*.msg"))
    idl_files = sorted(msg_dir.glob("*.idl"))

    if not msg_files and not idl_files:
        print(f"[type-reg] Fant ingen .msg/.idl i {msg_dir}", file=sys.stderr)
        return 0

    types_all = {}
    registered = 0

    for p in msg_files:
        typename = f"{pkg_name}/msg/{p.stem}" if pkg_name else None
        try:
            text = p.read_text()
            if not typename:
                print(f"[type-reg] Hopper .msg (mangler pakkenavn): {p.name}", file=sys.stderr)
                continue
            tdict = get_types_from_msg(text, typename)
            types_all.update(tdict)
            registered += 1
        except Exception as e:
            print(f"[type-reg] Feil ved parsing av {p.name}: {e}", file=sys.stderr)

    for p in idl_files:
        typename = f"{pkg_name}/msg/{p.stem}" if pkg_name else None
        try:
            text = p.read_text()
            tdict = get_types_from_idl(text, typename)
            types_all.update(tdict)
            registered += 1
        except Exception as e:
            print(f"[type-reg] Feil ved parsing av {p.name}: {e}", file=sys.stderr)

    if types_all:
        typestore.register(types_all)
        print(f"[type-reg] Registrerte {registered} topp-typer fra {msg_dir} (sum {len(types_all)} typedefs).")
    else:
        print("[type-reg] Ingen typedefs ble registrert.", file=sys.stderr)

    return registered


def register_dep_packages_into(typestore: Typestore, dep_pkgs: list[str]) -> int:
    if not dep_pkgs:
        return 0
    try:
        from ament_index_python.packages import get_package_share_directory
    except Exception as e:
        print(f"[type-reg] ament_index ikke tilgjengelig: {e}", file=sys.stderr)
        return 0

    total = 0
    for pkg in dep_pkgs:
        try:
            share = Path(get_package_share_directory(pkg))
            msg_dir = share / 'msg'
            total += register_all_types_in_dir_into(typestore, msg_dir, pkg)
        except Exception as e:
            print(f"[type-reg] Fant ikke share-dir for {pkg}: {e}", file=sys.stderr)
    return total


def build_typestore(msg_dirs: list[Path], pkg_names: list[str], dep_pkgs: list[str]) -> Typestore:
    ts = Typestore()
    if dep_pkgs:
        register_dep_packages_into(ts, dep_pkgs)
    for i, d in enumerate(msg_dirs or []):
        pkg = pkg_names[i] if (i < len(pkg_names) and pkg_names[i]) else None
        register_all_types_in_dir_into(ts, d, pkg)
    return ts


# ------------------------------ Main --------------------------------------- #

def main():
    ap = argparse.ArgumentParser(description="Eksporter et ROS2 topic til CSV fra rosbag2.")
    ap.add_argument('--bag', required=True, help='Sti til bag-mappe (metadata.yaml + .db3/.mcap)')
    ap.add_argument('--topic', required=True, help='Topic som skal eksporteres (eks /dvl)')
    ap.add_argument('--out', required=True, help='Utfil (CSV)')
    ap.add_argument('--time-unit', choices=['ns', 's'], default='ns', help='Tidsstempel i ns eller sekunder')
    ap.add_argument('--max', type=int, default=0, help='Maks antall meldinger (0=alle)')

    ap.add_argument('--msg-dir', type=Path, action='append', default=[],
                    help='Mappe med .msg/.idl (kan oppgis flere ganger). Eks: .../src/<pkg>/msg')
    ap.add_argument('--pkg-name', type=str, action='append', default=[],
                    help='Pakkenavn for hver --msg-dir i samme rekkefølge (valgfritt).')
    ap.add_argument('--dep-pkg', dest='dep_pkgs', action='append', default=[],
                    help='Avhengighetspakke registrert via ament_index (kan gjentas).')

    args = ap.parse_args()

    bagpath = Path(args.bag)
    if not bagpath.exists():
        raise SystemExit(f"Fant ikke bag: {bagpath}")

    # Bygg typestore FØR AnyReader
    typestore = build_typestore(args.msg_dir, args.pkg_name, args.dep_pkgs)

    # Viktig: gi default_typestore til AnyReader
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections if c.topic == args.topic]
        if not conns:
            tilgjengelig = ', '.join(sorted({c.topic for c in reader.connections}))
            raise SystemExit(f"Fant ikke topic: {args.topic}\nTilgjengelige topics: {tilgjengelig}")

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

        outpath = Path(args.out)
        outpath.parent.mkdir(parents=True, exist_ok=True)
        with outpath.open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for r in rows:
                w.writerow({k: r.get(k, '') for k in keys})

        print(f"Skrev {len(rows)} rader til {outpath}")
        preview = ', '.join(keys[:10]) + (' ...' if len(keys) > 10 else '')
        print('Kolonner:', preview)


if __name__ == '__main__':
    main()
