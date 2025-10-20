#!/usr/bin/env python3
import argparse, csv, math, sys
from pathlib import Path
from dataclasses import is_dataclass, asdict

from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg, get_types_from_idl

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

    # ukjente typer droppes stille
    return out

# ------------------------ Type registration -------------------------------- #

def infer_pkg_name_from_msg_dir(msg_dir: Path) -> str | None:
    """
    Støtter både .../src/<pakke>/msg og .../share/<pakke>/msg
    """
    try:
        if msg_dir.name != 'msg':
            return None
        parent = msg_dir.parent
        if parent and parent.name:
            # .../<pakke>/msg
            return parent.name
    except Exception:
        pass
    return None

def register_all_types_in_dir(reader: AnyReader, msg_dir: Path, pkg_name: str | None = None) -> int:
    """
    Leser alle .msg og .idl i msg_dir og registrerer dem i typestore.
    """
    if not msg_dir.exists() or not msg_dir.is_dir():
        print(f"[type-reg] Advarsel: msg-dir finnes ikke: {msg_dir}", file=sys.stderr)
        return 0

    if not pkg_name:
        pkg_name = infer_pkg_name_from_msg_dir(msg_dir)
    if not pkg_name:
        print(f"[type-reg] Klarte ikke å utlede pakkenavn fra {msg_dir}. "
              f"Oppgi gjerne --pkg-name.", file=sys.stderr)

    msg_files = sorted(msg_dir.glob("*.msg"))
    idl_files = sorted(msg_dir.glob("*.idl"))

    if not msg_files and not idl_files:
        print(f"[type-reg] Fant ingen .msg eller .idl i {msg_dir}", file=sys.stderr)
        return 0

    types_all = {}
    registered = 0

    # .msg
    for p in msg_files:
        typename = f"{pkg_name}/msg/{p.stem}" if pkg_name else None
        try:
            text = p.read_text()
            if not typename:
                print(f"[type-reg] Hopper over (mangler pakkenavn): {p.name}", file=sys.stderr)
                continue
            tdict = get_types_from_msg(text, typename)
            types_all.update(tdict)
            registered += 1
        except Exception as e:
            print(f"[type-reg] Feil ved parsing av {p.name}: {e}", file=sys.stderr)

    # .idl
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
        reader.typestore.register(types_all)
        print(f"[type-reg] Registrerte {registered} typer fra {msg_dir} "
              f"(totalt {len(types_all)} entries).")
    else:
        print("[type-reg] Ingen typer ble registrert.", file=sys.stderr)

    return registered

def register_dep_packages(reader: AnyReader, dep_pkgs: list[str]) -> int:
    """
    Registrer alle .msg/.idl for oppgitte avhengighetspakker via ament_index.
    Eks: std_msgs, sensor_msgs, geometry_msgs, builtin_interfaces
    """
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
            total += register_all_types_in_dir(reader, msg_dir, pkg)
        except Exception as e:
            print(f"[type-reg] Fant ikke share-dir for {pkg}: {e}", file=sys.stderr)
    return total

# ------------------------------ Main --------------------------------------- #

def main():
    ap = argparse.ArgumentParser(description="Eksporter et ROS2 topic til CSV fra rosbag2.")
    ap.add_argument('--bag', required=True, help='Sti til bag-mappe (metadata.yaml + .db3/.mcap)')
    ap.add_argument('--topic', required=True, help='Topic som skal eksporteres (eks /dvl)')
    ap.add_argument('--out', required=True, help='Utfil (CSV)')
    ap.add_argument('--time-unit', choices=['ns', 's'], default='ns', help='Tidsstempel i ns eller sekunder')
    ap.add_argument('--max', type=int, default=0, help='Maks antall meldinger (0=alle)')

    # Nytt: flere msg-dir + dep-pakker
    ap.add_argument('--msg-dir', type=Path, action='append', default=[],
                    help='Mappe med .msg/.idl (kan oppgis flere ganger). Eks: .../src/<pkg>/msg')
    ap.add_argument('--pkg-name', type=str, action='append', default=[],
                    help='Pakkenavn for hver --msg-dir i samme rekkefølge (valgfritt).')
    ap.add_argument('--dep-pkg', dest='dep_pkgs', action='append', default=[],
                    help='Avhengighetspakke som skal registreres via ament_index (kan gjentas).')

    args = ap.parse_args()

    bagpath = Path(args.bag)
    if not bagpath.exists():
        raise SystemExit(f"Fant ikke bag: {bagpath}")

    with AnyReader([bagpath]) as reader:
        # Registrer avhengigheter først (std_msgs osv.)
        if args.dep_pkgs:
            register_dep_packages(reader, args.dep_pkgs)

        # Registrer egne/andre msg-mapper
        if args.msg_dir:
            for i, d in enumerate(args.msg_dir):
                pkg = None
                if i < len(args.pkg_name):
                    pkg = args.pkg_name[i]
                register_all_types_in_dir(reader, d, pkg)

        # Filtrer connections for valgt topic
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
