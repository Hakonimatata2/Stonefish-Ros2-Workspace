import math
from dataclasses import is_dataclass, asdict
from pathlib import Path
from typing import Iterable

import pandas as pd
from rosbags.highlevel import AnyReader
from rosbags.typesys.store import Typestore
from rosbags.typesys import get_types_from_msg, get_types_from_idl


# --------- util: flatten av vilkårlige ROS-meldinger til flat dict --------- #
def _flatten(obj, prefix='', out=None, max_list=16):
    if out is None:
        out = {}
    key = (lambda name: f'{prefix}.{name}') if prefix else (lambda name: name)

    if is_dataclass(obj):
        return _flatten(asdict(obj), prefix, out, max_list)

    if isinstance(obj, dict):
        for k, v in obj.items():
            _flatten(v, key(k), out, max_list)
        return out

    if isinstance(obj, (list, tuple)):
        n = min(len(obj), max_list)
        for i in range(n):
            _flatten(obj[i], key(str(i)), out, max_list)
        if len(obj) > n:
            out[key('_truncated')] = len(obj) - n
        return out

    if isinstance(obj, (int, float, bool, str)):
        # unngå NaN/Inf i CSV/DF
        if isinstance(obj, float) and (math.isnan(obj) or math.isinf(obj)):
            out[prefix] = ''
        else:
            out[prefix] = obj
        return out

    # ukjente/bytes/None osv: legg inn tom streng eller bytes-len
    if isinstance(obj, (bytes, bytearray)):
        out[prefix] = len(obj)  # lagrer lengde i stedet for råbytes
        return out

    return out


# --------- type-registrering (egne msg/idl + avhengigheter) ---------------- #
def _infer_pkg_from_msgdir(msg_dir: Path) -> str | None:
    return msg_dir.parent.name if msg_dir.name == 'msg' and msg_dir.parent else None

def _register_all_types(typestore: Typestore, msg_dir: Path, pkg_name: str | None = None) -> int:
    if not msg_dir or not msg_dir.exists():
        return 0
    if not pkg_name:
        pkg_name = _infer_pkg_from_msgdir(msg_dir)

    msg_files = sorted(msg_dir.glob('*.msg'))
    idl_files = sorted(msg_dir.glob('*.idl'))
    types_all, n = {}, 0

    for p in msg_files:
        typename = f'{pkg_name}/msg/{p.stem}' if pkg_name else None
        if not typename:
            continue
        tdict = get_types_from_msg(p.read_text(), typename)
        types_all.update(tdict); n += 1

    for p in idl_files:
        text = p.read_text()
        try:
            tdict = get_types_from_idl(text, f'{pkg_name}/msg/{p.stem}')
        except TypeError:
            tdict = get_types_from_idl(text)
        types_all.update(tdict); n += 1

    if types_all:
        typestore.register(types_all)
    return n

def _register_deps(typestore: Typestore, dep_pkgs: list[str]) -> int:
    if not dep_pkgs:
        return 0
    try:
        from ament_index_python.packages import get_package_share_directory
    except Exception:
        return 0
    total = 0
    for pkg in dep_pkgs:
        try:
            msg_dir = Path(get_package_share_directory(pkg)) / 'msg'
            total += _register_all_types(typestore, msg_dir, pkg)
        except Exception:
            pass
    return total


# def _build_typestore(msg_dirs: Iterable[Path] | None = None,
#                      pkg_names: Iterable[str] | None = None,
#                      dep_pkgs: Iterable[str] | None = None) -> Typestore:
#     ts = Typestore()
#     _register_deps(ts, list(dep_pkgs or []))
#     msg_dirs = list(msg_dirs or [])
#     pkg_names = list(pkg_names or [])
#     for i, d in enumerate(msg_dirs):
#         _register_all_types(ts, d, pkg_names[i] if i < len(pkg_names) else None)
#     return ts

# --- Riktig typestore for rosbags ≥0.10 ---
try:
    from rosbags.typesys.stores import Stores, get_typestore
    def make_typestore():
        return get_typestore(Stores.ROS2_HUMBLE)
except ImportError:
    # fallback for eldre rosbags
    from rosbags.typesys.store import Typestore
    def make_typestore():
        return Typestore()
    
def _build_typestore(msg_dirs: Iterable[Path] | None = None,
                     pkg_names: Iterable[str] | None = None,
                     dep_pkgs: Iterable[str] | None = None):
    ts = make_typestore()
    _register_deps(ts, list(dep_pkgs or []))
    msg_dirs = list(msg_dirs or [])
    pkg_names = list(pkg_names or [])
    for i, d in enumerate(msg_dirs):
        _register_all_types(ts, d, pkg_names[i] if i < len(pkg_names) else None)
    return ts



from rosbags.typesys import get_types_from_msg

def ensure_minimal_core_types(typestore):
    """Registrer et minimalt sett av ROS-typer dersom de ikke allerede finnes."""
    needed = [
        "builtin_interfaces/msg/Time",
        "std_msgs/msg/Header",
        "sensor_msgs/msg/FluidPressure",
    ]
    missing = [t for t in needed if t not in typestore.fielddefs]

    if not missing:
        return

    types = {}

    if "builtin_interfaces/msg/Time" in missing:
        types.update(get_types_from_msg(
            "int32 sec\nuint32 nanosec\n",
            "builtin_interfaces/msg/Time",
        ))

    if "std_msgs/msg/Header" in missing:
        types.update(get_types_from_msg(
            "builtin_interfaces/Time stamp\nstring frame_id\n",
            "std_msgs/msg/Header",
        ))

    if "sensor_msgs/msg/FluidPressure" in missing:
        types.update(get_types_from_msg(
            "std_msgs/Header header\nfloat64 fluid_pressure\nfloat64 variance\n",
            "sensor_msgs/msg/FluidPressure",
        ))

    if types:
        typestore.register(types)




# ---------------------- hovedfunksjon: bag -> DataFrame --------------------- #
def bag_topic_to_dataframe(
    bag_path: str | Path,
    topic: str,
    time_unit: str = 's',          # 'ns' eller 's'
    max_msgs: int = 0,              # 0 = alle
    msg_dirs: Iterable[Path] | None = [Path("/root/stonefish_ros2_ws/src/stonefish_ros2/msg")],
    pkg_names: Iterable[str] | None = None,
    dep_pkgs: Iterable[str] | None = ["std_msgs", "sensor_msgs", "geometry_msgs", "builtin_interfaces", "nav_msgs"],
) -> pd.DataFrame:
    """
    Leser et topic fra en ROS1/ROS2-bag (støttet av rosbags AnyReader) og returnerer en flat DataFrame.

    Kolonnen 't' er tidsstempel (int ns eller float s). Øvrige felt er flate nøkler.
    Råbytes lagres som lengde (int) for å unngå enorme celler.

    Eksempel:
        df = bag_topic_to_dataframe('path/to/bag', '/topic',sensor_msgs/msg/LaserScan
                                    time_unit='s',
                                    msg_dirs=[Path('/root/.../src/pkg/msg')],
                                    dep_pkgs=['std_msgs','sensor_msgs'])
    """
    bagpath = Path(bag_path)
    if not bagpath.exists():
        raise FileNotFoundError(f"Fant ikke bag: {bagpath}")

    typestore = _build_typestore(msg_dirs, pkg_names, dep_pkgs)
    ensure_minimal_core_types(typestore)

    rows, all_keys = [], set()
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            available = ', '.join(sorted({c.topic for c in reader.connections}))
            raise ValueError(f"Fant ikke topic: {topic}. Tilgjengelige: {available}")

        count = 0
        for conn, ts, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            row = {'t': ts if time_unit == 'ns' else ts / 1e9}
            data = {}
            _flatten(msg, out=data)
            row.update(data)

            rows.append(row)
            all_keys.update(row.keys())

            count += 1
            if max_msgs and count >= max_msgs:
                break

    if not rows:
        return pd.DataFrame(columns=['t'])

    cols = ['t'] + sorted(k for k in all_keys if k != 't')
    df = pd.DataFrame([{k: r.get(k, '') for k in cols} for r in rows], columns=cols)

    # prøv å gjøre numeriske kolonner numeriske
    for c in cols:
        if c == 't':
            continue
        df[c] = pd.to_numeric(df[c], errors='ignore')

    return df