import numpy as np
from math import sin, cos, atan2, asin, pi
from pathlib import Path
import numpy as np, cv2
from rosbags.highlevel import AnyReader
import pandas as pd
from rosbags.typesys import Stores, get_typestore
import math


def Rx(r):
    return np.array([[1,0,0],[0,cos(r),-sin(r)],[0,sin(r),cos(r)]], dtype=float)

def Ry(p):
    return np.array([[cos(p),0,sin(p)],[0,1,0],[-sin(p),0,cos(p)]], dtype=float)

def Rz(y):
    return np.array([[cos(y),-sin(y),0],[sin(y),cos(y),0],[0,0,1]], dtype=float)

def rpy_to_R_zyx(roll, pitch, yaw):
    return Rz(yaw) @ Ry(pitch) @ Rx(roll)

def R_to_rpy_zyx(R):
    sp = -R[2,0]
    sp = np.clip(sp, -1.0, 1.0)
    pitch = asin(sp)

    if abs(abs(pitch) - pi/2) < 1e-6:
        roll = 0.0
        yaw = atan2(-R[0,1], R[1,1])
    else:
        roll = atan2(R[2,1], R[2,2])
        yaw  = atan2(R[1,0], R[0,0])
    return roll, pitch, yaw


def quaternion_to_euler(w, x, y, z):
    """
    Converts quanternion to rpy euler (radians)
    """

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def apply_T(T, p_xyz, rpy):
    """T: 4x4, p_xyz: (x,y,z), rpy: (r,p,y) -> (p_out, rpy_out)"""
    p = np.array(p_xyz, dtype=float).reshape(3,1)
    R_in = rpy_to_R_zyx(*rpy)
    R_T = T[:3,:3]
    t_T = T[:3, 3:4]

    p_out = (R_T @ p + t_T).reshape(3)
    R_out = R_T @ R_in
    r_out, p_out_ang, y_out = R_to_rpy_zyx(R_out)
    return p_out, (r_out, p_out_ang, y_out)


def invert_T(T):
    """
    Inverts a 4x4 homogenious transformation matrix.
    """
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv



def map_to_sector(img: np.ndarray, fov_deg):
    """
    Made in collaboration with ChatGPT
    """

    nbins, nbeams = img.shape
    fov = np.deg2rad(fov_deg)
    full360 = np.isclose(fov, 2*np.pi)
    
    centered = (fov_deg > 180.0)

    radius_px = int(nbins)

    if centered:
        H = W = 2 * radius_px
        ys, xs = np.indices((H, W), dtype=np.float32)
        cx = cy = float(radius_px)
        x = xs - cx
        y = ys - cy
    else:
        H = radius_px
        W = max(1, int(2 * radius_px * np.sin(fov / 2.0)))
        ys, xs = np.indices((H, W), dtype=np.float32)
        cx = W / 2.0
        x = xs - cx
        y = ys

    rr_bins = np.hypot(x, y)
    th      = np.arctan2(x, y)

    if full360:
        bear_min, bear_max = -np.pi, np.pi
    else:
        bear_min, bear_max = -fov/2.0, +fov/2.0

    valid = (rr_bins >= 0.0) & (rr_bins <= (nbins - 1))
    if not full360:
        valid &= (th >= bear_min) & (th <= bear_max)

    r_idx = rr_bins[valid].clip(0, nbins - 1)
    if full360:
        b_idx = ((th[valid] + np.pi) % (2*np.pi)) / (2*np.pi) * (nbeams - 1)
    else:
        denom = max(1e-9, (bear_max - bear_min))
        b_idx = (th[valid] - bear_min) / denom * (nbeams - 1)

    r_nn = np.rint(r_idx).astype(int)
    b_nn = np.rint(b_idx).astype(int).clip(0, nbeams - 1)

    out = np.zeros((ys.shape[0], xs.shape[1]), np.float32)
    out[valid] = img[r_nn, b_nn]
    return out

def get_sonoptix_data(bag, topic="/sensor/sonoptix_echo/image"):

    Sonoptix_Data = {
        'time': [],
        'image': [],
    }

    with AnyReader([Path(bag)]) as reader:

        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise SystemExit(f"Could not find topic {topic}. Available: " + ", ".join(sorted({c.topic for c in reader.connections})))
        
        start_ns = reader.start_time

        for connection, timestamp_ns, rawdata in reader.messages(connections=conns):
            t_rel = (timestamp_ns - start_ns) * 1e-9
            msg = reader.deserialize(rawdata, connection.msgtype)

            # Image height and width
            H = msg.array_data.layout.dim[0].size
            W = msg.array_data.layout.dim[1].size

            # Reshape sonar data to image
            raw_sonar_data = np.asarray(msg.array_data.data, np.float32)
            im = raw_sonar_data.reshape(H, W)

            # Save image data
            Sonoptix_Data["time"].append(t_rel)
            Sonoptix_Data["image"].append(im)


    return Sonoptix_Data


def get_ping360_data(bag, topic="/sensor/ping360"):

    Ping360_Data = {
        'time': [], 
        'image': []
    }

    with AnyReader([Path(bag)]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise SystemExit(f"Could not find topic {topic}. Available: " + ", ".join(sorted({c.topic for c in reader.connections})))

        start_ns = reader.start_time
        current_img = None

        for connection, timestamp_ns, rawdata in reader.messages(connections=conns):
            t_rel = (timestamp_ns - start_ns) * 1e-9
            msg = reader.deserialize(rawdata, connection.msgtype)

            H = int(msg.number_of_samples)
            step = msg.angle_step
            W = int(round(360.0 / step))

            if current_img is None or current_img.shape != (H, W):
                current_img = np.zeros((H, W), np.uint8)

            # Image column is data from the angle
            col = int(round((float(msg.angle_deg) % 360.0) / step)) % W

            # Get intensities
            data = np.asarray(msg.data)
            data = data[:H]

            current_img[:, col] = data[::]

            # Save
            Ping360_Data['time'].append(t_rel)
            Ping360_Data['image'].append(current_img.copy())
    
    return Ping360_Data



def get_simulated_image_data(bag, topic):

    data_dict = {
        'time': [],
        'image': []
    }

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    with AnyReader([Path(bag)], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise SystemExit(f"Could not find topic {topic}. Available: " + ", ".join(sorted({c.topic for c in reader.connections})))

        start_ns = reader.start_time

        for conn, t_ns, raw in reader.messages(connections=conns):
            t_rel = (t_ns - start_ns) * 1e-9
            msg = reader.deserialize(raw, conn.msgtype)

            H = msg.height
            W = msg.width

            data = np.asarray(msg.data, dtype=np.float32)
            im = data.reshape(H, W)

            data_dict['time'].append(t_rel)
            data_dict['image'].append(im)
    
    return data_dict


def get_compressed_color_image(bag, topic):

    data_dict = {
        'time': [],
        'image': []
    }

    with AnyReader([Path(bag)]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise SystemExit(f"Could not find topic {topic}. Available: " + ", ".join(sorted({c.topic for c in reader.connections})))

        start_ns = reader.start_time

        for conn, t_ns, raw in reader.messages(connections=conns):
            t_rel = (t_ns - start_ns) * 1e-9
            msg = reader.deserialize(raw, conn.msgtype)

            
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

            data_dict['time'].append(t_rel)
            data_dict['image'].append(rgb)
    
    return data_dict


def get_color_image_data(bag, topic):

    data_dict = {
        'time': [],
        'image': []
    }

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    with AnyReader([Path(bag)], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise SystemExit(f"Could not find topic {topic}. Available: " + ", ".join(sorted({c.topic for c in reader.connections})))

        start_ns = reader.start_time

        for conn, t_ns, raw in reader.messages(connections=conns):
            t_rel = (t_ns - start_ns) * 1e-9
            msg = reader.deserialize(raw, conn.msgtype)

            w, h = msg.width, msg.height
            data = np.frombuffer(msg.data, dtype=np.uint8)
            img = np.reshape(data, (h, w, 3))            

            data_dict['time'].append(t_rel)
            data_dict['image'].append(img)
    
    return data_dict


def get_image_nearest_time(data_dict, target_time):
 
    times = np.array(data_dict['time'])
    images = data_dict['image']

    idx = np.argmin(np.abs(times - target_time))
    return images[idx]

def compare_singals(t1, t2, data1, data2):
   
    # to np array
    t1 = np.asarray(t1, dtype=float)
    t2 = np.asarray(t2, dtype=float)
    x1 = np.asarray(data1, dtype=float)
    x2 = np.asarray(data2, dtype=float)

    # Get overlap in time (valid signal)
    t_start = max(t1.min(), t2.min())
    t_end   = min(t1.max(), t2.max())

    # Find timestep
    dt1 = np.median(np.diff(np.unique(t1)))
    dt2 = np.median(np.diff(np.unique(t2)))
    dt = float(min(dt1, dt2))

    # Create shared grid
    tg = np.arange(t_start, t_end, dt)
    if tg.size < 2:
        raise ValueError("Too small overlap")

    # Interpolate in shared grid
    y1 = np.interp(tg, t1, x1)
    y2 = np.interp(tg, t2, x2)

    # Set mean = 0
    y1 = y1 - np.mean(y1)
    y2 = y2 - np.mean(y2)

    # Cross corelate
    cross_corelation = np.correlate(y2, y1, mode='full')
    lags = np.arange(-len(y1)+1, len(y2))

    # Best lag
    lag_samples = lags[np.argmax(cross_corelation)]
    lag_sec = lag_samples * dt


    # Allign and compute scores on same grid
    y2_alligned = np.interp(tg, t2-lag_sec, x2)
    e = y2_alligned - y1
    rmse = float(np.sqrt(np.mean(e**2)))
    correlation = np.corrcoef(y2_alligned, y1)[0,1]


    return float(lag_sec), t_start, t_end, rmse, correlation

def cut_dataframe(df, t1, t2):
    return df[(df["t"] >= t1) & (df["t"] <= t2)]


def print_bag_topics(bagdir: str):
    bagdir = Path(bagdir)
    with AnyReader([bagdir]) as reader:
        print('Topics and message types:')
        for c in sorted(reader.connections, key=lambda x: x.topic):
            print(f'{c.topic:45s}  {c.msgtype}')
