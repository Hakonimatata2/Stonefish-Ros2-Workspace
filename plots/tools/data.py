from scipy.spatial.transform import Rotation as R
from tools.bag_to_df import bag_topic_to_dataframe
from tools.tools import *
import numpy as np


class Nucleus():
    """
    This class processes and holds ins data.
    The ins output data is in world frame.
    Angle offsets are removed (dvl is forward facing).
    The positions are translated from INS -> Vehicle Origin.
    """
   
    def __init__(self, bagdir, xyz_post, rpy_post, t0=None,
                 # Offset to IMU (from SOLAQUA paper)
                 r_cg_to_ins=np.array([0.178, 0.0, 0.402]),
                 rpy_cg_to_ins=np.array([np.deg2rad(-90), 0, np.deg2rad(-90)]), # Offsets due to mounting orientation
                 ):
        
        self.df_ins = bag_topic_to_dataframe(bagdir, topic="/nucleus1000dvl/ins")

        # Normalize time
        if t0: self.set_t0(t0)

        R_body_to_sensor = R.from_euler('xyz', rpy_cg_to_ins, degrees=False).as_matrix()
        R_sensor_to_body = R_body_to_sensor.T

        R_world_to_sensor = R.from_quat(self.df_ins[["quaternion.x", "quaternion.y", "quaternion.z", "quaternion.w"]].values).as_matrix()
        R_world_to_body = R_world_to_sensor @ R_sensor_to_body
        
        positions_ins = self.df_ins[["positionFrame.x", "positionFrame.y", "positionFrame.z"]].to_numpy()
        positions_cg = positions_ins - (R_world_to_body @ r_cg_to_ins.reshape(3,1)).reshape(-1,3)

        rpy_body = R.from_matrix(R_world_to_body).as_euler('xyz', degrees=False)
        
        roll, pitch, yaw  = rpy_body[:,0], rpy_body[:,1], rpy_body[:,2]
        x, y, z = positions_cg[:, 0], positions_cg[:, 1], positions_cg[:, 2]
        xyz_raw = np.array([x, y, z]).T
        rpy_raw = np.array([roll, pitch, yaw]).T

        # Transform path to net
        self.xyz, self.rpy = transform_path(xyz_post, rpy_post, xyz_raw, rpy_raw)


    # RAW data at IMU position
    @property
    def x(self): return self.xyz[:, 0]
    @property
    def y(self): return self.xyz[:, 1]
    @property
    def z(self): return self.xyz[:, 2]
    @property
    def roll(self): return self.rpy[:, 0]
    @property
    def pitch(self): return self.rpy[:, 1]
    @property
    def yaw(self): return self.rpy[:, 2]
    @property
    def t(self): return self.df_ins["t"].to_numpy(dtype=float)

    # Filtered at IMU position
    @property
    def x_f(self): return self.get_filtered_xyz_rpy()[0][:, 0]
    @property
    def y_f(self): return self.get_filtered_xyz_rpy()[0][:, 1]
    @property
    def z_f(self): return self.get_filtered_xyz_rpy()[0][:, 2]
    @property
    def roll_f(self): return self.get_filtered_xyz_rpy()[1][:, 0]
    @property
    def pitch_f(self): return self.get_filtered_xyz_rpy()[1][:, 1]
    @property
    def yaw_f(self): return self.get_filtered_xyz_rpy()[1][:, 2]


    def get_filtered_xyz_rpy(self, win_sec=1):
        """
        Returns two lists: filtered xyz and filtered rpy
        """
        from scipy.ndimage import gaussian_filter1d

        t = self.df_ins["t"].to_numpy()
        t0 = t[0]
        t -= t0
        dt = np.median(np.diff(t)) if len(t) > 1 else 0.01
        fs = 1.0 / max(dt, 1e-6) # Sample frequency

        def lp_gauss(signal):
            """
            Gaussian filtering with zero phase lag :)
            """
            FWHM = 2.355 # full width at half maximum
            sigma = (win_sec / FWHM) * fs
            return gaussian_filter1d(signal, sigma=sigma, mode='nearest')

        # Smooth signals
        x_s = lp_gauss(self.x)
        y_s = lp_gauss(self.y)
        z_s = lp_gauss(self.z)

        # From angle to unit circle
        roll_c  = np.exp(1j * self.roll)
        pitch_c = np.exp(1j * self.pitch)
        yaw_c   = np.exp(1j * self.yaw)

        # Filter real and imag separately
        roll_c_s = lp_gauss(roll_c.real) + 1j * lp_gauss(roll_c.imag)
        pitch_c_s = lp_gauss(pitch_c.real) + 1j * lp_gauss(pitch_c.imag)
        yaw_c_s = lp_gauss(yaw_c.real) + 1j * lp_gauss(yaw_c.imag)

        # Normalize
        roll_c_s  /= np.abs(roll_c_s)
        pitch_c_s /= np.abs(pitch_c_s)
        yaw_c_s   /= np.abs(yaw_c_s)

        # Back to angle
        roll_s = np.angle(roll_c_s)
        pitch_s = np.angle(pitch_c_s)
        yaw_s = np.angle(yaw_c_s)

        xyz_filtered = np.array([x_s, y_s, z_s]).T
        rpy_filtered = np.array([roll_s, pitch_s, yaw_s]).T

        return xyz_filtered, rpy_filtered
    
    def get_downsapled_filtered_xyz_rpy(self, every_n, win_sec=1):
        """
        Return list of downsampled time, xyz and rpy
        """
        xyz_filtered, rpy_filtered = self.get_filtered_xyz_rpy(win_sec)
        xyz_downsampled  = xyz_filtered[::every_n]
        rpy_downsampled  = rpy_filtered[::every_n]
        time_downsampled = self.t[::every_n]
        print(f"Num. keypoints: {len(xyz_downsampled)}")
        return time_downsampled, xyz_downsampled, rpy_downsampled

    def write_keypoints(self, every_n=10, win_sec=1):
        output_file = Path("/root/stonefish_ros2_ws/plots/keypoints.txt")
        # output_file = "keypoints.txt"
        t_ds, xyz_ds, rpy_ds = self.get_downsapled_filtered_xyz_rpy(every_n, win_sec)
        x_list, y_list, z_list = xyz_ds[:,0], xyz_ds[:,1], xyz_ds[:,2]
        roll_list, pitch_list, yaw_list = rpy_ds[:,0], rpy_ds[:,1], rpy_ds[:,2]

        fmt = lambda v: f"{v:.4f}"
        lines = []
        for t, x, y, z, roll, pitch, yaw in zip(t_ds, x_list, y_list, z_list, roll_list, pitch_list, yaw_list):
            line = (
                f'<keypoint time="{fmt(t)}" '
                f'xyz="{fmt(x)} {fmt(y)} {fmt(z)}" '
                f'rpy="{fmt(roll)} {fmt(pitch)} {fmt(yaw)}"/>'
            )
            lines.append(line)

        with open(output_file, "w") as f:
            f.write("\n".join(lines))

        print(f"Wrote {len(lines)} keypoints to {output_file}")

    def set_t0(self, t_0):
        self.df_ins["t"] -= t_0

    def start_at_t_zero(self):
        self.df_ins["t"] -= self.df_ins["t"][0]
