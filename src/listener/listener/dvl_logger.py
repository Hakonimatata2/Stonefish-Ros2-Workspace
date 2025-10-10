import csv
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Bytt til riktig pakkenavn/type hvis nødvendig:
from stonefish_ros2.msg import DVL  # DVL-beams: stonefish_ros2/msg/DVLBeam

def cov3x3_as_matrix(cov):
    return [
        [cov[0], cov[1], cov[2]],
        [cov[3], cov[4], cov[5]],
        [cov[6], cov[7], cov[8]],
    ]

class DVLLogger(Node):
    def __init__(self):
        super().__init__('dvl_logger')

        # -------- parametere --------
        self.declare_parameter('dvl_topic', '/dvl/data')
        self.declare_parameter('best_effort_qos', True)
        self.declare_parameter('log_to_csv', True)
        self.declare_parameter('csv_path', str(Path.home() / 'dvl_log.csv'))
        self.declare_parameter('log_beams_as_json', True)

        dvl_topic = self.get_parameter('dvl_topic').get_parameter_value().string_value
        best_effort = self.get_parameter('best_effort_qos').get_parameter_value().bool_value
        self.log_to_csv = self.get_parameter('log_to_csv').get_parameter_value().bool_value
        self.csv_path = Path(self.get_parameter('csv_path').get_parameter_value().string_value)
        self.log_beams_json = self.get_parameter('log_beams_as_json').get_parameter_value().bool_value

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT if best_effort else ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        # -------- CSV forberedelse --------
        self.csv_file = None
        self.csv_writer = None
        if self.log_to_csv:
            self.csv_file = self.csv_path.open('w', newline='')
            fieldnames = [
                'stamp_sec','stamp_nanosec','frame_id',
                'vel_x','vel_y','vel_z',
                'altitude',
                # flater ut 3x3-kovarians
                'c00','c01','c02','c10','c11','c12','c20','c21','c22',
                'cov_invalid',
                'num_beams'
            ]
            if self.log_beams_json:
                fieldnames.append('beams_json')  # én kolonne med JSON
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
            self.csv_writer.writeheader()
            self.get_logger().info(f"Logger til CSV: {self.csv_path}")

        # -------- Abonnement --------
        self.sub_dvl = self.create_subscription(DVL, dvl_topic, self.on_dvl, qos)
        self.get_logger().info(f"Lytter på DVL-topic: {dvl_topic} (QoS={'BEST_EFFORT' if best_effort else 'RELIABLE'})")

    def on_dvl(self, msg: DVL):
        # Konsoll-logg (kompakt)
        v = msg.velocity
        C = msg.velocity_covariance
        cov_invalid = all(x == -1.0 for x in C)
        self.get_logger().info(
            f"DVL t={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} "
            f"frame={msg.header.frame_id} vel=({v.x:.3f},{v.y:.3f},{v.z:.3f}) m/s alt={msg.altitude:.2f} m "
            f"beams={len(msg.beams)} cov_invalid={cov_invalid}"
        )

        # CSV-logg
        if self.csv_writer:
            row = {
                'stamp_sec': msg.header.stamp.sec,
                'stamp_nanosec': msg.header.stamp.nanosec,
                'frame_id': msg.header.frame_id,
                'vel_x': v.x, 'vel_y': v.y, 'vel_z': v.z,
                'altitude': msg.altitude,
                'c00': C[0], 'c01': C[1], 'c02': C[2],
                'c10': C[3], 'c11': C[4], 'c12': C[5],
                'c20': C[6], 'c21': C[7], 'c22': C[8],
                'cov_invalid': cov_invalid,
                'num_beams': len(msg.beams),
            }
            if self.log_beams_json:
                # Legg hele beams ene kolonne som JSON (robust ved varierende lengde)
                # Anta DVLBeam har feltene range, velocity, velocity_covariance; tilpass ved behov.
                beams_out = []
                for b in msg.beams:
                    entry = {}
                    # getattr for å være robust hvis felt mangler i din konkrete type
                    entry['range'] = getattr(b, 'range', float('nan'))
                    entry['velocity'] = getattr(b, 'velocity', float('nan'))
                    vcov = getattr(b, 'velocity_covariance', [])
                    entry['velocity_covariance'] = list(vcov) if vcov else []
                    beams_out.append(entry)
                row['beams_json'] = json.dumps(beams_out)

            self.csv_writer.writerow(row)
            # flush lett slik at filen oppdateres kontinuerlig
            if self.csv_file:
                self.csv_file.flush()

    def destroy_node(self):
        # Rydd pent
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = DVLLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
