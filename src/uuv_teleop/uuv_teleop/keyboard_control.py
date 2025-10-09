#!/usr/bin/env python3

import sys, select, tty, termios
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# Config
TOPICS = [
    "/thruster_setpoints/T1", "/thruster_setpoints/T2",
    "/thruster_setpoints/T3", "/thruster_setpoints/T4",
    "/thruster_setpoints/T5", "/thruster_setpoints/T6",
    "/thruster_setpoints/T7", "/thruster_setpoints/T8",
]
U_LIMIT = 10.0  # N limit
RATE_HZ = 30.0  # publish frequency
STEP_F  = 5.0   # N step on key press
STEP_M  = 1.0   # Nm step

#       T1      T2         T3       T4    T5      T6     T7      T8
B = np.array([
    [ 0.7071,  0.7071, -0.7071, -0.7071,  0.0,   0.0,   0.0,    0.0 ],  # Fx (surge)
    [-0.7071,  0.7071, -0.7071,  0.7071,  0.0,   0.0,   0.0,    0.0 ],  # Fy (sway)
    [ 0.0,     0.0,     0.0,      0.0,   -1.0,   1.0,   1.0,   -1.0 ],  # Fz (heave)
    [ 0.0,     0.0,     0.0,      0.0,    0.218, 0.218,-0.218, -0.218], # Mx (roll)
    [ 0.0,     0.0,     0.0,      0.0,    0.12, -0.12,  0.12,  -0.12 ], # My (pitch)
    [-0.1888,  0.1888,  0.1888,  -0.1888, 0.0,   0.0,   0.0,    0.0 ],  # Mz (yaw)
], dtype=float)


def get_keyboard_keypress():
    """
    Get key press
    """
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    return sys.stdin.read(1) if dr else None

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        if len(TOPICS) != B.shape[1]:
            raise RuntimeError(f"TOPICS={len(TOPICS)} matcher ikke TAM-kolonner={B.shape[1]}")
        self.N = len(TOPICS)
        self.B_pinv = np.linalg.pinv(B)
        self.u_min = -U_LIMIT * np.ones(self.N)
        self.u_max =  U_LIMIT * np.ones(self.N)
        self.dt = 1.0 / RATE_HZ

        self.tau = np.zeros(6) # Desired [Fx,Fy,Fz,Mx,My,Mz]
        self.pubs = [self.create_publisher(Float64, toptic, 10) for toptic in TOPICS]

        # Make sure teminal can read keyboard inputs without pressing enter
        self.fd = sys.stdin.fileno()
        self.old_term = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        self.get_logger().info(
            "Taster: w/s=Fx ±, a/d=Fy ±, r/f=Fz ±, q/e=Mz ± | SPACE=0 | x/ESC=exit"
        )
        self.create_timer(self.dt, self.loop)

    def loop(self):
        # Read keyboard input
        while True:
            c = get_keyboard_keypress()
            if c is None:
                break
            if   c == 'w': self.tau[0] += STEP_F
            elif c == 's': self.tau[0] -= STEP_F
            elif c == 'a': self.tau[1] += STEP_F
            elif c == 'd': self.tau[1] -= STEP_F
            elif c == 'r': self.tau[2] += STEP_F
            elif c == 'f': self.tau[2] -= STEP_F
            elif c == 'q': self.tau[5] += STEP_M
            elif c == 'e': self.tau[5] -= STEP_M
            elif c == ' ': self.tau[:] = 0.0
            elif c in ['x', '\x1b']:  # x or ESC
                rclpy.shutdown()
                return

        # Compute forces using the psudo inverse u = B^+ * tau
        u = self.B_pinv @ self.tau
        u = np.clip(u, self.u_min, self.u_max)

        # Pblish the 
        for j, pub in enumerate(self.pubs):
            msg = Float64(); msg.data = float(u[j])
            pub.publish(msg)

        # Show status
        self.get_logger().info(f"tau={np.round(self.tau,2)}  u={np.round(u,2)}")

    def destroy_node(self):
        # Reset terminal as it was
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)
        except Exception:
            pass
        return super().destroy_node()

def main():
    rclpy.init()
    node = KeyboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
