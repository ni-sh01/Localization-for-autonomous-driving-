import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from mocap_msgs.msg import RigidBodies
from std_srvs.srv import Trigger
import numpy as np
import transforms3d
import csv
from datetime import datetime
import os
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, dt):
        self.dt = dt
        self.Phi = np.eye(6)
        self.Phi[0, 1] = dt  # x, vx
        self.Phi[2, 3] = dt  # y, vy
        self.Phi[4, 5] = dt  # yaw, yaw_rate
        self.Q = np.eye(6) * 0.01
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1  # x
        self.H[1, 2] = 1  # y
        self.H[2, 4] = 1  # yaw
        self.R = np.eye(3) * 0.1
        self.x = np.zeros(6)
        self.P = np.eye(6)

    def predict(self):
        self.x = self.Phi @ self.x
        self.P = self.Phi @ self.P @ self.Phi.T + self.Q

    def correct(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
        return self.x

class EL_Localization(Node):
    def __init__(self):
        super().__init__('el_localization')
        self.kf = KalmanFilter(dt=0.1)
        self.pose_pub = self.create_publisher(PoseStamped, '/ego_pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/ego_twist', 10)
        self.create_subscription(RigidBodies, '/pose_modelcars', self.pose_callback, 10)
        self.create_service(Trigger, 'plot_data', self.plot_callback)
        self.log_path = os.path.expanduser("~/kalman_logs")
        os.makedirs(self.log_path, exist_ok=True)
        self.log_file = os.path.join(self.log_path, f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'yaw', 'vx', 'vy', 'yaw_rate'])

    def pose_callback(self, msg):
        for rb in msg.rigidbodies:
            if rb.rigid_body_name == '8':  # Change to your rigid body ID
                x = rb.pose.position.x
                y = rb.pose.position.y
                q = [rb.pose.orientation.w, rb.pose.orientation.x,
                     rb.pose.orientation.y, rb.pose.orientation.z]
                _, _, yaw = transforms3d.euler.quat2euler(q, axes='sxyz')
                z = np.array([x, y, yaw])
                self.kf.predict()
                state = self.kf.correct(z)
                x, vx, y, vy, yaw, yaw_rate = state

                # Publish pose
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'map'
                pose_msg.pose.position.x = x
                pose_msg.pose.position.y = y
                pose_msg.pose.orientation.z = yaw  # storing yaw in z for 2D
                self.pose_pub.publish(pose_msg)

                # Publish velocity
                twist_msg = TwistStamped()
                twist_msg.header.stamp = self.get_clock().now().to_msg()
                twist_msg.header.frame_id = 'base_link'
                twist_msg.twist.linear.x = vx
                twist_msg.twist.linear.y = vy
                twist_msg.twist.angular.z = yaw_rate
                self.twist_pub.publish(twist_msg)

                # Log data
                timestamp = self.get_clock().now().to_msg().sec
                with open(self.log_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, x, y, yaw, vx, vy, yaw_rate])

    def plot_callback(self, request, response):
        try:
            timestamps = []
            x_vals, y_vals, yaw_vals = [], [], []
            vx_vals, vy_vals, yaw_rate_vals = [], [], []

            with open(self.log_file, 'r') as file:
                reader = csv.DictReader(file)
                for row in reader:
                    timestamps.append(float(row['timestamp']))
                    x_vals.append(float(row['x']))
                    y_vals.append(float(row['y']))
                    yaw_vals.append(float(row['yaw']))
                    vx_vals.append(float(row['vx']))
                    vy_vals.append(float(row['vy']))
                    yaw_rate_vals.append(float(row['yaw_rate']))

            plt.figure(figsize=(12, 8))
            plt.subplot(3, 1, 1)
            plt.plot(timestamps, x_vals, label='x')
            plt.plot(timestamps, y_vals, label='y')
            plt.legend(); plt.title('Position'); plt.grid()

            plt.subplot(3, 1, 2)
            plt.plot(timestamps, yaw_vals, label='yaw')
            plt.legend(); plt.title('Yaw'); plt.grid()

            plt.subplot(3, 1, 3)
            plt.plot(timestamps, vx_vals, label='vx')
            plt.plot(timestamps, vy_vals, label='vy')
            plt.plot(timestamps, yaw_rate_vals, label='yaw_rate')
            plt.legend(); plt.title('Velocities and Turn Rate'); plt.grid()

            plt.tight_layout()
            plt.show()

            response.success = True
            response.message = "Plot displayed successfully."
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = EL_Localization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

