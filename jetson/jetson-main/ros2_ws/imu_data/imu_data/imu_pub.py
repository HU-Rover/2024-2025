import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
import numpy as np
import um7
import math
import time

class UM7Driver(Node):
    def __init__(self):
        super().__init__('um7_driver')
        self.imu = um7.UM7(
            'um7',
            '/dev/ttyTHS1',
            ['gyro_proc', 'accel_proc', 'yaw', 'pitch', 'roll']
        )
        time.sleep(0.5)

        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # 50 Hz
        for i in range (0,10):
            self.imu.catchallsamples(
                ['gyro_proc', 'accel_proc', 'yaw', 'pitch', 'roll'], 0.2
            )
            
        self.offset   = math.radians(self.imu.state.get('yaw',   0.0))

    def publish_imu_data(self):
        try:
            self.imu.catchallsamples(
                ['gyro_proc', 'accel_proc', 'yaw', 'pitch', 'roll'], 0.2
            )

            # --- Read Euler angles in radians ---
            yaw   = math.radians(self.imu.state.get('yaw',   0.0))
            pitch = math.radians(self.imu.state.get('pitch', 0.0))
            roll  = math.radians(self.imu.state.get('roll',  0.0))
            
            yaw = yaw - self.offset
            yaw = -yaw
            self.get_logger().info(f"{yaw}")
            
            

            # --- Build a unit quaternion via tf_transformations ---
            q = quaternion_from_euler(roll, pitch, yaw)          # [x, y, z, w]
            q = q / np.linalg.norm(q)                            # normalize

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            # Assign orientation
            imu_msg.orientation = Quaternion(
                x = float(q[0]),
                y = float(q[1]),
                z = float(q[2]),
                w = float(q[3])
            )

            # --- Angular velocity (rad/s) ---
            imu_msg.angular_velocity.x = math.radians(
                self.imu.state.get('gyro_proc_x', 0.0)
            )
            imu_msg.angular_velocity.y = math.radians(
                self.imu.state.get('gyro_proc_y', 0.0)
            )
            imu_msg.angular_velocity.z = math.radians(
                self.imu.state.get('gyro_proc_z', 0.0)
            )

            # --- Linear acceleration (m/s²) ---
            imu_msg.linear_acceleration.x = self.imu.state.get('accel_proc_x', 0.0)
            imu_msg.linear_acceleration.y = self.imu.state.get('accel_proc_y', 0.0)
            imu_msg.linear_acceleration.z = self.imu.state.get('accel_proc_z', 0.0)

            # --- Covariances ---
            imu_msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
            imu_msg.angular_velocity_covariance = imu_msg.orientation_covariance
            imu_msg.linear_acceleration_covariance = [
                0.1, 0.0, 0.0,
                0.0, 0.1, 0.0,
                0.0, 0.0, 0.1
            ]

            self.publisher_.publish(imu_msg)

        except Exception as e:
            self.get_logger().warn(f"UM7 read failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UM7Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
