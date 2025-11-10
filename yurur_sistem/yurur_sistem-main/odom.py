import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32
from rover_msgs.msg import EncoderMsg
import tf_transformations
import tf2_ros
import math
from message_filters import Subscriber, ApproximateTimeSynchronizer

class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/point_cloud', 10)

        # Subscribe to left and right encoder topics
        self.encoder_left_sub = Subscriber(self, EncoderMsg, '/encoder_left')  # Left wheels
        self.encoder_right_sub = Subscriber(self, EncoderMsg, '/encoder_right')  # Right wheels

        self.ts = ApproximateTimeSynchronizer(
            [self.encoder_left_sub, self.encoder_right_sub], queue_size=10, slop=0.05,
            allow_headerless = True
        )
        self.ts.registerCallback(self.update_odometry)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Robot parameters
        self.WHEEL_BASE = 0.94  # Distance between left and right wheels (meters)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

    def update_odometry(self, encoder_left_msg: EncoderMsg, encoder_right_msg: EncoderMsg):
        V_left = (encoder_left_msg.l_front + encoder_left_msg.l_back) / 2 
        V_right = (encoder_right_msg.r_front + encoder_right_msg.r_back) / 2

        V = (V_left + V_right) / 2.0
        omega = (V_right - V_left) / self.WHEEL_BASE

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Update robot pose
        self.theta += omega * dt
        self.x += V * math.cos(self.theta) * dt
        self.y += V * math.sin(self.theta) * dt

        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.odom_pub.publish(odom_msg)

        # TF: odom -> base_link
        t_base = TransformStamped()
        t_base.header.stamp = current_time.to_msg()
        t_base.header.frame_id = 'odom'
        t_base.child_frame_id = 'base_link'
        t_base.transform.translation.x = self.x
        t_base.transform.translation.y = self.y
        t_base.transform.translation.z = 0.0
        t_base.transform.rotation.x = q[0]
        t_base.transform.rotation.y = q[1]
        t_base.transform.rotation.z = q[2]
        t_base.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t_base)

        # TF: base_link -> sensor (e.g., camera)
        t_sensor = TransformStamped()
        t_sensor.header.stamp = current_time.to_msg()
        t_sensor.header.frame_id = 'base_link'
        t_sensor.child_frame_id = 'camera_depth_optical_frame'
        t_sensor.transform.translation.x = 0.2
        t_sensor.transform.translation.y = 0.0
        t_sensor.transform.translation.z = 0.1
        q_sensor = tf_transformations.quaternion_from_euler(-1.57, 0, -1.57)
        t_sensor.transform.rotation.x = q_sensor[0]
        t_sensor.transform.rotation.y = q_sensor[1]
        t_sensor.transform.rotation.z = q_sensor[2]
        t_sensor.transform.rotation.w = q_sensor[3]
        self.tf_broadcaster.sendTransform(t_sensor)

def main():
    rclpy.init()
    node = WheelOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

