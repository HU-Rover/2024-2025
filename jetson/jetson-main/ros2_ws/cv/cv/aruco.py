#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import tf2_ros
import cv2
import cv2.aruco as aruco
from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R
import tf_transformations as tf
import threading
import math
import time

class CamNode(Node):
    def __init__(self):
        super().__init__("cam_node")

        self.bridge = CvBridge()
        self.cam_pose_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.aruco_yokken = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber_goal = self.create_subscription(BehaviorTreeLog,'behavior_tree_log',self.goal_callback,10)
        self.sayac = 0
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.gidilen_idler = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(0.5, self.get_base_link_orientation)

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        self.aruco_parameters = aruco.DetectorParameters_create()
        self.marker_size = 0.16  # meters
        self.published = 0

        self.init_realsense()
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.user_input_thread = threading.Thread(target=self.wait_for_input_loop, daemon=True)
        self.user_input_thread.start()

    def get_base_link_orientation(self):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            self.q = trans.transform.rotation
        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")
            
    
    def goal_callback(self,msg:BehaviorTreeLog):
        for event in msg.event_log:
            
            if(event.node_name =='NavigateWithReplanning' and event.current_status =='SUCCESS'):
                ids = [] 
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()

                self.image = np.asanyarray(color_frame.get_data())
                self.image = cv2.flip(self.image, -1)
                gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

                corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_parameters)
                if ids is not None :
                    ids = ids.flatten()
                    
                    to_remove = set(ids) & set(self.gidilen_idler)
                    ids = [x for x in ids if x not in to_remove]
                    self.get_logger().info(f'{self.gidilen_idler}   {ids} {ids is None}')
                if ids is None:
                    ids = [] 
                
                while(len(ids) == 0 ):
                    frames = self.pipeline.wait_for_frames()
                    color_frame = frames.get_color_frame()

                    self.image = np.asanyarray(color_frame.get_data())
                    self.image = cv2.flip(self.image, -1)
                    gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

                    corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_parameters)
                    
                    if ids is not None :
                        ids = ids.flatten()
                        to_remove = set(ids) & set(self.gidilen_idler)

                        ids = [x for x in ids if x not in to_remove]
                        
                    if ids is None:
                        ids = []

                        
                    if (self.sayac < 40):
                        goal = Twist()
                        goal.linear.x = -0.5
                        goal.linear.y = 0.0
                        goal.linear.z = 0.0
            
                        goal.angular.x = 0.0
                        goal.angular.y = 0.0
                        goal.angular.z = 0.0
            
                        self.aruco_yokken.publish(goal)  
                        self.sayac += 1

                    else:
                        goal.linear.x = 0.0
                        goal.linear.y = 0.0
                        goal.linear.z = 0.0
              
                        goal.angular.x = 0.0
                        goal.angular.y = 0.0
                        goal.angular.z = 0.3
                        self.aruco_yokken.publish(goal)  
                
        

    def init_realsense(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        profile = self.pipeline.start(config)
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()

        self.camera_matrix = np.array([
            [intr.fx, 0.0, intr.ppx],
            [0.0, intr.fy, intr.ppy],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array(intr.coeffs[:5])

        self.get_logger().info("RealSense calibration loaded.")

    def quaternion_to_euler(self, x, y, z, w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def wait_for_input_loop(self):
        while rclpy.ok():
            try:
                ID = input().strip()
                if not ID:
                    continue
                self.lookup_and_publish_pose(ID)
            except Exception as e:
                self.get_logger().warn(f"{e}")

    def lookup_and_publish_pose(self, ID):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'map', f'goal_pose_{ID}', rclpy.time.Time())

            translation = transform.transform.translation
            rotation = transform.transform.rotation

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()

            goal_pose.pose.position.x = translation.x
            goal_pose.pose.position.y = translation.y
            goal_pose.pose.position.z = 0.0  # For 2D nav

            # Keep only yaw
            quat = [rotation.x, rotation.y, rotation.z, rotation.w]
            _, _, yaw = tf.euler_from_quaternion(quat)
            yaw_quat = tf.quaternion_from_euler(0, 0, yaw)

            goal_pose.pose.orientation.x = yaw_quat[0]
            goal_pose.pose.orientation.y = yaw_quat[1]
            goal_pose.pose.orientation.z = yaw_quat[2]
            goal_pose.pose.orientation.w = yaw_quat[3]

            self.cam_pose_pub.publish(goal_pose)
            self.get_logger().info(f"Published goal from goal_pose_{ID}")
            self.gidilen_idler.append(int(ID))
        except Exception as e:
            self.get_logger().warn(f"{e}")


    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        self.image = np.asanyarray(color_frame.get_data())
        self.image = cv2.flip(self.image, -1)
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_parameters)

        if ids is not None:
            self.sayac = 0
            aruco.drawDetectedMarkers(self.image, corners, ids)
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

            for i, marker_id in enumerate(ids.flatten()):

                # Publish the transform for the ArUco marker
                aruco_link = TransformStamped()
                aruco_link.header.stamp = self.get_clock().now().to_msg()
                aruco_link.header.frame_id = 'base_link'
                aruco_link.child_frame_id = f'aruco{marker_id}'
                aruco_link.transform.translation.x = tvec[i][0][2]
                aruco_link.transform.translation.y = -tvec[i][0][0]
                aruco_link.transform.translation.z = -tvec[i][0][1]
                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvec[i][0]))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()
                aruco_link.transform.rotation.x = quat[2]
                aruco_link.transform.rotation.y = -quat[0]
                aruco_link.transform.rotation.z = -quat[1]
                aruco_link.transform.rotation.w	 = quat[3]
                
                self.tf_broadcaster.sendTransform(aruco_link)
                
                quat_goal = tf.quaternion_from_euler(0, math.pi, 0)
                goal_tf = TransformStamped()
                goal_tf.header.stamp = self.get_clock().now().to_msg()
                goal_tf.header.frame_id = f'aruco{marker_id}'
                goal_tf.child_frame_id = f'goal_pose_{marker_id}'
                goal_tf.transform.translation.x = 2.0
                goal_tf.transform.translation.y = 0.0
                goal_tf.transform.translation.z = 0.0
                goal_tf.transform.rotation.x = quat_goal[0]
                goal_tf.transform.rotation.y = quat_goal[1]
                goal_tf.transform.rotation.z = quat_goal[2]
                goal_tf.transform.rotation.w = quat_goal[3]
                self.static_broadcaster.sendTransform(goal_tf)

                self.get_logger().info(f"Detected marker IDs: {ids.flatten()}")


def main(args=None):
    rclpy.init(args=args)
    node = CamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

