import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        # Publisher for compressed images on the /camera/image_raw topic
        self.publisher = self.create_publisher(CompressedImage, '/camera/image_raw', 10)
        # Create a timer that fires at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        # Initialize the camera capture (default camera index 0)
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame from camera")
            return

        # Compress the image frame using JPEG encoding
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            self.get_logger().error("Failed to compress the image")
            return

        # Create and populate the CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"  # Specify the compression format
        msg.data = buffer.tobytes()  # Convert the image buffer to bytes

        # Publish the compressed image
        self.publisher.publish(msg)
        self.get_logger().info("Published a compressed image frame")

    def destroy_node(self):
        # Release the camera resource when shutting down
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Camera publisher node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

