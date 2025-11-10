import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import pyrealsense2 as rs
import cv2
import numpy as np

class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_compressed_publisher')

        # Publisher
        self.publisher_ = self.create_publisher(CompressedImage, 'camera/image/compressed',30)

        # RealSense setup
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        self.pipeline.start(config)

        # Timer to publish frames
        timer_period = 1.0 / 15  # 15 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            return

        # Convert to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Flip the image upside down
        color_image = cv2.flip(color_image, -1)

        # Convert to grayscale
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Optional: convert grayscale back to 3-channel so JPEG viewers work properly
        gray_bgr = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)

        # Compress to JPEG
        success, encoded_image = cv2.imencode('.jpg', gray_bgr)
        if not success:
            self.get_logger().warn("Failed to encode image")
            return

        # Create and publish CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = encoded_image.tobytes()

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

