import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        # Subscribe to the /camera/image_raw2 topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw2',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        try:
            # Convert compressed image to a NumPy array
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is not None:
                # Display the received image
                cv2.imshow('Received Image', image)
                cv2.waitKey(1)
            else:
                self.get_logger().error("Failed to decode image")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Camera subscriber node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

