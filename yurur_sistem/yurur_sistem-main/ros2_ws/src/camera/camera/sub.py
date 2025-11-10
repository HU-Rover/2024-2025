#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')

        # Subscribe to compressed image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            'camera/image/compressed',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: CompressedImage):
        # Convert bytes back to numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)

        # Decode the image from JPEG format
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warn("Failed to decode image.")
            return

        # Display the image
        cv2.imshow("Compressed Image", image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

