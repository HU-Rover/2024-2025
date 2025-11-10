import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class RealSenseSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_subscriber')

        # Create a subscriber to the compressed image topic
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # Convert the compressed image data to a NumPy array
            np_arr = np.frombuffer(msg.data, np.uint8)

            # Decode the JPEG image
            color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)



            if color_image is not None:
            
                flipped_image = cv2.flip(color_image, -1)
                
                # Display the image using OpenCV
                cv2.imshow("Received Image(Flipped)", flipped_image)
                cv2.waitKey(1)  # Display each frame

            else:
                self.get_logger().warning('Failed to decode the image.')

        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    realsense_subscriber = RealSenseSubscriber()

    try:
        rclpy.spin(realsense_subscriber)
    except KeyboardInterrupt:
        realsense_subscriber.get_logger().info('Shutting down node.')
    finally:
        realsense_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

