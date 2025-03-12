import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CVSubscriber(Node):

    def __init__(self):
        super().__init__('cv_simple_sub_pub')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Image, '/processed_image', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Received image')
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Convert the image to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Apply binary threshold
        _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
        # Display the binary image
        # cv2.imshow("Binary Image", binary_image)
        # cv2.waitKey(1)
        # Convert the binary image back to ROS Image message
        binary_image_msg = self.bridge.cv2_to_imgmsg(binary_image, encoding='mono8')
        # Publish the binary image
        self.publisher_.publish(binary_image_msg)


def main(args=None):
    rclpy.init(args=args)

    cv_subscriber = CVSubscriber()

    rclpy.spin(cv_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cv_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
