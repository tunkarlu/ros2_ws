import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class ImageSubscriber(Node):
    def __init__(self, topic_name):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.listener_callback,
            10)
        self.br = CvBridge()
        self.get_logger().info(f"Subscribed to topic: {topic_name}")
    
    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        cv2.imshow(self.subscription.topic_name, current_frame)
        cv2.waitKey(1)
from cv_bridge import CvBridge
import cv2
def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 2:
        print("Usage: ros2 run ave_robo video_viewer <topic_name>")
        return

    topic_name = sys.argv[1]
    image_subscriber = ImageSubscriber(topic_name)
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  # Close OpenCV windows

if __name__ == '__main__':
    main()