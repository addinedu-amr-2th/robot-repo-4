import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FrameSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription_ = self.create_subscription(Image, 'camera_topic', self.process_frame, 10)
        self.bridge = CvBridge()

    def process_frame(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        print(frame)
        cv2.imshow('Camera Frame', frame)
        cv2.waitKey(1)  # 1ms 대기 후 창 업데이트

def main(args=None):
    rclpy.init(args=args)
    subscriber = FrameSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
