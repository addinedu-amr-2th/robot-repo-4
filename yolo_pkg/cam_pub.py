import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_topic', 10)
        self.timer_ = self.create_timer(0.1, self.publish_frame)
        self.bridge_ = CvBridge()
        self.cap_ = cv2.VideoCapture(0)  # USB 카메라 장치 번호에 따라 변경할 수 있음

    def publish_frame(self):
        ret, frame = self.cap_.read()
        if ret:
            msg = self.bridge_.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
