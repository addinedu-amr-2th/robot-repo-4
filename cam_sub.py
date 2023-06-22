import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera_topic',
            self.receive_frame,
            10
        )

    def receive_frame(self, msg):
        frame = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        cv2.imshow('Frame', frame)
        cv2.waitKey(1)  # Refresh display (1 ms)

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
