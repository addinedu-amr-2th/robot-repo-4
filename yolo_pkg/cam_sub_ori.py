import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from ultralytics import YOLO
from PIL import Image
import cv2
import numpy as np
import os

model = YOLO("/home/yun/robot_ws/yolov8m.pt")
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 1
color = (0, 0, 255) # BGR color format
thickness = 2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription_ = self.create_subscription(Image, 'camera_topic', self.callback, 10)
        self.bridge_ = CvBridge()
        self.publisher_ = self.create_publisher(String, 'yolo_data', 10)
        self.timer_ = self.create_timer(0.01, self.timer_callback)

    def callback(self, msg):
        self.frame = self.bridge_.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def timer_callback(self):
        result = model(self.frame)[0]
        boxes = result.boxes.boxes
        boxes_list = boxes.tolist()

        if len(boxes_list) != 0:
            box1 = boxes[0]
            x1, y1, x2, y2, score, idx = box1

            cv2.rectangle(self.frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(self.frame, result.names[int(idx)], (int(x1), int(y1)-10), font, font_scale, color, thickness)

            x_center, y_center, diagnal = (x1+x2)/2, (y1+y2)/2, (x1-x2)**2+(y1-y2)**2
            data = [x_center.tolist(), y_center.tolist(), np.sqrt(diagnal.tolist()), score.tolist()]

            msg = String()
            msg.data = str(data)
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
