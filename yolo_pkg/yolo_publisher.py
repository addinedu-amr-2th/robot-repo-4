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

model = YOLO("/home/ane4/amr-repository/amr_ws/pinkbot/src/yolo_pkg/yolo_pkg/yolov8m.pt")

font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 1
color = (0, 0, 255) # BGR color format
thickness = 2

class YoloPublisher(Node):
    def __init__(self):
        super().__init__('yolo_publisher')
        self.publisher_ = self.create_publisher(String, 'yolo_data', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.cv_bridge_ = CvBridge()
        self.cap = cv2.VideoCapture(-1)

    def timer_callback(self):
        ret, image = self.cap.read()
        if ret:
            result = model(image)[0]
            boxes = result.boxes.data
            boxes_list = boxes.tolist()

            if len(boxes_list) != 0:
                box1 = boxes[0]
                x1, y1, x2, y2, score, idx = box1
                box1_cls = result.names[int(idx)]

                cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(image, result.names[int(idx)], (int(x1), int(y1)-10), font, font_scale, color, thickness)

                x_center, y_center, diagnal = (x1+x2)/2, (y1+y2)/2, (x1-x2)**2+(y1-y2)**2
                data = [x_center.tolist(), y_center.tolist(), np.sqrt(diagnal.tolist()), score.tolist(), box1_cls]

                msg = String()
                msg.data = str(data)
                self.publisher_.publish(msg)
            else:
                data = []
                msg = String()
                msg.data = str(data)
                self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    yolo_publisher = YoloPublisher()
    rclpy.spin(yolo_publisher)
    yolo_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
