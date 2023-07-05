#The general code from Open-manipulator packages

from math import exp
import os
import rclpy
import select
import sys
import threading
import time

from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

from std_msgs.msg import String, Bool, Float32

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

present_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
present_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

task_position_delta = 0.01  # kinematics meter
joint_angle_delta = 0.05  # radian default : 0.05
path_time = 0.5  # second


usage = """
Control Your OpenManipulator!
---------------------------
Task Space Control:
         (Forward, X+)
              W                   Q (Upward, Z+)
(Left, Y+) A     D (Right, Y-)    Z (Downward, Z-)
              X 
        (Backward, X-)

Joint Space Control:
- Joint1 : Increase (Y), Decrease (H)
- Joint2 : Increase (U), Decrease (J)
- Joint3 : Increase (I), Decrease (K)
- Joint4 : Increase (O), Decrease (L)
- Gripper: Increase (F), Decrease (G) | Fully Open (V), Fully Close (B)

INIT : (1)
HOME : (2)

CTRL-C to quit
"""

e = """
Communications Failed
"""


class TeleopKeyboard(Node):

    qos = QoSProfile(depth=10)
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    def __init__(self):
        super().__init__('teleop_keyboard')
        key_value = ''
        # Create joint_states subscriber
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            self.qos)
        self.joint_state_subscription

        # Create kinematics_pose subscriber
        self.kinematics_pose_subscription = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            self.qos)
        self.kinematics_pose_subscription

        # Create manipulator state subscriber
        self.open_manipulator_state_subscription = self.create_subscription(
            OpenManipulatorState,
            'states',
            self.open_manipulator_state_callback,
            self.qos)
        self.open_manipulator_state_subscription

        # Create Service Clients
        self.goal_joint_space = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.goal_task_space = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        self.tool_control = self.create_client(SetJointPosition, 'goal_tool_control')
        self.goal_joint_space_req = SetJointPosition.Request()
        self.goal_task_space_req = SetKinematicsPose.Request()
        self.tool_control_req = SetJointPosition.Request()

        # gpt subscriber
        self.gpt_sub=self.create_subscription(String, 'gpt_cmd', self.gpt_callback, self.qos)
        self.gpt_sub
        self.param = None
        self.method = None
        
        # success publisher
        self.success_pub=self.create_publisher(Bool, 'success', 10)
        self.success = Bool()
        self.success.data = True

        # yolo subscriber
        self.subscription_yolo = self.create_subscription(String, 'yolo_data', self.yolo_callback, 10)
        self.subscription_yolo
        self.x_center = 0.0
        self.y_center = 0.0
        self.class_name = 'none'
        self.target_cat = 'none'
        self.target_class = 'person'
        self.yolo_target_sub = self.create_subscription(String, 'yolo_target', self.yolo_target_callback, 10)
        self.yolo_cat_dict = {'none':['none'],
                              'vehicle':['bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat'],
                              'outdoor':['traffic light', 'fire hydrant', 'street sign', 'stop sign', 'parking meter', 'bench'],
                              'traffic':['traffic light', 'fire hydrant', 'street sign', 'stop sign', 'parking meter'],
                              'animal':['bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe'],
                              'accessory':['hat', 'backpack', 'umbrella', 'shoe', 'eye glasses', 'handbag', 'tie', 'suitcase'],
                              'cloth':['hat', 'shoe', 'tie'],
                              'sports':['frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket'],
                              'baseball':['baseball bat', 'baseball glove', 'sports ball'],
                              'toy':['book', 'baseball bat', 'baseball glove', 'sports ball', 'frisbee', 'teddy bear'],
                              'kitchen':['microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'blender', 'bottle', 'plate', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl'],
                              'drink':['bottle', 'wine glass', 'cup', 'bowl'],
                              'water':['bottle', 'wine glass', 'cup', 'bowl'],
                              'food':['bottle', 'wine glass', 'cup', 'bowl', 'plate', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake'],
                              'fruit':['banana', 'apple', 'orange'],
                              'vegitable':['broccoli', 'carrot'],
                              'bread':['sandwich', 'hot dog', 'pizza', 'donut', 'cake'],
                              'furniture':['chair', 'potted plant', 'bed', 'mirror', 'dining table', 'window', 'desk', 'toilet', 'door'],
                              'electronic':['tv', 'laptop', 'mouse', 'remote',' keyboard', 'cell phone'],
                              'appliance':['microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'blender'],
                              'indoor':['book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush', 'hair brush']}
        self.look = False
        
        # angle publisher
        self.angle_pub=self.create_publisher(Float32, 'angle', 10)
        self.angle=Float32()
        self.param_check_pub=self.create_publisher(String, 'param_check', 10)
        self.param_check=String()     
            
    def yolo_callback(self, msg):
        data = msg.data
        data_list = data.strip('[').strip(']').split(', ')
        #print(data_list)
        if len(data_list)>1 :
            self.x_center = float(data_list[0][:7])
            self.y_center = float(data_list[1][:7])
            self.class_name = data_list[-1].strip("\'")
            #print(self.x_center, self.y_center)
            self.param_check.data = self.class_name
            self.param_check_pub.publish(self.param_check)
        
            
    def yolo_target_callback(self, msg):
        data = msg.data
        if data!=None and len(data)>1:
            print(data)
            if data in ['person', 'bicycle', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'street sign', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'hat', 'backpack', 'umbrella', 'shoe', 'eyeglasses', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', 'plate', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'bananan', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'mirror', 'dining table', 'window', 'desk', 'toilet', 'door', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'blender', 'book', 'clock', 'vase', 'teddy bear', 'hair drier', 'toothbrush', 'hair brush']:
                self.target_class = data
                self.target_cat = None
            elif data in list(self.yolo_cat_dict.keys()):
                self.target_class = 'none'
                self.target_cat = data
            else :
                self.success.data = True
                self.success_pub.publish(self.success)


    def gpt_callback(self, msg):
        print(msg.data)
        tmp_list = msg.data.split(':')
        if tmp_list[0] == 'robot_arm':
            self.param = tmp_list[-1].strip(' ')
            self.method = tmp_list[1]
            print(self.method, self.param)
        

    def send_goal_task_space(self):
        self.goal_task_space_req.end_effector_name = 'gripper'
        self.goal_task_space_req.kinematics_pose.pose.position.x = goal_kinematics_pose[0]
        self.goal_task_space_req.kinematics_pose.pose.position.y = goal_kinematics_pose[1]
        self.goal_task_space_req.kinematics_pose.pose.position.z = goal_kinematics_pose[2]
        self.goal_task_space_req.kinematics_pose.pose.orientation.w = goal_kinematics_pose[3]
        self.goal_task_space_req.kinematics_pose.pose.orientation.x = goal_kinematics_pose[4]
        self.goal_task_space_req.kinematics_pose.pose.orientation.y = goal_kinematics_pose[5]
        self.goal_task_space_req.kinematics_pose.pose.orientation.z = goal_kinematics_pose[6]
        self.goal_task_space_req.path_time = path_time

        try:
            self.goal_task_space.call_async(self.goal_task_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Kinematic Pose failed %r' % (e,))

    def send_goal_joint_space(self, path_time):
        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.goal_joint_space_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1], goal_joint_angle[2], goal_joint_angle[3], goal_joint_angle[4]]
        self.goal_joint_space_req.path_time = path_time

        try:
            self.goal_joint_space.call_async(self.goal_joint_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))

    def send_tool_control_request(self):
        self.tool_control_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.tool_control_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1], goal_joint_angle[2], goal_joint_angle[3], goal_joint_angle[4]]
        self.tool_control_req.path_time = path_time

        try:
            self.tool_control_result = self.tool_control.call_async(self.tool_control_req)
            self.get_logger().info(self.tool_control_result)

        except Exception as e:
            self.get_logger().info('Tool control failed %r' % (e,))

    def kinematics_pose_callback(self, msg):
        present_kinematics_pose[0] = msg.pose.position.x
        present_kinematics_pose[1] = msg.pose.position.y
        present_kinematics_pose[2] = msg.pose.position.z
        present_kinematics_pose[3] = msg.pose.orientation.w
        present_kinematics_pose[4] = msg.pose.orientation.x
        present_kinematics_pose[5] = msg.pose.orientation.y
        present_kinematics_pose[6] = msg.pose.orientation.z

    def joint_state_callback(self, msg):
        present_joint_angle[0] = msg.position[0]
        present_joint_angle[1] = msg.position[1]
        present_joint_angle[2] = msg.position[2]
        present_joint_angle[3] = msg.position[3]
        present_joint_angle[4] = msg.position[4]

    def open_manipulator_state_callback(self, msg):
        if msg.open_manipulator_moving_state == 'STOPPED':
            for index in range(0, 7):
                goal_kinematics_pose[index] = present_kinematics_pose[index]
            for index in range(0, 5):
                goal_joint_angle[index] = present_joint_angle[index]

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print_present_values()
    return key

def print_present_values():
    print(usage)
    print('Joint Angle(Rad): [{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}]'.format(
        present_joint_angle[0],
        present_joint_angle[1],
        present_joint_angle[2],
        present_joint_angle[3],
        present_joint_angle[4]))
    print('Kinematics Pose(Pose X, Y, Z | Orientation W, X, Y, Z): {:.3f}, {:.3f}, {:.3f} | {:.3f}, {:.3f}, {:.3f}, {:.3f}'.format(
        present_kinematics_pose[0],
        present_kinematics_pose[1],
        present_kinematics_pose[2],
        present_kinematics_pose[3],
        present_kinematics_pose[4],
        present_kinematics_pose[5],
        present_kinematics_pose[6]))


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
        rclpy.init()
    except Exception as e:
        print(e)

    try:
        teleop_keyboard = TeleopKeyboard()
    except Exception as e:
        print(e)

    try:
        while(rclpy.ok()):
            rclpy.spin_once(teleop_keyboard)
            key_value = get_key(settings)
            print('key_value : ', key_value)

            print(teleop_keyboard.class_name)
            if key_value == '\x03': # crtl+c : exit
                break
            elif key_value == '' and teleop_keyboard.class_name == teleop_keyboard.target_class: # tracking object
                if teleop_keyboard.look == True:
                    teleop_keyboard.angle.data = prev_goal_joint_angle[0]
                    teleop_keyboard.angle_pub.publish(teleop_keyboard.angle)
            
                elif key_value == '' and teleop_keyboard.class_name != teleop_keyboard.target_class:
                    teleop_keyboard.angle.data = prev_goal_joint_angle[0]
                    teleop_keyboard.angle_pub.publish(teleop_keyboard.angle)
                    teleop_keyboard.angle.data = 0.0
                    teleop_keyboard.look = False
                    
                # print(teleop_keyboard.class_name)
                # print(teleop_keyboard.x_center)
                print('goal', prev_goal_joint_angle[0])
                if teleop_keyboard.x_center < 240:
                    angle_gap = 320 - teleop_keyboard.x_center
                    print(teleop_keyboard.x_center)
                    goal_joint_angle[0] = prev_goal_joint_angle[0] + angle_gap*(0.01/100)
                    print(goal_joint_angle[0])
                    pathtime = 0.01 #angle_gap*(0.01/180)
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    #time.sleep(0.1)
                    prev_goal_joint_angle[0] = goal_joint_angle[0]
                elif teleop_keyboard.x_center > 400:
                    angle_gap = teleop_keyboard.x_center - 320
                    print(teleop_keyboard.x_center)
                    goal_joint_angle[0] = prev_goal_joint_angle[0] - angle_gap*(0.01/100)
                    print(goal_joint_angle[0])
                    pathtime = 0.01 #angle_gap*(0.01/180)
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    #time.sleep(0.1)
                    prev_goal_joint_angle[0] = goal_joint_angle[0]

                if teleop_keyboard.method in ['look', 'lookat'] and abs(prev_goal_joint_angle[0]) < 0.05:
                    teleop_keyboard.look = False
                    teleop_keyboard.success.data = True
                    teleop_keyboard.success_pub.publish(teleop_keyboard.success)

                    
                elif teleop_keyboard.method in ['look', 'lookat'] or teleop_keyboard.param in ['look', 'lookat']:
                    teleop_keyboard.look = True

                    
                elif teleop_keyboard.method in ['search', 'find', 'follow'] or teleop_keyboard.param in ['search', 'find', 'follow']:
                    teleop_keyboard.look = True
                    teleop_keyboard.success.data = True
                    teleop_keyboard.success_pub.publish(teleop_keyboard.success)
                #     if teleop_keyboard.param not in ['person', 'bicycle', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'street sign', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'hat', 'backpack', 'umbrella', 'shoe', 'eyeglasses', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', 'plate', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'bananan', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'mirror', 'dining table', 'window', 'desk', 'toilet', 'door', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'blender', 'book', 'clock', 'vase', 'teddy bear', 'hair drier', 'toothbrush', 'hair brush']:
                #         teleop_keyboard.param = 'none'
                #     teleop_keyboard.method = 'none'
                # print(teleop_keyboard.x_center)      
                      
                # continue
                        
                ##y
                # if teleop_keyboard.y_center < 180:
                #     angle_gap = 200 - teleop_keyboard.y_center
                #     print(teleop_keyboard.y_center)
                #     goal_joint_angle[2] = prev_goal_joint_angle[2] - angle_gap*(0.01/100)
                #     print(goal_joint_angle[2])
                #     pathtime = 0.1
                #     teleop_keyboard.send_goal_joint_space(pathtime)
                #     #time.sleep(0.1)
                #     prev_goal_joint_angle[2] = goal_joint_angle[2]
                # elif teleop_keyboard.y_center > 300:
                #     angle_gap = teleop_keyboard.y_center - 200
                #     print(teleop_keyboard.y_center)
                #     goal_joint_angle[2] = prev_goal_joint_angle[2] + angle_gap*(0.01/100)
                #     print(goal_joint_angle[2])
                #     pathtime = 0.1
                #     teleop_keyboard.send_goal_joint_space(pathtime)
                #     #time.sleep(0.1)
                #     prev_goal_joint_angle[2] = goal_joint_angle[2]
                # else:
                #     print(teleop_keyboard.y_center)
                #     continue

            else:
                if key_value == 'w':
                    goal_kinematics_pose[0] = prev_goal_kinematics_pose[0] + task_position_delta
                    teleop_keyboard.send_goal_task_space()
                elif key_value == 'x':
                    goal_kinematics_pose[0] = prev_goal_kinematics_pose[0] - task_position_delta
                    teleop_keyboard.send_goal_task_space()
                elif key_value == 'a':
                    goal_kinematics_pose[1] = prev_goal_kinematics_pose[1] + task_position_delta
                    teleop_keyboard.send_goal_task_space()
                elif key_value == 'd':
                    goal_kinematics_pose[1] = prev_goal_kinematics_pose[1] - task_position_delta
                    teleop_keyboard.send_goal_task_space()
                elif key_value == 'q':
                    goal_kinematics_pose[2] = prev_goal_kinematics_pose[2] + task_position_delta
                    teleop_keyboard.send_goal_task_space()
                elif key_value == 'z':
                    goal_kinematics_pose[2] = prev_goal_kinematics_pose[2] - task_position_delta
                    teleop_keyboard.send_goal_task_space()
                elif key_value == 'y':
                    goal_joint_angle[0] = prev_goal_joint_angle[0] + joint_angle_delta
                    pathtime = path_time
                    teleop_keyboard.send_goal_joint_space(pathtime)
                elif key_value == 'h': 
                    goal_joint_angle[0] = prev_goal_joint_angle[0] - joint_angle_delta
                    pathtime = path_time
                    teleop_keyboard.send_goal_joint_space(pathtime)            
                elif key_value == 'u':
                    goal_joint_angle[1] = prev_goal_joint_angle[1] + joint_angle_delta
                    pathtime = path_time
                    teleop_keyboard.send_goal_joint_space(pathtime)
                elif key_value == 'j':
                    goal_joint_angle[1] = prev_goal_joint_angle[1] - joint_angle_delta
                    pathtime = path_time
                    teleop_keyboard.send_goal_joint_space(pathtime)
                elif key_value == 'i':
                    goal_joint_angle[2] = prev_goal_joint_angle[2] + joint_angle_delta
                    pathtime = path_time
                    teleop_keyboard.send_goal_joint_space(pathtime)
                elif key_value == 'k':
                    goal_joint_angle[2] = prev_goal_joint_angle[2] - joint_angle_delta
                    pathtime = path_time
                    teleop_keyboard.send_goal_joint_space(pathtime)
                elif key_value == 'o':
                    goal_joint_angle[3] = prev_goal_joint_angle[3] + joint_angle_delta
                    pathtime = path_time
                    teleop_keyboard.send_goal_joint_space(pathtime)
                elif key_value == 'l':
                    goal_joint_angle[3] = prev_goal_joint_angle[3] - joint_angle_delta
                    pathtime = path_time
                    teleop_keyboard.send_goal_joint_space(pathtime)
                elif key_value == 'f':
                    goal_joint_angle[4] = prev_goal_joint_angle[4] + 0.002
                    teleop_keyboard.send_tool_control_request()
                elif key_value == 'g':
                    goal_joint_angle[4] = prev_goal_joint_angle[4] - 0.002
                    teleop_keyboard.send_tool_control_request()
                elif key_value == 'v':
                    goal_joint_angle[4] = 0.01
                    teleop_keyboard.send_tool_control_request()
                elif key_value == 'b':
                    goal_joint_angle[4] = -0.01
                    teleop_keyboard.send_tool_control_request()
                    
                elif key_value == '1' or teleop_keyboard.method in ['idle', 'home', 'base']: #home
                    goal_joint_angle[0] = 0.0
                    goal_joint_angle[1] = -0.60
                    goal_joint_angle[2] = 0.0
                    goal_joint_angle[3] = 0.7
                    pathtime = 1.0
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    teleop_keyboard.success.data = True
                    teleop_keyboard.success_pub.publish(teleop_keyboard.success)
                    
                elif key_value == '2' or teleop_keyboard.method in ['grip', 'catch'] or teleop_keyboard.param in ['grip', 'catch']: #grip on
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = 0.8
                    goal_joint_angle[2] = -1.0
                    goal_joint_angle[3] = 0.70
                    goal_joint_angle[4] = 0.0
                    pathtime = 2.0
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(2)
                    pathtime = 1.0
                    teleop_keyboard.send_tool_control_request()
                    time.sleep(1)
                    goal_joint_angle[0] = 0.0
                    goal_joint_angle[1] = -0.60
                    goal_joint_angle[2] = 0.0
                    goal_joint_angle[3] = 0.7
                    pathtime = 1.0
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    teleop_keyboard.success.data = True
                    teleop_keyboard.success_pub.publish(teleop_keyboard.success)
                    
                elif key_value == '3' or teleop_keyboard.method in ['release'] or teleop_keyboard.param in ['release']: #grip off
                    goal_joint_angle[0] = 0.0
                    goal_joint_angle[1] = 0.8
                    goal_joint_angle[2] = -1.0
                    goal_joint_angle[3] = 0.70
                    goal_joint_angle[4] = 0.01
                    pathtime = 2.0
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(2)
                    pathtime = 1.0
                    teleop_keyboard.send_tool_control_request()
                    time.sleep(1)
                    goal_joint_angle[0] = 0.0
                    goal_joint_angle[1] = -0.60
                    goal_joint_angle[2] = 0.0
                    goal_joint_angle[3] = 0.7
                    pathtime = 1.0
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    teleop_keyboard.success.data = True
                    teleop_keyboard.success_pub.publish(teleop_keyboard.success)
                    
                elif key_value == '4' or teleop_keyboard.method in ['search', 'find'] or teleop_keyboard.param in ['search', 'find']: #searching
                    goal_joint_angle[0] = 0.0
                    goal_joint_angle[1] = 0.0
                    goal_joint_angle[2] = -1.57
                    goal_joint_angle[3] = 1.57
                    pathtime = 1.0
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(1)
                    goal_joint_angle[0] = 3.14
                    goal_joint_angle[1] = 0.0
                    goal_joint_angle[2] = -1.57
                    goal_joint_angle[3] = 1.57
                    pathtime = 2.0
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(2)
                    goal_joint_angle[0] = -3.14
                    goal_joint_angle[1] = 0.0
                    goal_joint_angle[2] = -1.57
                    goal_joint_angle[3] = 1.57
                    pathtime = 4.0
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(4)
                    goal_joint_angle[0] = 0.0
                    goal_joint_angle[1] = 0.0
                    goal_joint_angle[2] = -1.57
                    goal_joint_angle[3] = 1.57
                    pathtime = 2.0
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(2)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.6
                    goal_joint_angle[2] = 0.3
                    goal_joint_angle[3] = 0.7
                    pathtime = 1.0
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    #prev_goal_joint_angle[0] = goal_joint_angle[0]
                    teleop_keyboard.method = None
                    teleop_keyboard.success.data = True
                    teleop_keyboard.success_pub.publish(teleop_keyboard.success)
                    
                    
                elif key_value == '5' or teleop_keyboard.param =='bow' or teleop_keyboard.method=='bow': #bow
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.6
                    goal_joint_angle[2] = 0.3
                    goal_joint_angle[3] = 0.7
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.5)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = 0.0
                    goal_joint_angle[2] = 0.3
                    goal_joint_angle[3] = 1.0
                    goal_joint_angle[4] = -0.01
                    pathtime = 2.0
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    teleop_keyboard.send_tool_control_request()
                    time.sleep(2)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.6
                    goal_joint_angle[2] = 0.3
                    goal_joint_angle[3] = 0.7
                    goal_joint_angle[4] = 0.01
                    pathtime = 2.0
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    teleop_keyboard.send_tool_control_request()
                    teleop_keyboard.param = None
                    teleop_keyboard.success.data = True
                    teleop_keyboard.success_pub.publish(teleop_keyboard.success)
                    
                elif key_value == '6' or teleop_keyboard.param =='sad':
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = 0.0
                    goal_joint_angle[2] = 0.0
                    goal_joint_angle[3] = 1.57
                    pathtime = 1.0
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(1)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = 0.1
                    goal_joint_angle[2] = 0.1
                    goal_joint_angle[3] = 1.57
                    pathtime = 0.2
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.2)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = 0.0
                    goal_joint_angle[2] = 0.0
                    goal_joint_angle[3] = 1.57
                    pathtime = 0.2
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.2)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = 0.1
                    goal_joint_angle[2] = 0.1
                    goal_joint_angle[3] = 1.57
                    pathtime = 0.2
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.2)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = 0.0
                    goal_joint_angle[2] = 0.0
                    goal_joint_angle[3] = 1.57
                    pathtime = 0.2
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.2)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = 0.1
                    goal_joint_angle[2] = 0.1
                    goal_joint_angle[3] = 1.57
                    pathtime = 0.2
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.2)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.6
                    goal_joint_angle[2] = 0.3
                    goal_joint_angle[3] = 0.7
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    teleop_keyboard.param = None
                    teleop_keyboard.success.data = True
                    teleop_keyboard.success_pub.publish(teleop_keyboard.success)
                    
                elif key_value == '7' or teleop_keyboard.param in ['yes', 'agree', 'positive']: #positive
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.6
                    goal_joint_angle[2] = 0.3
                    goal_joint_angle[3] = 0.7
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.5)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.6
                    goal_joint_angle[2] = 0.3
                    goal_joint_angle[3] = 1.0
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.5)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.6
                    goal_joint_angle[2] = 0.3
                    goal_joint_angle[3] = 0.7
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.5)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.6
                    goal_joint_angle[2] = 0.3
                    goal_joint_angle[3] = 1.0
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.5)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.6
                    goal_joint_angle[2] = 0.3
                    goal_joint_angle[3] = 0.7
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    teleop_keyboard.param = None
                    teleop_keyboard.success.data = True
                    teleop_keyboard.success_pub.publish(teleop_keyboard.success)
                    
                elif key_value == '8' or teleop_keyboard.param in ['no', 'disagree', 'nagative']: #negative
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.6
                    goal_joint_angle[2] = 0.3
                    goal_joint_angle[3] = 0.7
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.5)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]+0.2
                    goal_joint_angle[1] = -0.6
                    goal_joint_angle[2] = 0.3
                    goal_joint_angle[3] = 0.7
                    pathtime = 0.2
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.2)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]-0.2
                    goal_joint_angle[1] = -0.6
                    goal_joint_angle[2] = 0.3
                    goal_joint_angle[3] = 0.7
                    pathtime = 0.4
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.4)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]+0.2
                    goal_joint_angle[1] = -0.6
                    goal_joint_angle[2] = 0.3
                    goal_joint_angle[3] = 0.7
                    pathtime = 0.4
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.4)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.6
                    goal_joint_angle[2] = 0.3
                    goal_joint_angle[3] = 0.7
                    pathtime = 0.2
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    teleop_keyboard.param = None
                    teleop_keyboard.success.data = True
                    teleop_keyboard.success_pub.publish(teleop_keyboard.success)
                    
                elif key_value == '9' or teleop_keyboard.param in ['happy', 'dance', 'dancing']: #dance
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = 0.0
                    goal_joint_angle[2] = -1.57
                    goal_joint_angle[3] = 0.0
                    pathtime = 1.0
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(1)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.5
                    goal_joint_angle[2] = -0.7
                    goal_joint_angle[3] = -0.5
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.5)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.3
                    goal_joint_angle[2] = -1.0
                    goal_joint_angle[3] = -0.2
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.5)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = 0.0
                    goal_joint_angle[2] = -1.57
                    goal_joint_angle[3] = 0.0
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.5)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = 0.3
                    goal_joint_angle[2] = -1.0
                    goal_joint_angle[3] = -0.4
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.5)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = 0.1
                    goal_joint_angle[2] = -0.7
                    goal_joint_angle[3] = 0.0
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.5)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.1
                    goal_joint_angle[2] = -1.0
                    goal_joint_angle[3] = 0.0
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.5)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.4
                    goal_joint_angle[2] = -0.7
                    goal_joint_angle[3] = -0.5
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.5)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.3
                    goal_joint_angle[2] = -1.0
                    goal_joint_angle[3] = -0.2
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.5)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = 0.0
                    goal_joint_angle[2] = -3.0
                    goal_joint_angle[3] = 0.0
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.5)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = 0.3
                    goal_joint_angle[2] = -1.57
                    goal_joint_angle[3] = -0.4
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.5)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.3
                    goal_joint_angle[2] = 0.0
                    goal_joint_angle[3] = 0.0
                    pathtime = 0.5
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(0.5)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.5
                    goal_joint_angle[2] = 0.3
                    goal_joint_angle[3] = 0.7
                    pathtime = 1.0
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    teleop_keyboard.param = None
                    teleop_keyboard.success.data = True
                    teleop_keyboard.success_pub.publish(teleop_keyboard.success)
                    
                elif key_value == '0' or teleop_keyboard.param in ['surprise', 'surprised']: #straight
                    goal_joint_angle[0] = 0.0
                    goal_joint_angle[1] = 0.0
                    goal_joint_angle[2] = -1.57
                    goal_joint_angle[3] = 0.0
                    pathtime = 1.0
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    time.sleep(1.0)
                    goal_joint_angle[4] = -0.01
                    teleop_keyboard.send_tool_control_request()
                    time.sleep(1.5)
                    goal_joint_angle[4] = 0.01
                    teleop_keyboard.send_tool_control_request()
                    time.sleep(1)
                    goal_joint_angle[4] = -0.01
                    teleop_keyboard.send_tool_control_request()
                    time.sleep(1.5)
                    goal_joint_angle[4] = 0.01
                    teleop_keyboard.send_tool_control_request()
                    time.sleep(1)
                    goal_joint_angle[0] = prev_goal_joint_angle[0]
                    goal_joint_angle[1] = -0.6
                    goal_joint_angle[2] = 0.3
                    goal_joint_angle[3] = 0.7
                    pathtime = 1.0
                    teleop_keyboard.send_goal_joint_space(pathtime)
                    teleop_keyboard.param = None
                    teleop_keyboard.success.data = True
                    teleop_keyboard.success_pub.publish(teleop_keyboard.success)        
                
                

                elif key_value == '\x03':
                    break
                else:
                    for index in range(0, 7):
                        prev_goal_kinematics_pose[index] = goal_kinematics_pose[index]
                    for index in range(0, 5):
                        prev_goal_joint_angle[index] = goal_joint_angle[index]
                
            teleop_keyboard.method = 'none'
            teleop_keyboard.param = 'none'
      

    except Exception as e:
        print(e)

    finally:
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop_keyboard.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
