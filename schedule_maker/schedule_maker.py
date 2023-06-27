import rclpy as rp
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import time
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from minibot_interfaces.msg import LampCommand
import datetime, requests
from bs4 import BeautifulSoup
from fake_useragent import UserAgent
import urllib
import numpy as np
import threading

class ScheduleMaker(Node):
    def __init__(self):
        super().__init__('schedule_maker')
        self.ua = UserAgent(verify_ssl=False)
        self.userAgent = self.ua.random
        self.headers = {'User-Agent': self.userAgent}
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()
        self.gpt_feedback = String()
        self.gpt_feedback.data = ''
        self.feedback_publisher = self.create_publisher(String, 'agent_feedback_text', 10)
        self.gpt_cmd = String()
        self.robot_arm_command_publisher = self.create_publisher(String, 'gpt_cmd', 10)
        self.music = String()
        self.music_publisher = self.create_publisher(String, 'music', 10)
        self.schedule_subscription = self.create_subscription(String, 'schedule', self.schedule_callback, 10)
        self.yolo_target_publisher = self.create_publisher(String, 'yolo_target', 10)
        self.yolo_target = String()
        self.schedule_conductor = ScheduleConductor()
        self.priority = False
        self.success_subscription = self.create_subscription(Bool, 'success', self.success_callback, 10)
        self.success = Bool()
        self.success.data = True
        self.temp_schedule = None
        self.pause = False
        self.tts_text = String()
        self.tts_text.data = ''
        self.tts_publisher = self.create_publisher(String, 'tts_text', 10)
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        self.lamp_command = LampCommand()
        self.pose_current = PoseWithCovarianceStamped()
        self.lamp_publisher = self.create_publisher(LampCommand, '/minibot_io_controller/set_lamp', 10)
        self.twist_publisher = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        self.pos_subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.angle_subscription = self.create_subscription(Float32, 'angle', self.look_callback, 10)
        self.do_look = False
        self.do_follow = False
        self.do_stop = False
        self.angle = 0
        self.timer = self.create_timer(0.5, self.timer_callback)
        #self.param_check_sub = self.create_subscription(String, 'param_check', self.param_check, 10)
        
    # def param_check(self, param):
    #     if param.data is not None and len(param.data) > 0:
    #         if param.data != 'person':
    #             self.do_follow = False
    #             # self.do_stop = True
    
    def look_callback(self, msg):
        print(self.do_look, self.twist)
        print(msg.data)
        self.angle = msg.data
            
    def timer_callback(self):
        
        if self.do_follow == True:
            self.twist.linear.x = 0.1
        if self.do_look == True or self.do_follow == True:
            self.twist.angular.z = self.angle
        if self.do_stop == True:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.do_follow = False  
            # self.nav.cancelTask() 
        print(self.do_follow, self.do_look, self.do_stop, self.twist)
        self.twist_publisher.publish(self.twist)
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        
    def pose_callback(self, msg):
        self.pose_current = msg
        
    def parse_schedule(self, input_schedule=None):
        device, behavior, param = 'turtlebot', 'speak', None
        print(input_schedule)
        if input_schedule != None and len(input_schedule)>2:
            # input_schedule = input_schedule.strip().replace('(', '').replace(')', '').replace("\'", "").replace('\"','')
            if ':' in input_schedule:
                target_behavior = [behavior.strip().strip(' ') for behavior in input_schedule.split(':')]
                if len(target_behavior) > 2:
                    device, behavior, param = target_behavior[:3]
                elif len(target_behavior) == 2:
                    behavior, param = target_behavior
                elif len(target_behavior) == 1:
                    behavior = target_behavior
            
            elif behavior in ['priory','priority']:
                self.set_priority(device, behavior, param)
                return None, None, None
                    
            else:
                param = input_schedule      
            
        return device, behavior, param
            
            
    def set_priority(self, device='turtlebot', behavior='priority', param='low'):
        if param.strip() in ['low', 'last', 'false', 'less', 'lower']:
            self.priority = False
            
        elif param.strip() in ['high', 'first', 'critical', 'top', 'hight']:
            self.priority = True
            
    
    def schedule_callback(self, msg):
        if msg.data.isdigit() == False:
            input_schedules = msg.data.lower() + '*END:END:END'
            print('input_schedules :', input_schedules)
            if input_schedules!=None and len(input_schedules)>2 and ':' in input_schedules:
                if '*' not in input_schedules:
                    input_schedule = input_schedules
                    device, behavior, param = self.parse_schedule(input_schedule)
                    self.schedule_conductor.add_schedule(self.priority, device, behavior, param)
                    if behavior in ['pause', 'stop']:
                        input_schedule, input_schedules = None, None
                        
                else :
                    for input_schedule in input_schedules.split('*'):
                        device, behavior, param = self.parse_schedule(input_schedule)
                        self.schedule_conductor.add_schedule(self.priority, device, behavior, param)
                        if behavior in ['pause', 'stop']:
                            input_schedule, input_schedules = None, None
                            break
                self.priority = False
                
            
            if len(self.schedule_conductor.composed_schedule)<1 and len(self.schedule_conductor.schedule)>0:
                self.schedule_conductor.composed_schedule.append(self.schedule_conductor.schedule.pop(0)) 
                
            print('do behavior...')
            self.success.data = True
            self.success_callback(self.success)
            
            
    def success_callback(self, msg):
        # self.do_look = False
        # self.do_follow = False
        # self.do_stop = False
        if msg.data == True:
            print("success")
            # self.nav.cancelTask()
            if self.schedule_conductor.composed_schedule!=None and len(self.schedule_conductor.composed_schedule)>0:
                device, behavior, param = self.schedule_conductor.composed_schedule.pop(0)
                self.do_behavior(device, behavior, param)

        else:
            print("fail")
            # self.nav.cancelTask()
            self.schedule_conductor.composed_schedule = []
            self.schedule_conductor.schedule = []


    def do_behavior(self, device='turtlebot', behavior='speak', param=''):
        print("current behavior :", device, behavior, param)
            
        # do action  
        if device in ['turtlebot', 'minibot', 'pinkbot', 'robot']:
            if behavior in ['goto', 'go', 'move']:
                self.move(param)
                
            elif behavior in ['emotion']:
                if param in['idle', 'happy', 'no', 'yes', 'surprise', 'sad']:
                    self.gpt_cmd.data = f'robot_arm:emotion:{param}'
                    print(self.gpt_cmd.data)
                    self.robot_arm_command_publisher.publish(self.gpt_cmd)
                    
            elif behavior in ['motion']:
                if param in ['idle', 'yes', 'no', 'grip', 'release', 'search', 'bow']:
                    self.gpt_cmd.data = f'robot_arm:motion:{param}'
                    print(self.gpt_cmd.data)
                    self.robot_arm_command_publisher.publish(self.gpt_cmd)
                    
            elif behavior in ['bow']:
                self.gpt_cmd.data = f'robot_arm:motion:bow'
                print(self.gpt_cmd.data)
                self.robot_arm_command_publisher.publish(self.gpt_cmd)
                
            elif behavior in ['wait', 'delay', "stay"]:
                self.do_look = False
                self.do_follow = False
                if param.isdigit() == True:
                    time.sleep(int(param))
                    
                else:
                    time.sleep(3)
                self.do_stop = True
                
            elif param in ["stop", "halt", "stand", "freeze"]:
                if param.isdigit() == True:
                    time.sleep(int(param))
                    self.do_stop = True
                    
                else:
                    time.sleep(3)
                    self.do_stop = True
                    
            elif param in ["follow"]:
                self.do_stop = False
                self.do_follow = True
                self.yolo_target.data = 'person'
                print(self.yolo_target.data)
                self.yolo_target_publisher.publish(self.yolo_target)
                self.success.data = True
                self.success_callback(self.success)
                    
            elif behavior in ['led', 'light', 'lamp', 'eye']:
                self.led(param)
                self.success.data = True
                self.success_callback(self.success)
                
            elif behavior in ['blink']:
                self.led('blink')
                self.success.data = True
                self.success_callback(self.success)
                
            elif behavior in ['look_at', 'look']:
                self.do_stop = False
                self.do_look = True
                self.yolo_target.data = param
                print(self.yolo_target.data)
                self.yolo_target_publisher.publish(self.yolo_target)
                self.gpt_cmd.data = f'robot_arm:look:{param}'
                print(self.gpt_cmd.data)
                self.robot_arm_command_publisher.publish(self.gpt_cmd)
                self.success.data = True
                self.success_callback(self.success)
            
            elif behavior in ['follow']:
                self.do_stop = False
                self.do_follow = True
                self.yolo_target.data = param
                print(self.yolo_target.data)
                self.yolo_target_publisher.publish(self.yolo_target)
                self.success.data = True
                self.success_callback(self.success)
            
            elif behavior in ['speak', 'talk', 'tell']:
                if param != None and len(param) > 2 :
                    self.tts_text.data = param
                    self.tts_publisher.publish(self.tts_text)
                    self.tts_text.data = ''
                self.success.data = True
                self.success_callback(self.success)
                            
            elif behavior in ['search', 'find']:
                self.gpt_cmd.data = f'robot_arm:motion:search'
                print(self.gpt_cmd.data)
                self.yolo_target.data = param
                self.yolo_target_publisher.publish(self.yolo_target)
                self.robot_arm_command_publisher.publish(self.gpt_cmd)
            
            elif behavior in ['info', 'req' 'request']:
                self.info(param)    
                self.success.data = True
                self.success_callback(self.success)          
                
            elif behavior in ['turn', 'spin', 'rotate']:
                self.do_stop = False
                self.turn(param)
                self.success.data = True
                self.success_callback(self.success)
                
            elif behavior in ['song', 'music', 'play', 'mp3', 'wav', 'bgm']:
                self.music.data = param
                self.music_publisher.publish(self.music)
                self.success.data = True
                self.success_callback(self.success)
                
            else:
                self.success.data = True
                self.success_callback(self.success)
        
        
        elif device in ['robotarm','robot arm', 'robot_arm', 'open_manipulator', 'open manipulator']:
            
            if behavior in ['motion']:
                if param in ['idle', 'yes', 'no', 'grip', 'release', 'search', 'bow', 'find', 'agree', 'disagree']:
                    self.gpt_cmd.data = f'robot_arm:motion:{param}'
                    print(self.gpt_cmd.data)
                    self.robot_arm_command_publisher.publish(self.gpt_cmd)
                    
            elif behavior in ['emotion']:
                if param in['idle', 'happy', 'no', 'yes', 'surprise', 'sad']:
                    self.gpt_cmd.data = f'robot_arm:emotion:{param}'
                    print(self.gpt_cmd.data)
                    self.robot_arm_command_publisher.publish(self.gpt_cmd)
                        
            elif behavior in ['move']:
                self.success.data = True
                self.success_callback(self.success)
                
            elif behavior in ['dance']:
                self.gpt_cmd.data = f'robot_arm:emotion:happy'
                print(self.gpt_cmd.data)
                self.robot_arm_command_publisher.publish(self.gpt_cmd)
                
            elif behavior in ['grip']:
                self.gpt_cmd.data = f'robot_arm:motion:grip'
                print(self.gpt_cmd.data)
                self.yolo_target.data = param
                self.yolo_target_publisher.publish(self.yolo_target)
                self.robot_arm_command_publisher.publish(self.gpt_cmd)
                
            elif behavior in ['release']:
                self.gpt_cmd.data = f'robot_arm:motion:release'
                print(self.gpt_cmd.data)
                self.yolo_target.data = param
                self.yolo_target_publisher.publish(self.yolo_target)
                self.robot_arm_command_publisher.publish(self.gpt_cmd)
                
            elif behavior in ['search', 'find']:
                self.gpt_cmd.data = f'robot_arm:motion:search'
                print(self.gpt_cmd.data)
                self.yolo_target.data = param
                self.yolo_target_publisher.publish(self.yolo_target)
                self.robot_arm_command_publisher.publish(self.gpt_cmd)
            
            elif behavior in ['look_at', 'look', 'follow']:
                self.do_stop = False
                self.do_look = True
                self.yolo_target.data = param
                print(self.yolo_target.data)
                self.yolo_target_publisher.publish(self.yolo_target)
                self.gpt_cmd.data = f'robot_arm:look:{param}'
                print(self.gpt_cmd.data)
                self.robot_arm_command_publisher.publish(self.gpt_cmd)
                self.success.data = True
                self.success_callback(self.success)
            
            elif behavior in ['turn', 'spin', 'rotate']:
                self.do_stop = False
                self.success.data = True
                self.success_callback(self.success)
                
            else:
                self.success.data = True
                self.success_callback(self.success)
        else:
            self.success.data = True
            self.success_callback(self.success)
                
                
    def move(self, param='front'):
        if param.isdigit():
            prev_pose = self.pose_current.pose.pose
            prev_position = prev_pose.position
            prev_orientation = prev_pose.orientation
            prev_angle = float(euler_from_quaternion([prev_orientation.x, prev_orientation.y, prev_orientation.z, prev_orientation.w])[2])
            target_position = [prev_position.x + float(param)*float(np.cos(prev_angle)), prev_position.y + float(param)*float(np.sin(prev_angle)), prev_position.z]
            self.goal_pose.pose.orientation = prev_orientation
            self.goal_pose.pose.position.x = target_position[0]
            self.goal_pose.pose.position.y = target_position[1]
            self.goal_pose.pose.position.z = target_position[2]
            self.nav.goToPose(self.goal_pose)
            i=0
            while not self.nav.isTaskComplete():
                i += 1
                feedback = self.nav.getFeedback()
                distance = feedback.distance_remaining
                print(distance)
                if abs(distance) < 0.3:
                    self.nav.cancelTask()
                    self.success.data = True
                    self.success_callback(self.success)  
                    
        elif param in ["follow", "following"]:
            self.do_follow = True
            self.yolo_target.data = 'person'
            print(self.yolo_target.data)
            self.yolo_target_publisher.publish(self.yolo_target)
            self.success.data = True
            
            self.success_callback(self.success)
            
        elif param in ["front", "forward", "ahead", "infront"]:
            prev_pose = self.pose_current.pose.pose
            prev_position = prev_pose.position
            prev_orientation = prev_pose.orientation
            prev_angle = float(euler_from_quaternion([prev_orientation.x, prev_orientation.y, prev_orientation.z, prev_orientation.w])[2])
            target_position = [prev_position.x + 1*float(np.cos(prev_angle)), prev_position.y + 1*float(np.sin(prev_angle)), prev_position.z]
            self.goal_pose.pose.orientation = prev_orientation
            self.goal_pose.pose.position.x = target_position[0]
            self.goal_pose.pose.position.y = target_position[1]
            self.goal_pose.pose.position.z = target_position[2]
            self.nav.goToPose(self.goal_pose)
            i=0
            while not self.nav.isTaskComplete():
                i += 1
                feedback = self.nav.getFeedback()
                distance = feedback.distance_remaining
                print(distance)
                if abs(distance) < 0.3:
                    self.nav.cancelTask()
                    self.success.data = True
                    self.success_callback(self.success) 
            
        elif param in ["back", "backward", "behind", "reverse"]:
            prev_pose = self.pose_current.pose.pose
            prev_position = prev_pose.position
            prev_orientation = prev_pose.orientation
            prev_angle = float(euler_from_quaternion([prev_orientation.x, prev_orientation.y, prev_orientation.z, prev_orientation.w])[2])
            target_orientation = quaternion_from_euler(0, 0, prev_angle+float(np.pi))
            target_position = [prev_position.x - 1*float(np.cos(prev_angle)), prev_position.y - 1*float(np.sin(prev_angle)), prev_position.z]
            self.goal_pose.pose.orientation.x = target_orientation[0]
            self.goal_pose.pose.orientation.y = target_orientation[1]
            self.goal_pose.pose.orientation.z = target_orientation[2]
            self.goal_pose.pose.orientation.w = target_orientation[3]
            self.nav.goToPose(self.goal_pose)
            i=0
            while not self.nav.isTaskComplete():
                i += 1
                feedback = self.nav.getFeedback()
                print(feedback)
            self.goal_pose.pose.position.x = target_position[0]
            self.goal_pose.pose.position.y = target_position[1]
            self.goal_pose.pose.position.z = target_position[2]
            self.nav.goToPose(self.goal_pose)
            i=0
            while not self.nav.isTaskComplete():
                i += 1
                feedback = self.nav.getFeedback()
                distance = feedback.distance_remaining
                print(distance)
                if abs(distance) < 0.3:
                    self.nav.cancelTask()
                    self.success.data = True
                    self.success_callback(self.success) 
            
        elif param in ["left", "leftward"]:
            prev_pose = self.pose_current.pose.pose
            prev_position = prev_pose.position
            prev_orientation = prev_pose.orientation
            prev_angle = float(euler_from_quaternion([prev_orientation.x, prev_orientation.y, prev_orientation.z, prev_orientation.w])[2])
            target_orientation = quaternion_from_euler(0, 0, prev_angle+float(np.pi)*0.5)
            target_position = [prev_position.x + 1*float(np.sin(prev_angle)), prev_position.y - 1*float(np.cos(prev_angle)), prev_position.z]
            self.goal_pose.pose.orientation.x = target_orientation[0]
            self.goal_pose.pose.orientation.y = target_orientation[1]
            self.goal_pose.pose.orientation.z = target_orientation[2]
            self.goal_pose.pose.orientation.w = target_orientation[3]
            self.nav.goToPose(self.goal_pose)
            time.sleep(5)
            self.goal_pose.pose.position.x = target_position[0]
            self.goal_pose.pose.position.y = target_position[1]
            self.goal_pose.pose.position.z = target_position[2]
            self.nav.goToPose(self.goal_pose)
            i=0
            while not self.nav.isTaskComplete():
                i += 1
                feedback = self.nav.getFeedback()
                distance = feedback.distance_remaining
                print(distance)
                if abs(distance) < 0.3:
                    self.nav.cancelTask()
                    self.success.data = True
                    self.success_callback(self.success) 
            
        elif param in ["right", "rightward"]:
            prev_pose = self.pose_current.pose.pose
            prev_position = prev_pose.position
            prev_orientation = prev_pose.orientation
            prev_angle = float(euler_from_quaternion([prev_orientation.x, prev_orientation.y, prev_orientation.z, prev_orientation.w])[2])
            target_orientation = quaternion_from_euler(0, 0, prev_angle-float(np.pi)*0.5)
            target_position = [prev_position.x - 1*float(np.sin(prev_angle)), prev_position.y + 1*float(np.cos(prev_angle)), prev_position.z]
            self.goal_pose.pose.orientation.x = target_orientation[0]
            self.goal_pose.pose.orientation.y = target_orientation[1]
            self.goal_pose.pose.orientation.z = target_orientation[2]
            self.goal_pose.pose.orientation.w = target_orientation[3]
            self.nav.goToPose(self.goal_pose)
            time.sleep(5)
            self.goal_pose.pose.position.x = target_position[0]
            self.goal_pose.pose.position.y = target_position[1]
            self.goal_pose.pose.position.z = target_position[2]
            self.nav.goToPose(self.goal_pose)
            i=0
            while not self.nav.isTaskComplete():
                i += 1
                feedback = self.nav.getFeedback()
                distance = feedback.distance_remaining
                print(distance)
                if abs(distance) < 0.3:
                    self.nav.cancelTask()
                    self.success.data = True
                    self.success_callback(self.success)
        
        elif param in ['center', 'center hall']:
            self.goal_pose.pose.position.x = 3.3995699882507324
            self.goal_pose.pose.position.y = 0.3484017252922058
            self.goal_pose.pose.position.z = 0.002471923828125
            self.nav.goToPose(self.goal_pose)
            i=0
            while not self.nav.isTaskComplete():
                i += 1
                feedback = self.nav.getFeedback()
                distance = feedback.distance_remaining
                print(distance)
                if abs(distance) < 0.3:
                    self.nav.cancelTask()
                    self.success.data = True
                    self.success_callback(self.success)
            
        elif param in ['home', 'home position', 'home pose']:
            self.goal_pose.pose.position.x = 0.5027405023574829
            self.goal_pose.pose.position.y = -0.2068520039319992
            self.goal_pose.pose.position.z = 0.11181640625
            self.nav.goToPose(self.goal_pose)
            i=0
            while not self.nav.isTaskComplete():
                i += 1
                feedback = self.nav.getFeedback()
                distance = feedback.distance_remaining
                print(distance)
                if abs(distance) < 0.3:
                    self.nav.cancelTask()
                    self.success.data = True
                    self.success_callback(self.success)
            
            
        elif param in ['stage', 'chalkboard', 'presentation']:
            self.goal_pose.pose.position.x = 6.597291946411133
            self.goal_pose.pose.position.y = 0.39837130904197693
            self.goal_pose.pose.position.z = 0.002471923828125
            self.nav.goToPose(self.goal_pose)
            i = 0
            while not self.nav.isTaskComplete():
                i += 1
                feedback = self.nav.getFeedback()
                distance = feedback.distance_remaining
                print(distance)
                if abs(distance) < 0.3:
                    self.nav.cancelTask()
                    self.success.data = True
                    self.success_callback(self.success)
            
            
        elif param in ['door']:
            self.goal_pose.pose.position.x = 6.545937538146973
            self.goal_pose.pose.position.y = 3.100027561187744
            self.goal_pose.pose.position.z = -0.001434326171875
            self.nav.goToPose(self.goal_pose)
            i=0
            while not self.nav.isTaskComplete():
                i += 1
                feedback = self.nav.getFeedback()
                distance = feedback.distance_remaining
                print(distance)
                if abs(distance) < 0.3:
                    self.nav.cancelTask()
                    self.success.data = True
                    self.success_callback(self.success)
                    
        elif param in ['stop']:
            self.do_stop = True
            self.nav.cancelTask()
            self.success.data = True
            self.success_callback(self.success)
                    
        else:
            prev_pose = self.pose_current.pose.pose
            prev_position = prev_pose.position
            prev_orientation = prev_pose.orientation
            prev_angle = float(euler_from_quaternion([prev_orientation.x, prev_orientation.y, prev_orientation.z, prev_orientation.w])[2])
            target_position = [prev_position.x + 1*float(np.cos(prev_angle)), prev_position.y + 1*float(np.sin(prev_angle)), prev_position.z]
            self.goal_pose.pose.orientation = prev_orientation
            self.goal_pose.pose.position.x = target_position[0]
            self.goal_pose.pose.position.y = target_position[1]
            self.goal_pose.pose.position.z = target_position[2]
            self.nav.goToPose(self.goal_pose)
            i=0
            while not self.nav.isTaskComplete():
                i += 1
                feedback = self.nav.getFeedback()
                distance = feedback.distance_remaining
                print(distance)
                if abs(distance) < 0.3:
                    self.nav.cancelTask()
                    self.success.data = True
                    self.success_callback(self.success) 
                            
    def turn(self, param="around"):
        if param.isdigit():
            prev_pose = self.pose_current.pose.pose
            prev_position = prev_pose.position
            prev_orientation = prev_pose.orientation
            prev_angle = float(euler_from_quaternion([prev_orientation.x, prev_orientation.y, prev_orientation.z, prev_orientation.w])[2])
            target_orientation = quaternion_from_euler(0, 0, prev_angle-float(np.pi/180*float(param)))
            self.goal_pose.pose.position = prev_position
            self.goal_pose.pose.orientation.x = target_orientation[0]
            self.goal_pose.pose.orientation.y = target_orientation[1]
            self.goal_pose.pose.orientation.z = target_orientation[2]
            self.goal_pose.pose.orientation.w = target_orientation[3]
            print(f"turn {param}")
            self.nav.goToPose(self.goal_pose)
            time.sleep(5)
            self.nav.cancelTask()
            
        elif param in ['around']:
            print("turn around")
            prev_pose = self.pose_current.pose.pose
            prev_position = prev_pose.position
            prev_orientation = prev_pose.orientation
            prev_angle = float(euler_from_quaternion([prev_orientation.x, prev_orientation.y, prev_orientation.z, prev_orientation.w])[2])
            target_orientation = quaternion_from_euler(0, 0, prev_angle+float(np.pi*2/3))
            self.goal_pose.pose.position = prev_position
            self.goal_pose.pose.orientation.x = target_orientation[0]
            self.goal_pose.pose.orientation.y = target_orientation[1]
            self.goal_pose.pose.orientation.z = target_orientation[2]
            self.goal_pose.pose.orientation.w = target_orientation[3]
            self.nav.goToPose(self.goal_pose)
            time.sleep(3)
            target_orientation = quaternion_from_euler(0, 0, prev_angle+float(np.pi*4/3))
            self.goal_pose.pose.position = prev_position
            self.goal_pose.pose.orientation.x = target_orientation[0]
            self.goal_pose.pose.orientation.y = target_orientation[1]
            self.goal_pose.pose.orientation.z = target_orientation[2]
            self.goal_pose.pose.orientation.w = target_orientation[3]
            self.nav.goToPose(self.goal_pose)
            time.sleep(3)
            target_orientation = quaternion_from_euler(0, 0, prev_angle+float(np.pi*2))
            self.goal_pose.pose.position = prev_position
            self.goal_pose.pose.orientation.x = target_orientation[0]
            self.goal_pose.pose.orientation.y = target_orientation[1]
            self.goal_pose.pose.orientation.z = target_orientation[2]
            self.goal_pose.pose.orientation.w = target_orientation[3]
            self.nav.goToPose(self.goal_pose)
            time.sleep(5)
            self.nav.cancelTask()
            
        elif param in ["left", "leftward"]:
            prev_pose = self.pose_current.pose.pose
            prev_position = prev_pose.position
            prev_orientation = prev_pose.orientation
            prev_angle = float(euler_from_quaternion([prev_orientation.x, prev_orientation.y, prev_orientation.z, prev_orientation.w])[2])
            target_orientation = quaternion_from_euler(0, 0, prev_angle+float(np.pi*1/2))
            self.goal_pose.pose.position = prev_position
            self.goal_pose.pose.orientation.x = target_orientation[0]
            self.goal_pose.pose.orientation.y = target_orientation[1]
            self.goal_pose.pose.orientation.z = target_orientation[2]
            self.goal_pose.pose.orientation.w = target_orientation[3]
            print("turn left")
            self.nav.goToPose(self.goal_pose)
            time.sleep(5)
            self.nav.cancelTask()
                    
        elif param in ["right", "rightward"]:
            prev_pose = self.pose_current.pose.pose
            prev_position = prev_pose.position
            prev_orientation = prev_pose.orientation
            prev_angle = float(euler_from_quaternion([prev_orientation.x, prev_orientation.y, prev_orientation.z, prev_orientation.w])[2])
            target_orientation = quaternion_from_euler(0, 0, prev_angle-float(np.pi*1/2))
            self.goal_pose.pose.position = prev_position
            self.goal_pose.pose.orientation.x = target_orientation[0]
            self.goal_pose.pose.orientation.y = target_orientation[1]
            self.goal_pose.pose.orientation.z = target_orientation[2]
            self.goal_pose.pose.orientation.w = target_orientation[3]
            print("turn right")
            self.nav.goToPose(self.goal_pose)
            time.sleep(5)
            self.nav.cancelTask()
            
    def led(self, param="both"):
        self.lamp_command.l_command = 0
        self.lamp_command.r_command = 0
        
        if param in ["left", "leftward"]:
            self.lamp_command.l_command = 255
            self.lamp_command.r_command = 0
            
        elif param in ["right", "rightward"]:
            self.lamp_command.l_command = 0
            self.lamp_command.r_command = 255
            
        elif param in ["both", "simulaneously", "on", "activate", "enable", "front", "open"]:
            self.lamp_command.l_command = 255
            self.lamp_command.r_command = 255
            
        elif param in ["off", "deactivate", "kill", "stop", "terminate", "close"]:
            self.lamp_command.l_command = 0
            self.lamp_command.r_command = 0
            
        elif param in ["blink"]:
            for i in range(3):
                time.sleep(0.5)
                self.lamp_command.l_command = 255
                self.lamp_command.r_command = 255
                self.lamp_publisher.publish(self.lamp_command)
                time.sleep(0.5)
                self.lamp_command.l_command = 0
                self.lamp_command.r_command = 0
                self.lamp_publisher.publish(self.lamp_command)
                
        self.lamp_publisher.publish(self.lamp_command)
        time.sleep(1)
    
    def info(self, param):
        if param in ["pose", "position", "location"]:
            self.gpt_feedback.data = f'req:info:{param}*res:position:{self.pose_current.pose.pose.position}'
            
        elif param in ["orientation", "angle"]:
            self.gpt_feedback.data = f'req:info:{param}*res:orientation:{self.pose_current.pose.pose.orientation}, angle:{float(euler_from_quaternion([self.pose_current.pose.pose.orientation.x, self.pose_current.pose.pose.orientation.y, self.pose_current.pose.pose.orientation.z, self.pose_current.pose.pose.orientation.w])[2])} degree*'
            
        elif param in ['day', 'time']:
            now = datetime.datetime.now()
            week = {0:'월', 1:'화', 2:'수', 3:'목', 4:'금', 5:'토', 6:'일'}
            today = 'info : ' + now.strftime("%Y년 %m월 %d일 ") + week[now.weekday()]+'요일 ' + now.strftime("%H시 %M분 %S초")
            del now
            self.gpt_feedback.data = f'req:info:{param}*res:today:{today}*'
    
        elif param in ['weather']:
            url = 'https://weather.naver.com/'
            response = requests.get(url, headers=self.headers)
            if response.status_code == 200:
                html = response.text
                soup = BeautifulSoup(html, 'html.parser')
                temperature = soup.select(".current")[0].text.strip()
                weather = soup.select(".summary")[0].select(".weather")[0].text.strip()
                weathers = 'info : ' + temperature + ' '+ weather + ''
                del response
                del soup
                self.gpt_feedback.data = f'req:info:{param}*res:weathers:{weathers}*'
            else : 
                self.gpt_feedback.data = f'req:info:{param}*res:weathers:{response.status_code}*'

        elif param in ['news']: 
            title_list = []
            for i in range(1, 6):
                url = f'https://news.naver.com/main/main.naver?mode=LSD&mid=shm&sid1=10{i}'               
                response = requests.get(url, headers=self.headers)
                if response.status_code == 200:
                    html = response.text
                    soup = BeautifulSoup(html, 'html.parser') 
                    title = soup.select(".sh_item._cluster_content")[0].text.strip().split('\n')[1].strip()
                    title_list.append(title)
                    del response
                    del soup
                else :
                    self.gpt_feedback.data = f'req:info:{param}*res:news:{response.status_code}*'
            self.gpt_feedback.data = f'req:info:{param}*res:news:{", ".join(title_list)}*'
        
        else :
            keyword = urllib.parse.quote(param)
            url = f'https://ko.wikipedia.org/wiki/{keyword}'
            try :
                response = requests.get(url, headers=self.headers)
                if response.status_code == 200:
                    html = response.text
                    soup = BeautifulSoup(html, 'html.parser') 
                    wiki = soup.select(".mw-parser-output")[0].select('p')[0].text.strip()
                    try :
                        wiki += soup.select(".mw-parser-output")[0].select('p')[1].text.strip()
                    except :
                        pass
                    del response
                    del soup
                    del keyword
                    self.gpt_feedback.data = f'req:info:{param}*res:wiki:{wiki}*'
                else :
                    self.gpt_feedback.data = f'req:info:{param}*res:wiki:{response.status_code}*'
            except :
                self.gpt_feedback.data = f'req:info:{param}*res:wiki:there is no search result*'  
            
        if len(self.gpt_feedback.data)>1:
            self.feedback_publisher.publish(self.gpt_feedback)
        self.gpt_feedback.data = ''
    

              
class ScheduleConductor():
    def __init__(self):
        self.schedule = []
        self.composed_schedule = []
        self.behavior_dict = {}
        self.param_dict = {}
        self.cnt = 0
        
        
    def add_schedule(self, priority=False, device=None, behavior=None, param=None):
        if (device != None or behavior != None or param != None) and behavior != 'think':
            # device, behavior, param = self.check_query(device, behavior, param)
            print('schedule queue :', self.schedule)
            if behavior in ['pause', 'stop']:
                self.schedule = [('turtlebot', 'stop', param)]
                
            elif behavior == 'END':
                self.composed_schedule += self.schedule
                print('current schedule :', self.composed_schedule)
                self.schedule = []
                
            else :
                if priority == True:
                    self.schedule = self.schedule[:self.cnt] + [(device, behavior, param)] + self.schedule[self.cnt:]
                    self.cnt += 1
                    
                else:
                    self.cnt = 0
                    self.schedule = self.schedule + [(device, behavior, param)]       
                    
    
    def check_query(self, device, behavior, param):
        if behavior in self.behavior_dict[device]:
            pass
        
        else:
            device = None
            for key in self.behavior_dict.keys:
                if behavior in self.behavior_dict[key]:
                    device = key
                    
        if param in self.param_dict[behavior]:
            pass
        
        else :
            for key in self.param_dict[key]:
                if param in self.param_dict[key]:
                    behavior = key
                    
        return device, behavior, param
    
    
        
def main(args=None):
    rp.init(args=args)
    
    schedule_maker = ScheduleMaker()
    while (True):
        rp.spin_once(schedule_maker)


if __name__ == '__main__':
    main()