import rclpy as rp
from rclpy.node import Node
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from minibot_interfaces.msg import LampCommand
from geometry_msgs.msg import PoseWithCovarianceStamped

class ScheduleMaker(Node):
    '''
    스케쥴을 짜서 액션을 순서대로 내보내는 부분
    '''
    def __init__(self):
        super().__init__('schedule_maker')
        self.subscription = self.create_subscription(String, '/action_list', self.action_callback, 10)
        self.subscription
        self.gpt_feedback = self.create_publisher(String, '/gpt_feedback', 10)
        self.twist_publisher = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.lamp_publisher = self.create_publisher(LampCommand, '/minibot_io_controller/set_lamp', 10)
        self.pos_subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.pos_subscription
        self.twist = Twist()
        self.feedback = String()
        self.lamp_command = LampCommand()
        self.pose_current = PoseWithCovarianceStamped()
        
    def action_callback(self, msg):
        msg = msg.data
        print(msg)
        if len(msg) > 2 and ':' in msg:
            if '*' not in msg :
                action, param = msg.split(':')
                param = param.lower()
                self.do_action(action, param)
            else :
                for schedule in msg.split('*') :
                    if len(schedule) > 2 :
                        action, param = schedule.split(':')
                        action = action.lower()
                        param = param.lower()
                        self.do_action(action, param)
    
    def pose_callback(self, msg):
        self.pose_current = msg
    
    def do_action(self, func, param):
        '''
            해당하는 액션을 내보내는 부분
        '''
        if func in ["move", "moving", "go"]:
            self.move(param)
        elif func in ["turn", "spin", "roate", "pivot" "swivel", "twirl", "revolve", "wheel", "gyrate", "pirouette"]:
            self.turn(param)
        elif func in ["light", "led", "eye", "bulb", "red"]:
            self.led(param)
        elif func in ["blink"]:
            self.led("blink")
        elif func in ["info", "explain", "information"]:
            self.info(param)
        elif func in ["speak", "talk", "tell", "say"]:
            pass
        elif func in ["buzz", "alert", "buzzer"]:
            self.buzzer(param)
        else :
            self.fix_gpt()
    
    def move(self, param="forward"):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        if param in ["front", "forward", "ahead", "infront"]:
            self.twist.linear.x = 5.0
            self.twist.angular.z = 0.0
        elif param in ["back", "backward", "behind", "reverse"]:
            self.twist.linear.x = -5.0
            self.twist.angular.z = 0.0
        elif param in ["left", "leftward"]:
            self.twist.linear.x = 5.0
            self.twist.angular.z = 2.0
        elif param in ["right", "rightward"]:
            self.twist.linear.x = 5.0
            self.twist.angular.z = -2.0
        elif param in ["stop", "halt", "stand", "freeze", "stay"]:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        self.twist_publisher.publish(self.twist)
        time.sleep(1)
        
    def turn(self, param="around"):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        if param in ["left", "leftward"]:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 2.0
        elif param in ["right", "rightward"]:
            self.twist.linear.x = 0.0
            self.twist.angular.z = -2.0
        elif param in ["stop", "halt", "stand", "freeze", "stay"]:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        elif param in ["back", "u-turn", "reverse", "opposite"]:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 3.0
        elif param in ["around"]:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 5.0
        self.twist_publisher.publish(self.twist)
        time.sleep(1)
        
    def buzzer(self, param):
        time.sleep(1)
        pass
        
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
                time.sleep(0.5)
                self.lamp_command.l_command = 0
                self.lamp_command.r_command = 0
        self.lamp_publisher.publish(self.lamp_command)
        time.sleep(1)
        
    def info(self, param="pose"):
        self.feedback = ''
        if param in ["pose", "position", "location", "loc", "orientation"]:
            pose = str(self.pose_current.pose.pose.position)
            orientation = str(self.pose_current.pose.pose.orientation)
            self.feedback = pose + ' and ' + orientation
        
        self.gpt_feedback.publish(self.feedback)
        time.sleep(1)
    
    def fix_gpt(self):
        '''
        gpt 모델이 원하는 형식의 답을 내놓지 않으면 설득시키는 프롬프트를 보내주는 부분
        '''
        self.feedback = ''
        self.twist.linear.x = 0.0
        self.twist.angular.z = 200.0
        self.feedback = "로봇에 탑재된 프로그램으로써 너가 사용할 수 있는 함수명은 move, turn, info, led야"
        self.gpt_feedback.publish(self.feedback)
        time.sleep(1)
                        
class ActionMaker(Node):
    '''
    해당하는 액션을 받아서 수행하는 부분
    '''
    def __init__(self):
        super().__init__('action_maker')
        
    def goto(self):
        pass
    
    def follow(self):
        pass
    
    def speak(self):
        pass
    
    def robot_arm(self):
        pass
        
def main(args=None):
    rp.init(args=args)
    
    schedule_maker = ScheduleMaker()
    rp.spin(schedule_maker)
    
    schedule_maker.destroy_node()
    rp.shutdown()
    
if __name__ == '__main__':
    main()