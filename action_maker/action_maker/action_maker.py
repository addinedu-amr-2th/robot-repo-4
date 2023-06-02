import rclpy as rp
from rclpy.node import Node
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ActionMaker(Node):
    def __init__(self):
        super().__init__('action_maker')
        self.subscription = self.create_subscription(String, '/action_list', self.callback, 10)
        self.twist_publisher = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.feedback_publisher = self.create_publisher(String, '/feedback', 10)
        self.subscription
        self.idx = 0 # 남은 스케쥴 GPT에 전달용
        self.feedback = String()
        self.twist = Twist()
        
    def callback(self, msg):
        msg = msg.data
        self.idx = 0
        if len(msg) <= 1 or ': ' not in msg :
            self.feedback.data = "Finished"
            self.feedback_publisher.publish(self.feedback)
        elif '*' not in msg :
            func, param = msg.split(': ')
            self.do_action(func, param)
            self.feedback.data = "Finished"
            self.feedback_publisher.publish(self.feedback)
            
        else :
            action_list = msg.split('*')
            for action in action_list :
                if len(action) == '#' :
                    break
                elif len(action) >= 3 :
                    func, param = action.split(': ')
                    self.do_action(func, param)
                self.idx += 1
            self.feedback.data = "Finished"
            self.feedback_publisher.publish(self.feedback)
    
    def do_action(self, func, param):
        if func.lower() in ["move", "moving", "go"]:
            self.move(param.lower())
        elif func.lower() in ["turn", "spin", "roate", "pivot" "swivel", "twirl", "revolve", "wjee;", "gyrate", "pirouette"]:
            self.turn(param.lower())
        else :
            self.fix_gpt()
    
    def fix_gpt(self):
        '''
        gpt 모델이 원하는 형식의 답을 내놓지 않으면 설득시키는 프롬프트를 보내주는 부분
        '''
        fix_text = "너는 모바일 로봇에 ~"
        
    def move(self, param):
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
        
    def goto(self):
        pass
    
    def turn(self, param):
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
        elif param in ["around"]:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 5.0
        self.twist_publisher.publish(self.twist)
        
    def info(self):
        pass
    
    def speak(self):
        pass
    
    def led(self):
        pass
    
    def buzzer(self):
        pass
    
    def robot_arm(self):
        pass
        
def main(args=None):
    rp.init(args=args)
    
    action_maker =ActionMaker()
    rp.spin(action_maker)
    
    action_maker.destroy_node()
    rp.shutdown()
    
if __name__ == '__main__':
    main()