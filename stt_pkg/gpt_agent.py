import rclpy
from rclpy.node import Node
import openai, json, os
from std_msgs.msg import String
import re

API_PATH = '/home/ane4/Downloads/api.txt'
MODEL = 'davinci:ft-personal-2023-06-25-01-27-41'
USE_FEWSHOT = False

class ChatGPT(Node):
    def __init__(self):
        super().__init__('gpt_agent')
        self.few_shot_prompt = [{"role": "system", "content": "You are GOAP AI that installed in turtlebot4 which have a robot arm constist of 3 motors and YOLO with COCO dataset. You should make action plan for the robot that satisfies the user's request"},
                              {"role": "system", "content": "you must make plan with short words(device1:function1: params1* device2:function2: params2* function3: params3* ..."},
                              {"role": "system", "content": "function must be one word which is verb and a param must be one word of object name which is naun except speak or think function because the program input param into function directly"},
                              {"role": "system", "content": "function list: [goto, turn, find, look_at, follow, grip, dock, undock, release, emotion, speak, think, get_info, set_mem, move_hand, set_arm_3motors]"},
                              {"role": "system", "content": "**important format** for example if user request is hungry, then you must make actions like this -> turtlebot:priority:high*turtlebot:emotion:sad*turtlebot:speak:I'm sorry to hear that you are hungry*turtlebot:think:I should find some food*turtlebot:find:food*turtlebot:goto:food*robot_arm:grib...."},
                              {"assistant": "Can you dance?", "content":"turtlebot:priority:low* turtlebot:emotion:curious* turtlebot:speak:I'm robot so I can not dance but I'll do my best* turtlebot:turn:left* turtlebot:turn:right* turtlebot:turn:left* turtlebot:turn:right* robot_arm:motion:happy END"},
                              {"assistant": "Hello!", "content":"turtlebot:priority:low* turtlebot:emotion:Happy* turtlebot:look:person* turtlebot:speak:Hello, master!* robot_arm:motion:bow END"}]
        self.schedule_list = []
        self.model = MODEL # gpt-3.5-turbo or gpt-4
        self.longterm_memory = '' # 함수를 통해 GPT가 스스로 설정하는 중요한 기억
        self.memory_size = 0 # 단기기억 대화내역 크기
        self.object_on_scene = ''
        self.conversation_history=[] # 단기기억 대화 내역
        self.feedback_subscription = self.create_subscription(String, 'agent_feedback_text', self.input_request, 10)
        self.text_subscription = self.create_subscription(String, 'stt_text', self.input_request, 10)
        self.vision_subscription = self.create_subscription(String, 'object_on_scene', self.set_vision, 10)
        self.schedule_publisher = self.create_publisher(String, 'schedu"le', 10)
        self.schedule = String()

    def input_request(self, msg):
        request = msg.data
        if request != None or len(request)>1 :
            input = []
            if USE_FEWSHOT == True:
                input += self.few_shot_prompt
            if self.longterm_memory!=None and len(self.longterm_memory)>0:
                input += [{"role": "assistant", "content": f"longterm_memory: {self.longterm_memory}"}]
            if len(self.conversation_history)>0:
                input += self.conversation_history
            if self.object_on_scene != None and len(self.object_on_scene)>1:
                input += [{"role": "assistant", "content": f"objects on scene: {self.object_on_scene}"}]
            input += [{"role": "user", "content": request}]

            response = openai.ChatCompletion.create(
                model=self.model,
                messages=input,
                temperature=0.3,
                max_tokens=300
            )

            response_message = response["choices"][0]["message"]["content"]
            self.conversation_history += [{"role": "assistant", "content": request}, {"role": "assistant", "content": response_message}]
            if len(self.conversation_history) > 2*self.memory_size:
                self.conversation_history = self.conversation_history[2:]
            response_message = re.sub(r"\d+\.\s", "*", response_message).replace("\'", "").replace("::", ":").replace("[","")
            response_message = re.sub(r"- ", "*", response_message).replace('\"', '').replace("*:",":").replace("]","")
            print(response_message)
            self.schedule.data = response_message
            self.schedule_publisher.publish(self.schedule)
            self.object_on_scene = ''
            
    def set_vision(self, msg):
        self.object_on_scene = msg.data

    def feedback(self, feedback):
        if feedback!= None or len(feedback)>1 :
            self.input_request(f'feedback message - {feedback}')
            
class Openai(Node):
    def __init__(self):
        super().__init__('gpt_agent')
        self.few_shot_prompt=r'''
As block coding GOAP AI installed on turtlebot, make robot's plan that satisfy the user's request with like these\n
user request: Can you dance? \#\# GOAP: turtlebot:priority:low* turtlebot:emotion:curious* turtlebot:speak:I'm robot so I can not dance but I'll do my best* turtlebot:turn:left* turtlebot:turn:right* turtlebot:turn:left* turtlebot:turn:right* robot_arm:motion:happy END\n
user request: What is todays weather? \#\# GOAP: turtlebot:priority:high* turtlebot:speak:I will search weather for you* turtlebot:req:weather END system:info:rainy* turtlebot:emotion:sad* turtlebot:speak:Today is rainy you should take umbrella* turtlebot:find:umbrella* turtlebot:goto:umbrella* robot_arm:grip:umbrella* turtlebot:find:persion* turtlebot:goto:person* robot_arm:release:umbrella* robot_arm:motion:happy END\n
'''
        self.schedule_list = []
        self.model = MODEL # curie or davinci
        self.memory_size = 0 # 단기기억 대화내역 크기
        self.conversation_history=[] # 단기기억 대화 내역
        self.feedback_subscription = self.create_subscription(String, 'agent_feedback_text', self.input_request, 10)
        self.text_subscription = self.create_subscription(String, 'stt_text', self.input_request, 10)
        self.schedule_publisher = self.create_publisher(String, 'schedule', 10)
        self.schedule = String()

    def input_request(self, msg):
        request = msg.data
        if request != None or len(request)>1:
            input = ''
            if USE_FEWSHOT== True:
                input += self.few_shot_prompt
            if self.memory_size>0 and len(self.conversation_history)>0:
                input += ''.join(self.conversation_history) + '\n'
                # input = 
            # input += f"user request: {request} -> GOAP: "
            if len(MODEL)>7:
                input += f"{request} \n\n##\n\n"

            response = openai.Completion.create(
                engine=self.model,
                prompt=input,
                temperature=0.2,
                max_tokens=300,
                top_p=1.0,
                frequency_penalty=0.3,
                presence_penalty=0.3,
                stop='END'
            )
            response_message = response["choices"][0]["text"]
            # self.conversation_history += f"user request: {request} ## GOAP: {response_message} END"
            if len(self.conversation_history) > self.memory_size:
              self.conversation_history = self.conversation_history[1:]
            print(response_message)
            response_message = re.sub(r"\d+\.\s", "*", response_message).replace("\'", "").replace("::", ":").replace("[","")
            response_message = re.sub(r"- ", "*", response_message).replace('\"', '').replace("*:",":").replace("]","").replace("#","")
            self.schedule.data = response_message
            self.schedule_publisher.publish(self.schedule)
    
def main(args=None):
    with open(API_PATH, 'r') as api_file:
        api_key = api_file.read()
        print('api_key:', api_key)
        openai.api_key = api_key
        
    rclpy.init(args=args)
    
    if MODEL == 'gpt-3.5-turbo' or MODEL == 'gpt-4':
        agent = ChatGPT()
    else:
        agent = Openai()
        
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()