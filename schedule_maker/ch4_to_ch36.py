import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32

# robotarm에서 schduler로
class Ch4toCh36(Node):
    def __init__(self):
        super().__init__('ch4_to_ch36')
        self.command = String()
        self.command_publisher = self.create_publisher(String, 'gpt_cmd', 10)
        self.readfile_timer = self.create_timer(0.5, self.timer_callback)
        self.yolo_target = String()
        self.yolo_target_publisher = self.create_publisher(String, 'yolo_target', 10)
        self.success_subscriber = self.create_subscription(Bool, 'success', self.success_callback, 10)
        self.angle_subscriber = self.create_subscription(Float32, 'angle', self.angle_callback, 10)
        # self.param_check_sub = self.create_subscription(String, 'param_check', self.param_check, 10)
        
    def timer_callback(self):
        with open('/home/ane4/amr-repository/amr_ws/pinkbot/src/schedule_maker/schedule_maker/command.txt', "r") as file:
            command = file.read()
            if command != None and len(command) > 1:
                self.command.data = command
                print(command)
                self.command_publisher.publish(self.command)
                with open('/home/ane4/amr-repository/amr_ws/pinkbot/src/schedule_maker/schedule_maker/command.txt', "w") as file:
                    file.write("")
                    
        with open('/home/ane4/amr-repository/amr_ws/pinkbot/src/schedule_maker/schedule_maker/yolo_target.txt', "r") as file:
            yolo_target = file.read()
            if yolo_target != None and len(yolo_target) > 1:
                self.yolo_target.data = yolo_target
                print(yolo_target)
                self.yolo_target_publisher.publish(self.yolo_target)
                with open('/home/ane4/amr-repository/amr_ws/pinkbot/src/schedule_maker/schedule_maker/yolo_target.txt', "w") as file:
                    file.write("")

    def success_callback(self, msg):
        with open('/home/ane4/amr-repository/amr_ws/pinkbot/src/schedule_maker/schedule_maker/success.txt', "w") as file:
            if msg.data == True:
                print(msg.data)
                file.write("t")
            elif msg.data == False:
                print(msg.data)
                file.write("f")
    
    def angle_callback(self, msg):
        with open('/home/ane4/amr-repository/amr_ws/pinkbot/src/schedule_maker/schedule_maker/angle.txt', "w") as file:
            if msg.data != None:
                print(msg.data)
                file.write(str(msg.data))
                
    # def param_check(self, msg):
    #     with open('/home/ane4/amr-repository/amr_ws/pinkbot/src/schedule_maker/schedule_maker/param.txt', "w") as file:
    #         if msg.data != None and len(msg.data)>0:
    #             print(msg.data)
    #             file.write(msg.data)

def main(args=None):
    rp.init(args=args)
    ch4_to_ch36 = Ch4toCh36()
    rp.spin(ch4_to_ch36)
    ch4_to_ch36.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()