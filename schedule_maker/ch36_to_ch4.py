import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32

# schduler에서 robotarm으로
class Ch36toCh4(Node):
    def __init__(self):
        super().__init__('ch36_to_ch4')
        self.success = Bool()
        self.angle = Float32()
        self.command_subscriber = self.create_subscription(String, 'gpt_cmd', self.command_callback, 10)
        self.yolo_target_subscriber = self.create_subscription(String, 'yolo_target', self.yolo_target_callback, 10)
        self.readfile_timer = self.create_timer(0.5, self.timer_callback)
        self.success_publisher = self.create_publisher(Bool, 'success', 10)
        self.angle_publisher = self.create_publisher(Float32, 'angle', 10)
        # self.param = String()
        # self.param_publisher = self.create_publisher(String, 'param_check', 10)
        
        
    def timer_callback(self):
        with open('/home/ane4/amr-repository/amr_ws/pinkbot/src/schedule_maker/schedule_maker/success.txt', "r") as file:
            success = file.read()
            if success != None and len(success) >= 1:
                if success == 't':
                    self.success.data = True
                    print(self.success.data)
                    self.success_publisher.publish(self.success)
                elif success == 'f':
                    self.success.data = False
                    print(self.success.data)
                    self.success_publisher.publish(self.success)
                with open('/home/ane4/amr-repository/amr_ws/pinkbot/src/schedule_maker/schedule_maker/success.txt', "w") as file:
                    file.write("")
                    
        with open('/home/ane4/amr-repository/amr_ws/pinkbot/src/schedule_maker/schedule_maker/angle.txt', "r") as file:
            angle = file.read()
            if angle != None and len(angle) >= 1:
                self.angle.data = float(angle)
                self.angle_publisher.publish(self.angle)
                print(self.angle.data)
                with open('/home/ane4/amr-repository/amr_ws/pinkbot/src/schedule_maker/schedule_maker/angle.txt', "w") as file:
                    file.write("")
                    
        # with open('/home/ane4/amr-repository/amr_ws/pinkbot/src/schedule_maker/schedule_maker/param.txt', "r") as file:
        #     param = file.read()
        #     if param != None and len(param) >= 1:
        #         self.param.data = param
        #         self.param_publisher.publish(self.param)
        #         print(self.param.data)
        #     with open('/home/ane4/amr-repository/amr_ws/pinkbot/src/schedule_maker/schedule_maker/param.txt', "w") as file:
        #         file.write("")
        
    
    def command_callback(self, msg):
        with open('/home/ane4/amr-repository/amr_ws/pinkbot/src/schedule_maker/schedule_maker/command.txt', "w") as file:
            if msg.data!=None and len(msg.data)>2:
                print(msg.data)
                file.write(msg.data)
            
    def yolo_target_callback(self, msg):
        if msg.data!=None and len(msg.data)>1:
            with open('/home/ane4/amr-repository/amr_ws/pinkbot/src/schedule_maker/schedule_maker/yolo_target.txt', "w") as file:
                print(msg.data)
                file.write(msg.data)
        
        
def main(args=None):
    rp.init(args=args)
    ch36_to_ch4 = Ch36toCh4()
    rp.spin(ch36_to_ch4)
    ch36_to_ch4.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()