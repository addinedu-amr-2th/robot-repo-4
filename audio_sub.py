import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import pyaudio
import numpy as np

CHUNK = 2**10
RATE = 44100

class AudioSubscriber(Node):
    def __init__(self):
        super().__init__('audio_subscriber')
        self.subscription_ = self.create_subscription(Int16MultiArray, 'audio', self.callback, 10)

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Get available devices
        #devices = self.get_available_devices()
        output_device_index = 1

        self.output_stream = self.audio.open(format=pyaudio.paInt16, channels=1, rate=RATE, output=True,
                                             output_device_index=output_device_index)

    # def get_available_devices(self):
    #     devices = []
    #     for i in range(self.audio.get_device_count()):
    #         device_info = self.audio.get_device_info_by_index(i)
    #         devices.append(device_info)
    #         self.get_logger().info(f"Device {i}: {device_info['name']}")
    #     return devices

    def callback(self, msg):
        #print(msg)
        #print(msg.data)
        #data = np.frombuffer(msg.data, dtype=np.int16)
        data = np.array(msg.data, dtype=np.int16)
         
        print(int(np.average(np.abs(data))))
        self.output_stream.write(data.tobytes())

    def cleanup(self):
        self.output_stream.stop_stream()
        self.output_stream.close()
        self.audio.terminate()

def main(args=None):
    rp.init(args=args)

    audio_subscriber = AudioSubscriber()

    rp.spin(audio_subscriber)
    audio_subscriber.cleanup()

    audio_subscriber.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
