import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import pyaudio
import numpy as np

CHUNK = 2**10
RATE = 44100

class AudioPublisher(Node):
    def __init__(self):
        super().__init__('audio_publisher')
        self.publisher_ = self.create_publisher(Int16MultiArray, 'audio', 10)

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Get available devices
        #devices = self.get_available_devices()
        input_device_index = 0

        self.stream = self.audio.open(format=pyaudio.paInt16, channels=1, rate=RATE, input=True,
                                      frames_per_buffer=CHUNK, input_device_index=input_device_index)

    # def get_available_devices(self):
    #     devices = []
    #     for i in range(self.audio.get_device_count()):
    #         device_info = self.audio.get_device_info_by_index(i)
    #         devices.append(device_info)
    #         self.get_logger().info(f"Device {i}: {device_info['name']}")
    #     return devices

    def publish_audio(self):
        try:
            while True:
                data = np.frombuffer(self.stream.read(CHUNK), dtype=np.int16)
                print(int(np.average(np.abs(data))))

                msg = Int16MultiArray()
                msg.data = data.tolist()
                self.publisher_.publish(msg)
                

        except KeyboardInterrupt:
            self.get_logger().info("Stopped by user")

        finally:
            self.stream.stop_stream()
            self.stream.close()
            self.audio.terminate()

def main(args=None):
    rclpy.init(args=args)

    audio_publisher = AudioPublisher()
    audio_publisher.publish_audio()

    audio_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
