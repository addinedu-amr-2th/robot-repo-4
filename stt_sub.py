import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import speech_recognition as sr
import pyaudio
import numpy as np
from scipy.io import wavfile

CHUNK = 2**10
RATE = 44100
THRESHOLD = 1000
THRESHOLD_out = 100
VOLUME = []
VOLUME1 = []
VOLUME2 = []

class AudioSubscriber(Node):
    def __init__(self):
        super().__init__('stt_subscriber')
        self.subscription_ = self.create_subscription(Int16MultiArray, 'audio', self.callback, 10)
        self.r = sr.Recognizer()

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        output_device_index = 1

        self.output_stream = self.audio.open(format=pyaudio.paInt16, channels=1, rate=RATE, output=True,
                                             output_device_index=output_device_index)

        self.is_recording = False
        self.recorded_data = []

    def callback(self, msg):
        global VOLUME, VOLUME1, VOLUME2
        data = np.array(msg.data, dtype=np.int16)
        sound_level = int(np.average(np.abs(data)))
        VOLUME.append(sound_level)
        if len(VOLUME) >=41:
            VOLUME1 = VOLUME[0:20]
            VOLUME2 = VOLUME[20:40]
            VOLUME.pop(0)
            #print(len(VOLUME))
            
        #print(np.average(VOLUME1), np.average(VOLUME2))

        # Print sound level
        #print(sound_level)

        # Output sound
        # self.output_stream.write(data.tobytes())

        if not self.is_recording and sound_level >= np.average(VOLUME)*3:
            # Start recording
            self.is_recording = True
            self.recorded_data.append(data)
            print("Recording started.")
            #print(self.recorded_data)


        elif self.is_recording:
            # Continue recording
            self.recorded_data.append(data)
            #print(self.recorded_data)

            if np.average(VOLUME2)*3 < np.average(VOLUME):
                # Stop recording and perform STT
                self.is_recording = False
                print("Recording stopped.")

                audio_data = np.concatenate(self.recorded_data)  # Combine recorded data
                audio_data = audio_data.astype(np.int16)  # Convert data to int16 type
                self.recorded_data=[]
                # Save audio data as WAV file
                wavfile.write('audio.wav', RATE, audio_data)

                # Load WAV file and create AudioData object
                with sr.AudioFile('audio.wav') as source:
                    audio_output = self.r.record(source)

                try:
                    text = self.r.recognize_google(audio_output)  # Use Google Speech Recognition API
                    print(f"You said: {text}")
                    
                except sr.UnknownValueError:
                    print("Sorry, I couldn't understand audio.")
                except sr.RequestError as e:
                    print(f"Sorry, an error occurred. {e}")

def main(args=None):
    rclpy.init(args=args)
    audio_subscriber = AudioSubscriber()
    rclpy.spin(audio_subscriber)
    audio_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
