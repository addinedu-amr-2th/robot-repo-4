from gtts import gTTS
import pydub
from pydub.playback import play
import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String
from playsound import playsound
import multiprocessing


class TTS(Node):
    def __init__(self):
        super().__init__('tts_publisher')
        self.tts_text_subscription = self.create_subscription(String, 'tts_text', self.tts_callback, 10)
        self.music_subscription = self.create_subscription(String, 'music', self.music_callback, 10)
        self.title = 'happy'
        self.play = multiprocessing.Process(target=playsound, args=(f'./{self.title}.mp3',))
        self.play.daemon = True
        
    def save_text_as_mp3(self, text, file_path):
        tts = gTTS(text=text, lang='en')
        tts.save(file_path)

    def convert_mp3_to_wav(self, mp3_file_path, wav_file_path):
        audio = pydub.AudioSegment.from_mp3(mp3_file_path)
        audio.export(wav_file_path, format="wav")

    def play_wav_file(self, file_path):
        audio = pydub.AudioSegment.from_wav(file_path)
        play(audio)
        
        
    def tts_callback(self, msg):
        tts_text = msg.data
        mp3_file_path = "./output.mp3"
        wav_file_path = "./output.wav"
        self.save_text_as_mp3(tts_text, mp3_file_path)
        self.convert_mp3_to_wav(mp3_file_path, wav_file_path)
        self.play_wav_file(wav_file_path)
        
    def music_callback(self, msg):
        self.title = msg.data.replace(' ', '')
        print(self.title)
        if self.title != None and len(self.title) > 0:
            try:
                if self.title != 'stop':
                    self.play.start()
                else:
                    self.play.terminate()
            except:
                print('there is no such music')

def main(args=None):
    rp.init(args=args)
    tts = TTS()
    rp.spin(tts)
    tts.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()