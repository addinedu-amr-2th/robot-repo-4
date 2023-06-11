from gtts import gTTS
import pydub
from pydub.playback import play

def save_text_as_mp3(text, file_path):
    tts = gTTS(text=text, lang='en')
    tts.save(file_path)

def convert_mp3_to_wav(mp3_file_path, wav_file_path):
    audio = pydub.AudioSegment.from_mp3(mp3_file_path)
    audio.export(wav_file_path, format="wav")

def play_wav_file(file_path):
    audio = pydub.AudioSegment.from_wav(file_path)
    play(audio)

text = "Hello, world this is Yun, How are you doing?"
mp3_file_path = "/home/yun/output.mp3"
wav_file_path = "/home/yun/output.wav"

save_text_as_mp3(text, mp3_file_path)
convert_mp3_to_wav(mp3_file_path, wav_file_path)
play_wav_file(wav_file_path)
