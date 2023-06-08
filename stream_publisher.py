#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
processor.py: Process raw audio signals and publish frequency bins signals_f 

This is the digital twin of what the microphone board does. 
"""

import os
import openai
import rclpy
import whisper
import pyaudio
from torch.cuda.amp import autocast
import soundfile as sf
from scipy.io.wavfile import write
import noisereduce as nr
import numpy as np
from std_msgs.msg import String
current_dir = os.path.dirname(os.path.abspath(__file__))
# sys.path.append(current_dir + "/../../../crazyflie-audio/python/")
from audio_interfaces.msg import Signals
from audio_interfaces_py.messages import (
    read_signals_message,
)
from audio_interfaces_py.node_with_params import NodeWithParams

FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
FILENAME = "debug.wav"

class OpenAIGpt:
    def run(self, msg):
        openai.api_key = "YOUR_API_KEY"
        resopnse = openai.Completion.create(
            model="custom-stt-curie-001", # [text-babbage-001, text-ada-001, text-curie-001, text-davici-003, gpt-3.5-turbo]
            prompt=f"{msg}.",
            temperature=0.3, # [0: 정해진 답변 ~ 1: 창의적인 답변]
            max_tokens=100,
            top_p=1,
            frequency_penalty=0.0,
            presence_penalty=0.0,
            stop=['/n']
        )
        return resopnse.choices[0].text

class AudioSubscriber(NodeWithParams):
    """ Node to subscribe to audio/signals or audio
    """
    PARAMS_DICT = {"file_name": FILENAME}
    
    def __init__(self, Fs):
        super().__init__("audio_subscriber")
        self.Fs = Fs
        self.subscription_signals = self.create_subscription(Signals, "audio/signals", self.listener_callback_signals, 10)
        self.subscription_signals
        self.feedback_subscription = self.create_subscription(String, "/gpt_feedback", self.gpt_callback, 10)
        
        print("Recording started...")
        self.gpt_model = OpenAIGpt()
        self.model = whisper.load_model("medium")
        self.buffered_audio = []
        self.is_activated = False
        self.action_publisher = self.create_publisher(String, "/action_list", 10)
        self.action_string = String()
        self.save_time = 1.0
        
    def normalize_audio(self, signals):
        max_amplitude = np.max(np.abs(signals))  # Get the maximum absolute amplitude
        normalized_signals = signals / max_amplitude  # Normalize the signals
        return normalized_signals

    def listener_callback_signals(self, msg):
        _, signals = read_signals_message(msg)
        # self.get_logger().info(f"listener_callback_signals")

        self.buffered_audio.append(signals)  # Append current chunk to a buffer
    
        # Calculate the total duration of buffered audio
        total_duration = sum(len(chunk) for chunk in self.buffered_audio) / self.Fs
        
        if total_duration >= self.save_time:
            buffered_signals = np.concatenate(self.buffered_audio, axis=0).astype(np.float32) * 0.6
            reduced_signals = nr.reduce_noise(y=buffered_signals, sr=self.Fs)
            normalized_signals = self.normalize_audio(reduced_signals) 
            reduced_signals = nr.reduce_noise(normalized_signals, sr=self.Fs)  
            
            # Output the accumulated audio
            sf.write(FILENAME, reduced_signals, self.Fs)
         
            audio = whisper.load_audio(FILENAME)
            audio = whisper.pad_or_trim(audio)
            mel = whisper.log_mel_spectrogram(audio).to(self.model.device)
            options = whisper.DecodingOptions(task="transcribe", language="en", without_timestamps=True)
            
            with autocast():
                result = whisper.decode(self.model, mel, options).text
                print(result)
                if result != "Thank you" and self.is_activated == True :
                    self.send_to_gpt(result)
                    self.is_activated = False
                    self.save_time = 1.0
                elif self.is_activated == False:
                    if "robot" in result.lower() or "report" in result.lower():
                        self.is_activated = True
                        self.save_time = 2.0
                
            self.buffered_audio = []  # Clear the buffer
            buffered_signals = None
            normalized_signals = None
            reduced_signals = None
            
    def gpt_callback(self, msg) :
        self.send_to_gpt(msg)
            
    def send_to_gpt(self, result):
        try:
            self.action_list = self.gpt_model.run(result)
        except:
            self.action_list = ''
            result = result.lower()
            for action in [item for act in result.split(' and ') for item in act.split(' after ')]:
                if "turn" in action:
                    if "left" in action:
                        self.action_list += "turn:left"
                    elif "right" in action:
                        self.action_list += "turn:right"
                    else:
                        self.action_list += "turn:around"
                elif "red" in action:
                    if "left" in action:
                        self.action_list += "light:left"
                    elif "right" in action:
                        self.action_list += "light:right"
                    elif "off" in action:
                        self.action_list += "light:off"
                    else:
                        self.action_list += "light:both"
                elif "left" in action:
                    self.action_list += "move:left"
                elif "right" in action:
                    self.action_list += "move:right"
                elif "hello" in action:
                    self.action_list += "speak:hello"
                elif "follow" in action:
                    self.action_list += "follow:person"
                self.action_list += "*"
        
        self.action_string.data = self.action_list
        self.action_publisher.publish(self.action_string)
            
def main(args=None):
    rclpy.init(args=args)
    Fs = 44100
    subscriber = AudioSubscriber(Fs)
    rclpy.spin(subscriber)

    # Destroy the node explicitly
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
