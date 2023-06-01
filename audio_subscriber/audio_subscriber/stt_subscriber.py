#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
processor.py: Process raw audio signals and publish frequency bins signals_f 

This is the digital twin of what the microphone board does. 
"""

import os, sys, time
import rclpy
import whisper, torch
import pyaudio, librosa, wave
from torch.cuda.amp import autocast
import sounddevice as sd
import soundfile as sf
from scipy.io.wavfile import write
import noisereduce as nr
import numpy as np
from scipy import signal

current_dir = os.path.dirname(os.path.abspath(__file__))
# sys.path.append(current_dir + "/../../../crazyflie-audio/python/")
from audio_interfaces.msg import Signals
from audio_interfaces_py.messages import (
    read_signals_message,
)
from audio_interfaces_py.node_with_params import NodeWithParams

SAVE_TIME = 3.0
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
FILENAME = "debug.wav"

class AudioSubscriber(NodeWithParams):
    """ Node to subscribe to audio/signals or audio
    """
    PARAMS_DICT = {"save_time": SAVE_TIME}
    
    def __init__(self, Fs):
        super().__init__("audio_subscriber")
        self.Fs = Fs
        self.subscription_signals = self.create_subscription(
            Signals, "audio/signals", self.listener_callback_signals, 10
        )
        print("Recording started...")
        self.model = whisper.load_model("base")
        self.buffered_audio = []
        
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
        
        if total_duration >= SAVE_TIME:
            buffered_signals = np.concatenate(self.buffered_audio, axis=0).astype(np.float32) * 0.6
            reduced_signals = nr.reduce_noise(y=buffered_signals, sr=self.Fs)
            normalized_signals = self.normalize_audio(reduced_signals) 
            reduced_signals = nr.reduce_noise(normalized_signals, sr=self.Fs)  
            
            # Output the accumulated audio
            sf.write(FILENAME, reduced_signals, self.Fs)
         
            audio = whisper.load_audio(FILENAME)
            audio = whisper.pad_or_trim(audio)
            mel = whisper.log_mel_spectrogram(audio).to(self.model.device)
            options = whisper.DecodingOptions(language="en", without_timestamps=True)
            
            with autocast():
                result = whisper.decode(self.model, mel, options)
                self.get_logger().info(result.text)
                
            self.buffered_audio = []  # Clear the buffer
            buffered_signals = None
            normalized_signals = None
            reduced_signals = None
            
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
