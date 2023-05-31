#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
processor.py: Process raw audio signals and publish frequency bins signals_f 

This is the digital twin of what the microphone board does. 
"""

import os
import sys
import time

import numpy as np
from scipy import signal
import sounddevice as sd
import rclpy
from scipy.io.wavfile import write
import noisereduce as nr

current_dir = os.path.dirname(os.path.abspath(__file__))
# sys.path.append(current_dir + "/../../../crazyflie-audio/python/")
from audio_interfaces.msg import Signals
from audio_interfaces_py.messages import (
    read_signals_message,
)
from audio_interfaces_py.node_with_params import NodeWithParams
from audio_subscriber.parameters import TUKEY_ALPHA

SAVE_TIME = 2.0

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
        self.mic_positions = None
        self.buffered_audio = []
        
    def normalize_audio(self, signals):
        max_amplitude = np.max(np.abs(signals))  # Get the maximum absolute amplitude
        normalized_signals = signals / max_amplitude  # Normalize the signals
        return normalized_signals

    def listener_callback_signals(self, msg):
        self.mic_positions, signals = read_signals_message(msg)
        self.get_logger().info(f"listener_callback_signals")
        
        # n_buffer = signal.shape[0]
        self.buffered_audio.append(signals)  # Append current chunk to a buffer
    
        # Calculate the total duration of buffered audio
        total_duration = sum(len(chunk) for chunk in self.buffered_audio) / self.Fs
        
        if total_duration >= SAVE_TIME:
            # Concatenate buffered chunks
            buffered_signals = np.concatenate(self.buffered_audio, axis=0)     
            reduced_signals = nr.reduce_noise(y=buffered_signals, sr=self.Fs)
            normalized_signals = self.normalize_audio(reduced_signals) * 0.5
            reduced_signals = nr.reduce_noise(normalized_signals, sr=self.Fs)
            # Output the accumulated audio
            sd.play(reduced_signals, samplerate=self.Fs)
            self.buffered_audio = []  # Clear the buffer
            # time.sleep(SAVE_TIME)
            buffered_signals = None
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
