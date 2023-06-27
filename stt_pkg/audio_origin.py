import pyaudio
import numpy as np

CHUNK = 2**10
RATE = 44100

#########################################
# Initialize PyAudio
audio = pyaudio.PyAudio()

def get_available_devices():
    devices = []
    for i in range(audio.get_device_count()):
        device_info = audio.get_device_info_by_index(i)
        devices.append(device_info)
        print(f"Device {i}: {device_info['name']}")
    return devices

# Get available devices
devices = get_available_devices()
#######################################

p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16, channels=1, rate=RATE, input=True,
               frames_per_buffer=CHUNK, input_device_index=0)

output_stream = p.open(format=pyaudio.paInt16, channels=1, rate=RATE, output=True, output_device_index=1)

try:
    # 스피커 출력을 위한 설정
    

    while True:
        data = np.frombuffer(stream.read(CHUNK), dtype=np.int16)
        print(int(np.average(np.abs(data))))

        # 입력된 소리를 스피커로 출력
        output_stream.write(data.tobytes())

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    stream.stop_stream()
    stream.close()
    output_stream.stop_stream()
    output_stream.close()
    p.terminate()