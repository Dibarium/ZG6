import pyaudio
import numpy as np
import argparse
from tqdm import tqdm

MAX_OUTPUT_CHANNELS = 5

# Create the parser
parser = argparse.ArgumentParser(description='Select a scenario.')

# Add an argument for the scenario, with a default value of 0
parser.add_argument('device', type=int,  help='Device index number')
parser.add_argument('--duration', type=float, default=0.1, help='Duration in second by speaker (default: 0.1)')
parser.add_argument('--probability', type=float, default=0.3, help='Probability for noise occurence (default: 0.3)')
parser.add_argument('--volume', type=float, default=0.1, help='Volume (default: 0.1)')
parser.add_argument('--loop', type=int, default=20, help='Loop number (default: 20)')
args = parser.parse_args()

p = pyaudio.PyAudio()
device_info = p.get_device_info_by_index(args.device)
print(f"device info: name={device_info['name']}, samplerate={device_info['defaultSampleRate']}")

# Configuration
fs = 44100  # Sampling frequency, 44100 samples per second
output_device_index = args.device  # Default output device, change this to the index of your multi-output sound card

# Generate white noise
samples_size = int(args.duration * fs)
random_buffer = (np.random.randn(samples_size, MAX_OUTPUT_CHANNELS) * args.volume).astype(np.float32)

# Open stream
stream = p.open(format=pyaudio.paFloat32,
                channels=MAX_OUTPUT_CHANNELS,
                rate=fs,
                output=True,
                output_device_index=output_device_index)

speaker_id = -1
for loop in tqdm(range(args.loop)):

    rv = np.random.rand()
    if rv > args.probability:
        buffer = np.zeros((samples_size, MAX_OUTPUT_CHANNELS), dtype=np.float32)
    else:
        buffer = random_buffer

    # Play audio
    stream.write(buffer.tobytes())

# Stop and close the stream
stream.stop_stream()
stream.close()

# Close PyAudio
p.terminate()
