import pyaudio
import numpy as np
import argparse
from tqdm import tqdm

MAX_OUTPUT_CHANNELS = 5

# Create the parser
parser = argparse.ArgumentParser(description='Select a scenario.')

# Add an argument for the scenario, with a default value of 0
parser.add_argument('device', type=int,  help='Device index number')
parser.add_argument('--duration', type=float, default=2., help='Duration in second by speaker (default: 2)')
parser.add_argument('--scenario', type=int, default=0, help='Scenario number (default: 0). Select 0 for a sequential shift of the speaker position and 1 for a random choice')
parser.add_argument('--loop', type=int, default=1, help='Loop number (default: 1)')
args = parser.parse_args()

p = pyaudio.PyAudio()
device_info = p.get_device_info_by_index(args.device)
print(f"device info: name={device_info['name']}, samplerate={device_info['defaultSampleRate']}")

# Configuration
fs = 44100  # Sampling frequency, 44100 samples per second
volume = 0.5  # Volume, between 0.0 and 1.0
output_device_index = args.device  # Default output device, change this to the index of your multi-output sound card

# Generate white noise
samples_size = int(args.duration * fs)
random_buffer = (np.random.randn(samples_size) * volume).astype(np.float32)

# Open stream
stream = p.open(format=pyaudio.paFloat32,
                channels=MAX_OUTPUT_CHANNELS,
                rate=fs,
                output=True,
                output_device_index=output_device_index)

speaker_id = -1
n = 0
for loop in tqdm(range(args.loop)):

    if args.scenario == 0:
        speaker_id = (speaker_id+1)%5
    if args.scenario == 1:
        speaker_id = np.random.randint(0, 5)
    if args.scenario == 2: #scenario qui utilise les 2 enceintes de gauche (gauche gauche, gauche centre, gauche gauche, gauche centre)
        if n == 0:          #si les speaker_id sont répartis de gauche à droite
            speaker_id = 0
            n = 1
        if n == 1:
            speaker_id = 1
            n = 0

    samples = np.zeros((samples_size, MAX_OUTPUT_CHANNELS), dtype=np.float32)
    samples[:, speaker_id] = random_buffer

    # Play audio
    stream.write(samples.tobytes())

# Stop and close the stream
stream.stop_stream()
stream.close()

# Close PyAudio
p.terminate()
