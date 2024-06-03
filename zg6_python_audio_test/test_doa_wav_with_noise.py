import pyaudio
import warnings
import numpy as np
import argparse
from scipy.io import wavfile
from tqdm import tqdm


MAX_OUTPUT_CHANNELS = 5

# Create the parser
parser = argparse.ArgumentParser(description='Select a scenario.')

# Add an argument for the scenario, with a default value of 0
parser.add_argument('device', type=int,  help='Device index number (use script show_outputs to obtain the list of possible choice)')
parser.add_argument('--noise', type=float, default=0.005, help='Noise volume (default: 0.005)')
parser.add_argument('--scenario', type=int, default=0, help='Scenario number (default: 0). Select 0 for a sequential shift of the speaker position and 1 for a random choice')
parser.add_argument('--loop', type=int, default=1, help='Loop number (default: 1)')
parser.add_argument('--wavfile', type=str, default="./wav/voice1.wav", help='Wavefile to use (default: /wav/voice1.wav)')
args = parser.parse_args()

# load audio sound
with warnings.catch_warnings():
    warnings.simplefilter("ignore", wavfile.WavFileWarning)
    fs, audio_buffer = wavfile.read(args.wavfile)
    audio_buffer = audio_buffer.astype(float) / np.max(np.abs(np.ravel(audio_buffer))) 

output_device_index = args.device  # Default output device, change this to the index of your multi-output sound card
scenario_list = ["sequential", "random"]

# generate random noise
samples_size = len(audio_buffer)
random_buffer = (np.random.randn(samples_size, MAX_OUTPUT_CHANNELS) * args.noise).astype(np.float32)

# Open stream
p = pyaudio.PyAudio()
device_info = p.get_device_info_by_index(args.device)
print(f"device info: name={device_info['name']}, samplerate={device_info['defaultSampleRate']}")
stream = p.open(format=pyaudio.paFloat32,
                channels=MAX_OUTPUT_CHANNELS,
                rate=fs,
                output=True,
                output_device_index=output_device_index)

speaker_id = -1

for loop in tqdm(range(args.loop)):

    if args.scenario == 0:
        speaker_id = (speaker_id+1)%5
    if args.scenario == 1:
        speaker_id = np.random.randint(0, 5)

    buffer = (np.zeros((samples_size, MAX_OUTPUT_CHANNELS))).astype(np.float32)
    buffer += random_buffer
    if len(audio_buffer.shape) > 1:
        audio_buffer_single_channel = audio_buffer[:, 0]
    else:
        audio_buffer_single_channel = audio_buffer
    buffer[:, speaker_id] += audio_buffer_single_channel 

    # Play audio
    stream.write(buffer.tobytes())

# Stop and close the stream
stream.stop_stream()
stream.close()

# Close PyAudio
p.terminate()