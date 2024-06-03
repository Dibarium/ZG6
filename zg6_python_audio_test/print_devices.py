import pyaudio

def list_output_devices():
    p = pyaudio.PyAudio()
    output_devices = []
    print("Available output devices:")
    for i in range(p.get_device_count()):
        device_info = p.get_device_info_by_index(i)
        if device_info['maxOutputChannels'] > 0:
            print(f"Index: {i} - Name: {device_info['name']} - Max Output Channels: {device_info['maxOutputChannels']}")
            output_devices.append((i, device_info['name']))
    p.terminate()
    return output_devices


# List output devices
output_devices = list_output_devices()
