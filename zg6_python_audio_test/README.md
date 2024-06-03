# ZG6 Python Audio Test Platform


# Auto Spatialization Testing Toolkit

This repository contains a set of Python scripts designed for automated spatialization testing using a multi-output sound card like the scarlett 18i20. These tools are ideal for audio engineers, researchers, and enthusiasts looking to test and calibrate spatial audio setups. The toolkit allows for dynamic positioning of sound sources and includes capabilities for generating random noise across multiple speakers.


## Getting Started

Clone the repository to your local machine:

```bash
git https://git.enib.fr/choqueuse/zg6_python_audio_test.git
cd zg6_python_audio_test
```

Then install the project dependencies 

```
pip install -r requirements.txt
``` 


## Features

- **print_devices.py**: Enumerate all output devices managed by the OS to easily select the target device for testing.

    ```bash
    python print_devices.py
    ````

    The script should output somethink like 
    
    ```
    Available output devices:
    Index: 1 - ...
    ...
    Index: X - Name: Scarlett 18i20 USB - Max Output Channels: 20
    ...
    ```

    where X corresponds to the index of the scarlett device.


- **test_vad_noise_only.py**: Generate some random noise with a specific probability of occurence.

    ```bash
    python test_vad_noise_only.py 1 --loop 50 --duration 0.2
    ```

    where the number 1 correspond to the index of the scarlett device.

- **test_doa_noise_only.py**: Generate random noise with dynamic positioning across multiple speakers to test sound localisation algorithms.

    ```bash
    python test_doa_noise_only.py 1 --loop 20 --scenario 1 --duration 1
    ```

    where the number 1 correspond to the index of the scarlett device.

- **test_doa_noise_only.py**: Create a WAV file that combines dynamic positioning of sound with random background noise, simulating real-world spatial audio scenarios.

    ```bash
    python test_doa_wav_with_noise.py 1 --loop 5 --scenario 1 --wavfile "./wav/music1.wav"
    ```

    where the number 1 correspond to the index of the scarlett device.


## Contributing

Contributions to improve Auto Spatialization Testing Toolkit are welcome. Feel free to fork the repository and submit pull requests.