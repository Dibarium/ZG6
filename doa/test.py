import ctypes
import numpy as np
import matplotlib.pyplot as plt
from numpy.ctypeslib import ndpointer
from scipy.io import wavfile
from scipy.signal import hilbert
import tkinter

def create_signal(tau, sigma, N, type = np.int16):
    """Create a stereo random signal using the white Gaussian noise model and return it using the int16 format"""
    X = np.zeros((2, N))
    # implement the signal model

    x1=np.random.normal(0,sigma,N)
    x2=np.roll(x1,tau)
    X=np.vstack((x1,x2))
    
    X = X.astype(type) 
    return X

def load_audio2pcm(filename):
    """This function load a stereo audio file and convert the signal to a 1D signal with uint16 data type """
    Fs, X = wavfile.read("/home/Mathieu/Documents/Zg6visage/signal/zg6-signal-etudiants-main/"+filename)
    x = np.ravel(X, order='C')  # convert 2D signal to 1D signal
    x = x.astype(np.uint16)  # convert to uint16 data
    return Fs, x

def get_phase_shift(signal):
    """This function returns the phase shift between 2 signals. In other words, it returns the shift between 2 array"""
    h1= hilbert(signal[0])
    h2= hilbert(signal[1])

    P1=np.unwrap(np.angle(h1))
    P2=np.unwrap(np.angle(h2))

    dephasage = np.mean(P2-P1)
    return dephasage

def get_angle(tau):
    """returns the angle from the incoming sound"""
    return np.arcsin((343*tau)/10)


tau = 3   # samples
sigma = 2000
Fs = 16000  # Hz
N = 1600    # nombre d'échantillons
filename = "wav/noise.wav"

# Create the "wav" folder if it doesn't exist
#os.makedirs(os.path.dirname(filename), exist_ok=True)

X = create_signal(tau, sigma, N)
wavfile.write("/home/Mathieu/Documents/Zg6visage/signal/zg6-signal-etudiants-main/"+filename, Fs, np.transpose(X))
print(get_phase_shift(X))
print(get_angle(tau/Fs))

# import C function
dsp = ctypes.cdll.LoadLibrary("/home/Mathieu/Documents/Zg6visage/signal/zg6-signal-etudiants-main/dsp.so")

## load functions
print_pcm = dsp.print_pcm
print_pcm.restype = None
print_pcm.argtypes = [ndpointer(ctypes.c_uint16, flags="C_CONTIGUOUS"), ctypes.c_size_t]

compute_energy = dsp.compute_energy
compute_energy.restype = ctypes.c_float
compute_energy.argtypes = [ndpointer(ctypes.c_uint16, flags="C_CONTIGUOUS"), ctypes.c_size_t]

compute_xcorr = dsp.compute_xcorr
compute_xcorr.restype = ctypes.c_float
compute_xcorr.argtypes = [ndpointer(ctypes.c_uint16, flags="C_CONTIGUOUS"), ctypes.c_size_t, ndpointer(ctypes.c_float, flags="C_CONTIGUOUS"), ctypes.c_size_t]

#Load file
Fs, x = load_audio2pcm(filename)
print_pcm(x, 10)

#Energy pratique
energy_est = compute_energy(x, len(x))
print("estimated value: energy={}".format(energy_est))

 
from tkinter import * 

fenetre = Tk()
bg = PhotoImage(file = "/home/Mathieu/Documents/Zg6visage/signal/zg6-signal-etudiants-main/background.png") 
label1 = Label( fenetre, image = bg) 
label1.place(x = 0, y = 0)

label2 = Label( fenetre, text = "Energy : "+str(format(energy_est))) 
label2.place(x = 0, y = 0)

label3 = Label( fenetre, text = "Angle : "+ str(get_angle(tau/Fs))) 
label3.place(x = 0, y = 30)


fenetre.mainloop()