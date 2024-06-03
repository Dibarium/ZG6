# ZG6: Signal DSP Algorithm Prototyping


## Description

This repo contains a notebook and a C file for the prototyping of C algorithm for the ZG6 project.

* The jupyter notebook file :

    * generates a sound file and save it using the wav format in a `wav` folder,
    * imports some function of the C library,
    * compares the theoretical value of the `energy` and the one estimated using your C function.
    * compares the theoretical value of the `xcorr` (cross correlation) and the one estimated using your C function .
   
* The C file contains 3 functions :

    * `print_pcm`: prints the pcm values from an input array,
    * `compute_energy`: computes the signal energy,
    * `compute_xcorr`: computes the cross correlation.

## C Compilation

The C library can be compiled into a .so file using the command :

```
gcc -fPIC -shared dsp.c -o dsp.so
```

After compiling you c lib, I recommend to restart you python kernel in jupyter notebook to be sure that your notebook uses the last version of your library.  

If you encounter some problems using the compile C library, please indicate your error message **explicitly** in the following issue: https://git.enib.fr/choqueuse/zg6-signal-etudiants/-/issues/2

