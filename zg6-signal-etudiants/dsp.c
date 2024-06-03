#include <math.h>
#include <stdint.h>
#include <stdio.h>

void print_pcm(uint16_t *pcm_buf, int16_t N){
    // this function print the content of an audio signal
    for (int n=0; n<N; n=n+2)
    {
        int16_t x1 = (int16_t) pcm_buf[n];
        int16_t x2 = (int16_t) pcm_buf[n+1];
        printf("* sample%d: x=[%d,%d]\n", n/2, x1, x2);
    }
    fflush(stdout);
}

float compute_energy(uint16_t *pcm_buf, int16_t N){
    // this function computes the energy of the samples provided in the 
    // input array
    float energy = 0;

    // DO YOU STUFF HERE

    return energy;
}

void compute_xcorr(uint16_t *pcm_buf, int16_t N, float *R, int N_R){
    //this function the cross correlation for the samples provided in the 
    // input array.

    // DO YOU STUFF HERE
}
