/******************************************************************
 * file    main_pdm.c 
 * author  Eric Bouchare
 * version v1.0.0
 * date    2022, march, 26th
 * brief   Main program body
 ****************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "cca02m2/cca02m2_audio.h"
#include "lib/io.h"
#include "lib/uart.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define CHANNEL_NB              2
#define SAMPLING_FREQ           16000
#define SAMPLING_RES            16
#define VOLUME                  64U

#define LED_RED_NUM     (2)
#define LED_GREEN_NUM   (1)
#define LED_BLUE_NUM    (0)
#define LED_RED         (1 << LED_RED_NUM)
#define LED_GREEN       (1 << LED_GREEN_NUM)
#define LED_BLUE        (1 << LED_BLUE_NUM)

#define LED_RED_MASK    (1 << 4)
#define LED_GREEN_MASK  (1 << 7)

#define SPEED_OF_SOUND 343.0 // m/s
#define DISTANCE_BETWEEN_MICROPHONES 0.3 // meters

volatile int flag=0;
volatile float distance = 0.165;
uint8_t pdm_buf[MAX_PDMBUFSZ];
uint16_t pcm_buf[CHANNEL_NB * SAMPLING_FREQ / 1000 * N_MS_PER_INTERRUPT];
//uint16_t N = CHANNEL_NB * SAMPLING_FREQ / 1000 * N_MS_PER_INTERRUPT;
float pi = 3.14159265358979323846;

char angle_to_send[20];

void pdm_buf_filled_cb() {
    flag = 1;
}
float compute_energy(uint16_t *pcm_buf, int16_t N){
    // this function computes the energy of the samples provided in the 
    // input array
    float energy = 0;
    //uart_printf(_USART2,"Le buf est %d lalalala",pcm_buf);
    // DO YOU STUFF HERE
    for(int i=0;i<N;i++) {
        //float sample = (float)pcm_buf[i] ;
        //energy += pow(sample,2);
        energy += ((int16_t)pcm_buf[i]*(int16_t)pcm_buf[i]);
        //uart_printf(_USART2,"Le buf est %d ",pcm_buf[i]);

    }
    energy = energy*(1.0/N);

    return energy;
}
void compute_xcorr(uint16_t *pcm_buf, int16_t N, float *R, int N_R) {
    // this function computes the cross correlation for the samples provided in the 
    // input array.
    // N echantillon de pcm
    // N_r echantillon de la corrÃ©lation
    for (int m = 0; m < N_R; m++) {
        for (int n = 0; n < N / 2; n++) {
            int16_t x1 = (int16_t)pcm_buf[2 * n];
            int16_t x2 = (int16_t)pcm_buf[2 * n + 1];
            if (n >= m) {
                x2 = (int16_t)pcm_buf[(2 * (n - m) + N_R) % N];
                R[m] += (float)x1 * (float)x2;
            } else {
                R[m] += 0;
            }
        }
        R[m] = R[m] / (N / 2);
    }
}


int find_max_index(float *R, int N_R) {
    int max_index = 0;
    for (int i = 1; i < N_R; i++) {
        if (R[i] > R[max_index]) {
            max_index = i;
        }
    }
    return max_index; // on retourne lindex correspondant au decalage 
}

void absisse_tau(int tau_max, float *tau_array){
	for (int i =0;i<(2*tau_max+1);i++){
		tau_array[i]=-tau_max+i;
	}
}

void led(uint32_t on) {
    if (on){
        io_clear(_GPIOA, PIN_0);
    }
}
uint32_t leds_init(void) {
    io_configure(_GPIOA, PIN_0, PIN_MODE_OUTPUT | PIN_OPT_OUTPUT_OPENDRAIN, NULL);
    led(0);
    return 0;
}
int calculate_tau(float *R, int N_R) {
    int max_index = find_max_index(R, N_R);
    int tau = max_index - (N_R / 2);
    return tau;
}
float calculate_theta(int tau, float D) {
    float delta_d = SPEED_OF_SOUND * tau / SAMPLING_FREQ;
    float theta = asin(delta_d / D);
    return theta;
}

int main(void)
{
	AUDIO_IN_Init_t MicParams = {
		.SampleRate = SAMPLING_FREQ,
		.BitsPerSample = SAMPLING_RES,
		.ChannelsNbr = CHANNEL_NB,
		.Volume = VOLUME,
		.pdm_buf = pdm_buf,
		.cb = pdm_buf_filled_cb
	};
	
	AUDIO_IN_Init(&MicParams);
	AUDIO_IN_Record();
	uart_init(_USART2, 115200, UART_8N1);
	//uart_printf(_USART2," est l'energie du signal reÃ§u");

	io_configure(_GPIOA, (1<<5), PIN_MODE_OUTPUT, NULL);
	
    //uint16_t pcm_buf[] = {-2085,1093,919,-135,166,-72,-579,-420,982,-436,-1514,420,-492,-143,689,634,-1530,-611,538,261,-825,1466}
    
    //float temps = distance/340.0;
    int tau_max = 10;
    int N_R = 2 * tau_max + 1;;
	
	float R_est[N_R];
	memset(R_est, 0.0,sizeof(R_est));
	float tau_array[N_R];
	absisse_tau(tau_max,tau_array);


	while (1) {
		if (flag) {
			AUDIO_IN_PDMToPCM(pdm_buf,pcm_buf);
			flag=0;
			/* Do something with PCM data */
			float seuil = 4300000;
			float energy = compute_energy(pcm_buf,sizeof(pcm_buf)/sizeof(pcm_buf[0]));
			//uart_printf(_USART2,"On a une energie de %f",energy);
			if (energy>seuil){
				_GPIOA->BSRR |= (1<<5);


                // Assumons que x (pcm_buf) et sa longueur N sont dÃ©jÃ  dÃ©finis et initialisÃ©s

                int N = sizeof(pcm_buf) / sizeof(pcm_buf[0]);

                // Allocation et initialisation de R_est
                


                

                // Appeler la fonction compute_xcorr
                
                compute_xcorr(pcm_buf, N, R_est, N_R);
  				int indice = find_max_index(R_est,N_R);
  				float tau = tau_array[indice];
  				//int tau = calculate_tau(R_est, N_R);

    // Calculate theta
    			//float theta = calculate_theta(tau, DISTANCE_BETWEEN_MICROPHONES);

    // Convert theta to degrees for better interpretation
    			//float theta_degrees = theta * (180.0 / pi);
  				float angle = asinf(340.0*(tau/SAMPLING_FREQ)/distance);
  				int angle_deg =(int)(angle*180.0/pi);          
                
                /* Convert the calculated angle into a specific 3-charcter string
                1st character: sign
                2nd & 3rd charcter: value of the angle
                (the value of the angle is never > 70) */
                char sign;
                
                if (angle_deg >= 0)
                {
                    char sign = '+';
                } else 
                {
                    char sign = '-';
                }
                if (abs(angle_deg) < 10)
                {
                    sprintf(angle_to_send, "%c0%i", sign, angle_deg);
                } else
                {
                    sprintf(angle_to_send, "%c%i", sign, angle_deg);
                }
    			uart_printf(_USART2, angle_to_send);

                // Liberer la memoire allouee
                //free(R_est);

			}
			else {
				
				_GPIOA->BSRR |= (1<<21);//eteind la led
			
			}

    	}
	}
	
	return 0;
}