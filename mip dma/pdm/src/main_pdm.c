/****************************************************************
 * file    main_pdm.c 
 * author  Eric Boucharé
 * version v1.0.0
 * date    2022, march, 26th
 * brief   Main program body
 ****************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "cca02m2/cca02m2_audio.h"
#include "lib/io.h"

#define CHANNEL_NB				2
#define SAMPLING_FREQ			16000
#define SAMPLING_RES			16
#define VOLUME					64U

volatile int flag=0;

uint8_t pdm_buf[MAX_PDMBUFSZ];
uint16_t pcm_buf[CHANNEL_NB*SAMPLING_FREQ/1000*N_MS_PER_INTERRUPT];

void pdm_buf_filled_cb()
{
	flag=1;
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
	
	while (1) {
		if (flag) {
			AUDIO_IN_PDMToPCM(pdm_buf,pcm_buf);
			flag=0;
			/* Do something with PCM data */
		}
	}
	
	return 0;
}
