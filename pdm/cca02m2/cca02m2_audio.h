/**
******************************************************************************
* @file    cca02m2_audio.h
* @author  SRA
* @version v1.1.1
* @date    18-Dec-2020
* @brief   This file contains the common defines and functions prototypes for
*          the cca02m2_audio.c driver.
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2020 STMicroelectronics. 
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the 
* License. You may obtain a copy of the License at:
*                        opensource.org/licenses/BSD-3-Clause
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CCA02M2_AUDIO_H
#define CCA02M2_AUDIO_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "include/board.h"

/* Common Error codes */
#define BSP_ERROR_NONE                    0
#define BSP_ERROR_NO_INIT                -1
#define BSP_ERROR_WRONG_PARAM            -2
#define BSP_ERROR_BUSY                   -3
#define BSP_ERROR_PERIPH_FAILURE         -4
#define BSP_ERROR_COMPONENT_FAILURE      -5
#define BSP_ERROR_UNKNOWN_FAILURE        -6
#define BSP_ERROR_UNKNOWN_COMPONENT      -7
#define BSP_ERROR_BUS_FAILURE            -8
#define BSP_ERROR_CLOCK_FAILURE          -9
#define BSP_ERROR_MSP_FAILURE            -10

/* CCA02M2_AUDIO_ Exported Types */
typedef void (*OnBuf_t)(void);

typedef struct {                                   
	uint32_t	Device;                                           
	uint32_t	SampleRate;                                         
	uint32_t	BitsPerSample;                                          
	uint32_t	ChannelsNbr;                                         
	uint32_t	Volume;
	uint8_t *	pdm_buf;
	OnBuf_t		cb;
} AUDIO_IN_Init_t;

/** CCA02M2_AUDIO_ Exported Constants */

/* AUDIO FREQUENCY */
#define AUDIO_FREQ_48K						48000U
#define AUDIO_FREQ_32K						32000U
#define AUDIO_FREQ_16K						16000U
#define AUDIO_FREQ_8K						8000U

/* AUDIO RESOLUTION */
#define AUDIO_RES_16b						16U
#define AUDIO_RES_24b						24U
#define AUDIO_RES_32b						32U

/*------------------------------------------------------------------------------
                        AUDIO IN defines parameters
------------------------------------------------------------------------------*/ 

/* Audio In devices defaults */
#define AUDIO_IN_VOLUME_DEFAULT				64U

/*Number of millisecond of audio at each DMA interrupt*/
#define N_MS_PER_INTERRUPT					(10U)

/* Maximum values */
#define MX_SMAPLING_FREQ					AUDIO_FREQ_48K
#define MAX_BITRATE							3072	/* kHz or bkit/s */
#define MAX_CHANNEL_NB						2		/* number of channels per interface */
#define MAX_DECIMATION_FACTOR				160

#define MAX_PDMBUFSZ						(MAX_BITRATE/8*MAX_CHANNEL_NB*N_MS_PER_INTERRUPT)
#define MAX_PCMBUFSZ						(MAX_SAMPLING_FREQ/1000*MAX_CHANNEL_NB*N_MS_PER_INTERRUPT)

/* Audio In states */
#define AUDIO_IN_STATE_RESET               0U
#define AUDIO_IN_STATE_RECORDING           1U
#define AUDIO_IN_STATE_STOP                2U
#define AUDIO_IN_STATE_PAUSE               3U

/** CCA02M2_AUDIO_ Exported Macros */
#ifndef POS_VAL
#define POS_VAL(VAL)                  (POSITION_VAL(VAL) - 4U)
#endif
#define VOLUME_IN_CONVERT(Volume)     (((Volume) >= 100)? 239:((uint8_t)(((Volume) * 239) / 100)))
#define DMA_MAX(_X_)	(((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)
#define HTONS(A)		((((A) & (uint16_t)0xff00) >> 8) | (((A) & (uint16_t)0x00ff) << 8))


/****************************************************************/

/* CCA02M2_AUDIO_IN Exported Functions */
int32_t AUDIO_IN_Init(AUDIO_IN_Init_t* AudioInit);    
int32_t AUDIO_IN_DeInit(void);
int32_t AUDIO_IN_Record(void);
int32_t AUDIO_IN_Stop(void);

int32_t AUDIO_IN_SetSampleRate(uint32_t SampleRate);
int32_t AUDIO_IN_GetSampleRate(uint32_t *SampleRate);                 
int32_t AUDIO_IN_SetBitsPerSample(uint32_t BitsPerSample);
int32_t AUDIO_IN_GetBitsPerSample(uint32_t *BitsPerSample);                
int32_t AUDIO_IN_SetChannelsNbr(uint32_t ChannelNbr);
int32_t AUDIO_IN_GetChannelsNbr(uint32_t *ChannelNbr);
int32_t AUDIO_IN_SetVolume(uint32_t Volume);
int32_t AUDIO_IN_GetVolume(uint32_t *Volume);
int32_t AUDIO_IN_GetState(uint32_t *State);

/* Specific PDM recodr APIs */
int32_t AUDIO_IN_PDMToPCM(uint8_t *PDMBuf, uint16_t *PCMBuf);

#ifdef __cplusplus
}
#endif

#endif /* CCA02M2_AUDIO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
