/**
******************************************************************************
* @file    cca02m2_audio.c
* @author  SRA
* @version v1.1.1
* @date    18-Dec-2020
* @brief   This file provides the Audio driver for the cca02m2
*          board.
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
*   Modified march 2022
*   Eric Boucharé
*/

/* Includes ------------------------------------------------------------------*/
#include "cca02m2_audio.h"
#include "pdm2pcm_glo.h"			/* Include PDM to PCM lib header file */
#include "DSP/Include/arm_math.h"

#include "startup/rcc.h"
#include "lib/io.h"
#include "lib/dma.h"

#define ENABLE_HIGH_PERFORMANCE_MODE		0U

typedef struct {
	int32_t		Z; 
	int32_t		oldOut; 
	int32_t		oldIn; 
} HPFilterState_t;  
  
typedef struct {
	uint32_t		SampleRate;			/* Audio IN Sample rate           */
	uint32_t		BitsPerSample;		/* Audio IN Sample resolution     */
	uint32_t		ChannelsNbr;		/* Audio IN number of channel     */
	uint8_t *		pBuff;				/* Audio IN record buffer         */
	uint32_t		Size;				/* Audio IN record buffer size    */
	uint32_t		Volume;				/* Audio IN volume                */
	uint32_t		State;				/* Audio IN State                 */
	HPFilterState_t	HP_Filters[4];		/* HP filter state for each channel */
	uint32_t		DecimationFactor;
	DMA_Stream_t *	stream;				/* DMA Stream */
	OnBuf_t			cb;					/* callback for signaling buffer filled */
} AUDIO_IN_Ctx_t;


/* Private Variables */

/* Recording context */
AUDIO_IN_Ctx_t AudioInCtx;
  
/* Configuration values to generate the correct I2S bitrates */
static const I2SClkCfg_t i2scfg[] = {
	{.M=4, .N=96,  .R=5},		/* 1280 kbits/s, I2SDIV=30 */
	{.M=4, .N=96,  .R=5},		/* 2560 kbits/s, I2SDIV=15 */
	{.M=5, .N=192, .R=3},		/* 2048 kbits/s, I2SDIV=50 */
	{.M=5, .N=192, .R=3},		/* 4096 kbits/s, I2SDIV=25 */
	{.M=4, .N=192, .R=5},		/* 3072 kbits/s, I2SDIV=25 */
	{.M=4, .N=192, .R=5},		/* 6144 kbits/s, I2SDIV=10 */
};

#define DECIMATOR_NUM_TAPS (16U)
#define DECIMATOR_BLOCK_SIZE (16U*N_MS_PER_INTERRUPT)
#define DECIMATOR_FACTOR 2U
#define DECIMATOR_STATE_LENGTH (DECIMATOR_BLOCK_SIZE + (DECIMATOR_NUM_TAPS) -1U)
static arm_fir_decimate_instance_q15 ARM_Decimator_State[4];

/* PDM filters params */
static PDM_Filter_Handler_t  PDM_FilterHandler[4];
static PDM_Filter_Config_t   PDM_FilterConfig[4];

static uint8_t i2s_pdm_buf[2*MAX_PDMBUFSZ];

/* Private Function Prototypes */
static int32_t AUDIO_IN_PDMToPCM_Init(uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut);
static void I2S_DMA_Rx_cb(uint32_t stream, uint32_t bufid);


/*********************** Public functions **********************/
  
/** AUDIO_IN_Init
 *   Initialize wave recording.
 * 	   AudioInit: Init structure
 *	 returns BSP status
 */
int32_t AUDIO_IN_Init(AUDIO_IN_Init_t* AudioInit)
{
	int i2scfg_idx;
	uint32_t DIV;
	DMAEndPoint_t ep_i2s2_s3ch0 = {
		.type   = EP_SPI_RX,
		.addr0	= /* A COMPLETER */,
		.addr1	= NULL,
		.channel= /* A COMPLETER */,
		.cfg	= EP_FMT_HALF
	};
	DMAEndPoint_t ep_buf_rx = {
		.type   = EP_MEM,
		.addr0  = i2s_pdm_buf,
		.addr1  = (void*)((uint8_t*)i2s_pdm_buf+MAX_PDMBUFSZ),
		.channel= -1,
		.cfg    = EP_FMT_HALF|EP_AUTOINC|EP_BUF_DBL
	};
	
	/* Store the audio record context */
	AudioInCtx.ChannelsNbr     = AudioInit->ChannelsNbr;  
	AudioInCtx.SampleRate      = AudioInit->SampleRate; 
	AudioInCtx.BitsPerSample   = AudioInit->BitsPerSample;
	AudioInCtx.Volume          = AudioInit->Volume;
	AudioInCtx.State           = AUDIO_IN_STATE_RESET;
	AudioInCtx.pBuff           = AudioInit->pdm_buf;
	AudioInCtx.cb              = AudioInit->cb;
	
	uint32_t bitrate;     
	
	switch (AudioInit->SampleRate) {
	case AUDIO_FREQ_8K:
		bitrate = 1280;		// DecimationFactor=160
		i2scfg_idx = 0;
		break;
	case AUDIO_FREQ_16K:
		bitrate = 1280;		// DecimationFactor=80
		i2scfg_idx = 0;
		break;
	case AUDIO_FREQ_32K:
		bitrate = 2048;		// DecimationFactor=64
		i2scfg_idx = 2;
		break;
	case AUDIO_FREQ_48K:
		bitrate = 3072;		// DecimationFactor=64
		i2scfg_idx = 4;
		break;
	default:        
		bitrate = 0;
		break;
	}
	
	if (bitrate == 0U) return BSP_ERROR_WRONG_PARAM;
	
	AudioInCtx.DecimationFactor = (bitrate * 1000U)/AudioInit->SampleRate;

	/* Double buffer for 1 microphone */
	AudioInCtx.Size = bitrate/8U*AudioInCtx.ChannelsNbr*N_MS_PER_INTERRUPT;
	
	/************************************************************
	 * Hardware peripheral setup
	 ************************************************************/
	if (AudioInCtx.ChannelsNbr>1U) {
		bitrate *= 2U;
		/* i2scfg table is organized so that when bitrate is doubled
		   cfg is next in the table (for the supported bitrates) */
		i2scfg_idx += 1;
		
		/* Configure Timer 3 as a mere frequency divider
		 *               +-------+
		 *  input ch1 -->|  F/2  |--> output chan2
		 *     PB4       +-------+       PB5
		 */
		// Configure GPIOs AF2: PB4 --> TIM3 chan 1, PB5 --> TIM3 chan 2 

		/**************************************
		 **************************************
		 *
		 *  A COMPLETER avec le code développé dans 
		 *
		 *  le projet PDM_PREP
		 *
		 **************************************
		 **************************************/

	}      
	
	/**** PLLI2S clock setup */
	rcc_i2s_clk_cfg(&i2scfg[i2scfg_idx]);
	
	/**** I2S2 = SPI2 setup */
	/* I2S2 reset and Clock enable */
	_RCC->APB1RSTR |= 1<<14;
	_RCC->APB1RSTR &= ~(1<<14);
	_RCC->APB1ENR |= 1<<14;
	
	/*  Prescaler */
	DIV = sysclks.i2s_freq / (bitrate*1000);
	_SPI2->I2SPR = (DIV & 1)<<8 | ((DIV>>1) & 0xFF);
	
	/* pin cfg */
	io_configure(SPI2_GPIO_PORT,SPI2_GPIO_PINS,SPI2_GPIO_CONFIG,NULL);
	
	/* CR2: Rx DMA on, stream XXX, channel ZZZ */
	AudioInCtx.stream = dma_stream_init(_DMA1, /* A COMPLETER */, &ep_i2s2_s3ch0, &ep_buf_rx, STRM_PRIO_HIGH, I2S_DMA_Rx_cb);

	_SPI2->CR2 = 1;
	
	/* I2S Config: mode=Master Rx, STD=MSB, CKPOL=1, 16bits data len */
	_SPI2->I2SCFGR = (1<<11) | (3<<8) | (1<<4) | (1<<3);
	
	// not enabled !!
	_TIM3->EGR = 1;
	_TIM3->CR1 = 1;
	_SPI2->I2SCFGR |= 1<<10;
	dma_start(AudioInCtx.stream, AudioInCtx.Size/2);
	
	if (AUDIO_IN_PDMToPCM_Init(AudioInCtx.SampleRate, AudioInCtx.ChannelsNbr, AudioInCtx.ChannelsNbr)!= BSP_ERROR_NONE) {
		return  BSP_ERROR_NO_INIT;
	}
	
	/* Update BSP AUDIO IN state */     
	AudioInCtx.State = AUDIO_IN_STATE_STOP; 
	
	/* Return BSP status */ 
	return BSP_ERROR_NONE;
}

/** AUDIO_IN_DeInit
 *   Deinit the audio IN peripheral.
 *	 returns BSP status
 */
int32_t AUDIO_IN_DeInit(void)
{  
	AUDIO_IN_Stop();
	/* Update BSP AUDIO IN state */     
	AudioInCtx.State = AUDIO_IN_STATE_RESET;   

	/* Return BSP status */
	return BSP_ERROR_NONE;
}


/** AUDIO_IN_Record
 *   Start audio recording.
 *   returns BSP status
 */
int32_t AUDIO_IN_Record(void)
{  
	if(AudioInCtx.ChannelsNbr > 2U) {
#if 0
		if(HAL_SPI_Receive_DMA(&hAudioInSPI, (uint8_t *)SPI_InternalBuffer, (uint16_t)AudioInCtx.Size) != HAL_OK) {
			return BSP_ERROR_PERIPH_FAILURE;
		}
#endif
		return BSP_ERROR_PERIPH_FAILURE;
	}
	
	if(AudioInCtx.ChannelsNbr != 1U) {
		_TIM3->EGR = 1;
		_TIM3->CR1 |= 1;
	}
	_SPI2->I2SCFGR |= 1<<10;
	dma_start(AudioInCtx.stream, AudioInCtx.Size/2);

	/* Update BSP AUDIO IN state */     
	AudioInCtx.State = AUDIO_IN_STATE_RECORDING;           
	
	/* Return BSP status */
	return BSP_ERROR_NONE;
}

/** AUDIO_IN_Stop
 *   Stop audio recording.
 *   returns BSP status
 */
int32_t AUDIO_IN_Stop(void)
{
	if(AudioInCtx.ChannelsNbr > 2U) {
#if 0
		if(HAL_SPI_DMAStop(&hAudioInSPI)!= HAL_OK) {
			return BSP_ERROR_PERIPH_FAILURE;
		}
#endif
		return BSP_ERROR_PERIPH_FAILURE;
	}
			
	_TIM3->CR1 &= ~1;
	_SPI2->I2SCFGR &= ~(1<<10);
	dma_stop(AudioInCtx.stream);
	
	/* Update BSP AUDIO IN state */     
	AudioInCtx.State = AUDIO_IN_STATE_STOP;
	
	/* Return BSP status */
	return BSP_ERROR_NONE;  
}


/** AUDIO_IN_Pause
 *   Pause the audio file stream.
 *  returns BSP status
 */
int32_t AUDIO_IN_Pause(void)
{
	_SPI2->I2SCFGR &= ~(1<<10);

	/* Update BSP AUDIO IN state */     
	AudioInCtx.State = AUDIO_IN_STATE_PAUSE;    
	
	/* Return BSP status */
	return BSP_ERROR_NONE;
}

/** AUDIO_IN_Resume
* @brief  Resume the audio file stream.
* @retval BSP status
*/
int32_t AUDIO_IN_Resume(void)
{
	_SPI2->I2SCFGR |= 1<<10;
	
	/* Update BSP AUDIO IN state */     
	AudioInCtx.State = AUDIO_IN_STATE_RECORDING;
	
	/* Return BSP status */
	return BSP_ERROR_NONE;
}

/** AUDIO_IN_SetSampleRate
 *   Set Audio In frequency
 *     SampleRate  Input frequency to be set
 *   returns BSP status
 */
int32_t AUDIO_IN_SetSampleRate(uint32_t SampleRate)
{
	AUDIO_IN_Init_t audio_init;
	
	if(AudioInCtx.State == AUDIO_IN_STATE_STOP) {
		audio_init.ChannelsNbr   = AudioInCtx.ChannelsNbr;  
		audio_init.SampleRate    = SampleRate;   
		audio_init.BitsPerSample = AudioInCtx.BitsPerSample;
		audio_init.Volume        = AudioInCtx.Volume; 
		audio_init.pdm_buf       = AudioInCtx.pBuff;
		audio_init.cb            = AudioInCtx.cb;
		if(AUDIO_IN_Init(&audio_init) != BSP_ERROR_NONE) {
			return BSP_ERROR_NO_INIT;
		}   
	} else {
		return BSP_ERROR_BUSY;
	}  
	/* Return BSP status */
	return BSP_ERROR_NONE;  
}

/** AUDIO_IN_GetSampleRate
 *   Get Audio In frequency
 *     SampleRate  Audio Input frequency to be returned
 *   returns BSP status
 */
int32_t AUDIO_IN_GetSampleRate(uint32_t *SampleRate)
{
	/* Return audio in frequency */
	*SampleRate = AudioInCtx.SampleRate;
	/* Return BSP status */  
	return BSP_ERROR_NONE;
}

/** AUDIO_IN_SetBitsPerSample
 *   Set Audio In Resolution
 *     BitsPerSample  Input resolution to be set
 *   returns BSP status
 */
int32_t AUDIO_IN_SetBitsPerSample(uint32_t BitsPerSample)
{
	AUDIO_IN_Init_t audio_init;
	
	if(AudioInCtx.State == AUDIO_IN_STATE_STOP) {
		audio_init.ChannelsNbr   = AudioInCtx.ChannelsNbr;  
		audio_init.SampleRate    = AudioInCtx.SampleRate;   
		audio_init.BitsPerSample = BitsPerSample;
		audio_init.Volume        = AudioInCtx.Volume; 
		audio_init.pdm_buf       = AudioInCtx.pBuff;
		audio_init.cb            = AudioInCtx.cb;
		if(AUDIO_IN_Init(&audio_init) != BSP_ERROR_NONE) {
			return BSP_ERROR_NO_INIT;
		}
	} else {
		return BSP_ERROR_BUSY;
	}  
	/* Return BSP status */  
	return BSP_ERROR_NONE;
}

/** AUDIO_IN_GetBitsPerSample
 *   Get Audio In Resolution
 *     BitsPerSample  Input resolution to be returned
 *   returns BSP status
 */
int32_t AUDIO_IN_GetBitsPerSample(uint32_t *BitsPerSample)
{
	/* Return audio in resolution */
	*BitsPerSample = AudioInCtx.BitsPerSample;
	/* Return BSP status */  
	return BSP_ERROR_NONE;
}

/** AUDIO_IN_SetChannelsNbr
 *   Set Audio In Channel number
 *     ChannelNbr  Channel number to be used
 *   returns BSP status
 */
int32_t AUDIO_IN_SetChannelsNbr(uint32_t ChannelNbr)
{
	return BSP_ERROR_WRONG_PARAM;
}

/** AUDIO_IN_GetChannelsNbr
 *   Get Audio In Channel number
 *     ChannelNbr  Channel number to be used
 *   returns BSP status
 */
int32_t AUDIO_IN_GetChannelsNbr(uint32_t *ChannelNbr)
{
	/* Channel number to be returned */
	*ChannelNbr = AudioInCtx.ChannelsNbr;
	/* Return BSP status */  
	return BSP_ERROR_NONE;
}

/** AUDIO_IN_SetVolume
 *   Set the current audio in volume level.
 *	   Volume    Volume level to be returnd
 *	 return BSP status
 */
int32_t AUDIO_IN_SetVolume(uint32_t Volume)
{
	uint32_t index;      
	static int16_t VolumeGain[] = {
		-12,-12,-6,-3,0,2,3,5,6,7,8,9,9,10,11,11,12,12,13,13,14,14,15,15,15,
		16,16,17,17,17,17,18,18,18,19,19,19,19,19,20,20,20,20,21,21,21,21,21,
		22,22,22,22,22,22,23,23,23,23,23,23,23,24,24,24,24,24,24,24,25,25,25,
		25,25,25,25,25,25,26,26,26,26,26,26,26,26,26,27,27,27,27,27,27,27,27,
		27,27,28,28,28,28,28,28,28,28,28,28,28,28,29,29,29,29,29,29,29,29,29,
		29,29,29,29,30,30,30,30,30,30,30,31  
	};
	for (index = 0; index < AudioInCtx.ChannelsNbr; index++) {
		if (PDM_FilterConfig[index].mic_gain != VolumeGain[Volume]) {
			PDM_FilterConfig[index].mic_gain = VolumeGain[Volume];
			(void)PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
		}
	}
	/* Update AudioIn Context */
	AudioInCtx.Volume = Volume;
	/* Return BSP status */
	return BSP_ERROR_NONE;  
}

/** AUDIO_IN_GetVolume
 *   Get the current audio in volume level.
 *     Volume    Volume level to be returnd
 *   returns BSP status
 */
int32_t AUDIO_IN_GetVolume(uint32_t *Volume)
{
	/* Input Volume to be returned */
	*Volume = AudioInCtx.Volume;
	/* Return BSP status */
	return BSP_ERROR_NONE;  
}

/** AUDIO_IN_GetState
 *   Get Audio In device
 *     State     Audio Out state
 *   returns BSP status
 */
int32_t AUDIO_IN_GetState(uint32_t *State)
{
	/* Input State to be returned */
	*State = AudioInCtx.State;
	/* Return BSP status */
	return BSP_ERROR_NONE;
}

/** AUDIO_IN_PDMToPCM
 *   Converts audio format from PDM to PCM.
 *     PDMBuf    Pointer to PDM buffer data
 *     PCMBuf    Pointer to PCM buffer data
 *   returns BSP status
 */
int32_t AUDIO_IN_PDMToPCM(uint8_t *PDMBuf, uint16_t *PCMBuf)
{    
	uint32_t index;
	
	for(index = 0; index < AudioInCtx.ChannelsNbr; index++) {
		if (AudioInCtx.SampleRate == 8000U) {
			uint16_t Decimate_Out[8U*N_MS_PER_INTERRUPT];
			uint32_t ii;
			uint16_t PDM_Filter_Out[16U*N_MS_PER_INTERRUPT];
			
			(void)PDM_Filter(&PDMBuf[index], PDM_Filter_Out, &PDM_FilterHandler[index]);
			(void)arm_fir_decimate_q15 (&ARM_Decimator_State[index], (q15_t *)&(PDM_Filter_Out), (q15_t*)&(Decimate_Out), DECIMATOR_BLOCK_SIZE);
			for (ii=0; ii<(8U*N_MS_PER_INTERRUPT); ii++) {
				PCMBuf[(ii * AudioInCtx.ChannelsNbr) + index] = Decimate_Out[ii];
			}
		} else {
			switch(AudioInCtx.BitsPerSample) {
			case AUDIO_RES_16b:
				(void)PDM_Filter(&PDMBuf[index], (uint16_t*)&(PCMBuf[index]), &PDM_FilterHandler[index]);
				break;
			case AUDIO_RES_24b:
				(void)PDM_Filter(&PDMBuf[index], &((uint8_t*)(PCMBuf))[3U*index], &PDM_FilterHandler[index]);          
				break;
			case AUDIO_RES_32b:
				(void)PDM_Filter(&PDMBuf[index], (uint32_t*)&(PCMBuf[index]), &PDM_FilterHandler[index]);          
				break;
			default:
				break;
			}
		}
	}    
	return BSP_ERROR_NONE;
}


/*********************** Private functions **********************/

/** AUDIO_IN_PDMToPCM_Init
 *   Initialize the PDM library.
 *     AudioFreq  Audio sampling frequency
 *     ChnlNbrIn  Number of input audio channels in the PDM buffer
 *     ChnlNbrOut Number of desired output audio channels in the  resulting PCM buffer
 *   returns BSP status
 */
static int32_t AUDIO_IN_PDMToPCM_Init(uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut)
{  
	uint32_t index; 
#if (ENABLE_HIGH_PERFORMANCE_MODE == 0U)    
	static int16_t aState_ARM[4][DECIMATOR_STATE_LENGTH];
	static int16_t aCoeffs[] = { -1406, 1634, -1943, 2386, -3080, 4325, -7223, 21690, 21690, -7223, 4325, -3080, 2386, -1943, 1634, -1406, };
#endif
	
	/* Enable CRC peripheral to unlock the PDM library */
	_RCC->AHB1ENR |= 1<<12;
	
	for(index = 0; index < ChnlNbrIn; index++) {
		volatile uint32_t error = 0;
		/* Init PDM filters */
		PDM_FilterHandler[index].bit_order  = PDM_FILTER_BIT_ORDER_LSB;
		if (ChnlNbrIn == 1U) {
			PDM_FilterHandler[index].endianness = PDM_FILTER_ENDIANNESS_BE; /* For WB this should be LE, TODO after bugfix in PDMlib */
		} else {
			PDM_FilterHandler[index].endianness = PDM_FILTER_ENDIANNESS_LE;
		}
		PDM_FilterHandler[index].high_pass_tap = 2122358088;
		PDM_FilterHandler[index].out_ptr_channels = (uint16_t)ChnlNbrOut;
		PDM_FilterHandler[index].in_ptr_channels  = (uint16_t)ChnlNbrIn;
	
		/* PDM lib config phase */
		PDM_FilterConfig[index].output_samples_number = (uint16_t) ((AudioFreq/1000U) * N_MS_PER_INTERRUPT);
		PDM_FilterConfig[index].mic_gain = 24;
	
		switch (AudioInCtx.DecimationFactor) {
		case 16:
			PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_16;
			break;
		case 24:
			PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_24;
			break;
		case 32:
			PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_32;
			break;
		case 48:
			PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_48;
			break;
		case 64:
			PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_64;
			break;
		case 80:
			PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_80;
			break;
		case 128:
			PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_128;
			break;
		case 160:
			PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_80;
			PDM_FilterConfig[index].output_samples_number *= 2U;        
			PDM_FilterHandler[index].out_ptr_channels = 1;
			(void)arm_fir_decimate_init_q15  (&ARM_Decimator_State[index], DECIMATOR_NUM_TAPS, DECIMATOR_FACTOR,
		                      aCoeffs, aState_ARM[index], DECIMATOR_BLOCK_SIZE);
			break;
		default:
			break;
		}  
	
#if (ENABLE_HIGH_PERFORMANCE_MODE == 1U)
		switch(AudioInCtx.BitsPerSample)
		{
		case AUDIO_RES_16b:
			PDM_FilterConfig[index].bit_depth = PDM_FILTER_BITDEPTH_16;
			break;
		case AUDIO_RES_24b:
			PDM_FilterConfig[index].bit_depth = PDM_FILTER_BITDEPTH_24;
			break;
		case AUDIO_RES_32b:
			PDM_FilterConfig[index].bit_depth = PDM_FILTER_BITDEPTH_24IN32;
			break;
		default:
			break;        
		}
#endif
	
		error = PDM_Filter_Init((PDM_Filter_Handler_t *)(&PDM_FilterHandler[index]));
		if (error!=0U) {
			return  BSP_ERROR_NO_INIT;
		}      
		error = PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
		if (error!=0U) {
			return  BSP_ERROR_NO_INIT;
		}
	}
	return BSP_ERROR_NONE;
}

/** I2S_DMA_Rx_cb
 * @brief Rx Transfer completed callbacks. It performs demuxing of the bit-interleaved PDM streams into 
 * byte-interleaved data suitable for PDM to PCM conversion. 1 ms of data for each microphone is 
 * written into the buffer that the user indicates when calling the AUDIO_IN_Start(...) function.
 * @retval None
 */
#define CHANNEL_DEMUX_MASK					0x55U

static uint8_t Channel_Demux[128] = {
	0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
	0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
	0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
	0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
	0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
	0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
	0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
	0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
	0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
	0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
	0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
	0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
	0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
	0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
	0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
	0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f
};

void I2S_DMA_Rx_cb(uint32_t stream, uint32_t bufid)
{
	uint32_t index;
	switch(AudioInCtx.ChannelsNbr) {
	case 1: {
		uint8_t * DataTempI2S = bufid ? i2s_pdm_buf : i2s_pdm_buf+MAX_PDMBUFSZ;
		for(index = 0; index < (AudioInCtx.Size/4U); index++) {
			AudioInCtx.pBuff[index] = (DataTempI2S[index]);
		}
		break;
	}
	case 2: {
		uint8_t * DataTempI2S = bufid ? i2s_pdm_buf : i2s_pdm_buf+MAX_PDMBUFSZ;
		uint8_t a,b;
		for(index=0; index<(AudioInCtx.Size/2U); index++) {
			a = DataTempI2S[(index*2U)];
			b = DataTempI2S[(index*2U)+1U];
			AudioInCtx.pBuff[(index*2U)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] | (Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4);
			AudioInCtx.pBuff[(index*2U)+1U] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] | (Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4);
		}
		break;
	}
	case 4: {
#if 0
		uint8_t * DataTempI2S = bufid ? i2s_pdm_buf : i2s_pdm_buf+MAX_PDMBUFSZ;
		uint8_t * DataTempSPI = bufid ? SPI_InternalBuffer : SPI_InternalBuffer+MAX_PDM_PUBSZ;
		uint8_t a,b;
		for(index=0; index<(AudioInCtx.Size/2U); index++) {
			a = DataTempI2S[(index*2U)];
			b = DataTempI2S[(index*2U)+1U];
			AudioInCtx.pBuff[(index*4U)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] | (Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4);
			AudioInCtx.pBuff[(index*4U)+1U] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] | (Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4);
			
			a = DataTempSPI[(index*2U)];
			b = DataTempSPI[(index*2U)+1U];
			AudioInCtx.pBuff[(index*4U)+2U] = Channel_Demux[a & CHANNEL_DEMUX_MASK] | (Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4);
			AudioInCtx.pBuff[(index*4U)+3U] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] | (Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4);
		}
#endif
		break;
	}
	default:
		break;
	}
	if (AudioInCtx.cb) AudioInCtx.cb();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
