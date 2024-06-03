#include "dma.h"
#include "lib/io.h"
/* DMA Use
 *
 * DMA_Stream_t* dma_stream_init(DMA_t* dma, uint32_t stream, DMAEndPoint*src, DMAEndPoint*dest, uint32_t mode, OnTC cb)
 *
 *    initialize a new stream.
 *
 * int dma_start(DMA_Stream_t* s, uint32_t size);
 *
 *    start the DMA of size bytes.
 *    If size == 0, then the peripheral is the flow controller else
 *    it is the DMA controller.
 *
 *
 *
 */

#if 0
/* endpoint examples */
const DMAEndPoint dma1s5_uart2_rx = {
	.type    = EP_UART_RX,
	.addr0   = (void*)&_USART2->DR,		// (USART2_BASE+4),
	.addr1  = NULL,
	.req    = 4,
	.cfg    = EP_FMT_BYTE
};

const DMAEndPoint dma1s6_uart2_tx = {
	.type    = EP_UART_TX,
	.addr0   = (void*)&_USART2->DR,		// (USART2_BASE+4),
	.addr1   = NULL,
	.channel = 4,
	.cfg     = EP_FMT_BYTE
};
#endif

#define DMAStream(dma,stream)	((DMA_Stream_t*)((char*)dma+0x10+0x18*stream))

/* Interrupt Service Routines DMA1 */
static OnTC dma1_cb[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
static volatile int dma1_stat[8];

void DMA1_Stream0_IRQHandler()
{
	if (_DMA1->ISR[0] & (1<<5)) {	// Transfer Complete
		DMA_Stream_t* stream=DMAStream(_DMA1,0);
		if (dma1_cb[0]) dma1_cb[0](0,!(stream->CR & (1<<19)));
		_DMA1->IFCR[0] = 1<<5;
		dma1_stat[0] = DMA_TC;
	} else if (_DMA1->ISR[0] & (1<<0)) {	// FIFO error
		_DMA1->IFCR[0] = 1<<0;
		dma1_stat[0] = DMA_FIFO_ERR;
	} else if (_DMA1->ISR[0] & (1<<2)) {	// Direct mode error
		_DMA1->IFCR[0] = 1<<2;
		dma1_stat[0] = DMA_DIRECT_ERR;
	} else if (_DMA1->ISR[0] & (1<<3)) {	// Stream transfer error
		_DMA1->IFCR[0] = 1<<3;
		dma1_stat[0] = DMA_STREAM_ERR;
	}
}

void DMA1_Stream1_IRQHandler()
{
	if (_DMA1->ISR[0] & (1<<11)) {	// Transfer Complete
		DMA_Stream_t* stream=DMAStream(_DMA1,1);
		if (dma1_cb[1]) dma1_cb[1](1,!(stream->CR & (1<<19)));
		_DMA1->IFCR[0] = 1<<11;
		dma1_stat[1] = DMA_TC;
	} else if (_DMA1->ISR[0] & (1<<6)) {	// FIFO error
		_DMA1->IFCR[0] = 1<<6;
		dma1_stat[1] = DMA_FIFO_ERR;
	} else if (_DMA1->ISR[0] & (1<<8)) {	// Direct mode error
		_DMA1->IFCR[0] = 1<<8;
		dma1_stat[1] = DMA_DIRECT_ERR;
	} else if (_DMA1->ISR[0] & (1<<9)) {	// Stream transfer error
		_DMA1->IFCR[0] = 1<<9;
		dma1_stat[1] = DMA_STREAM_ERR;
	}
}

void DMA1_Stream2_IRQHandler()
{
	if (_DMA1->ISR[0] & (1<<21)) {	// Transfer Complete
		DMA_Stream_t* stream=DMAStream(_DMA1,2);
		if (dma1_cb[2]) dma1_cb[2](2,!(stream->CR & (1<<19)));
		_DMA1->IFCR[0] = 1<<21;
		dma1_stat[2] = DMA_TC;
	} else if (_DMA1->ISR[0] & (1<<16)) {	// FIFO error
		_DMA1->IFCR[0] = 1<<16;
		dma1_stat[2] = DMA_FIFO_ERR;
	} else if (_DMA1->ISR[0] & (1<<18)) {	// Direct mode error
		_DMA1->IFCR[0] = 1<<18;
		dma1_stat[2] = DMA_DIRECT_ERR;
	} else if (_DMA1->ISR[0] & (1<<19)) {	// Stream transfer error
		_DMA1->IFCR[0] = 1<<19;
		dma1_stat[2] = DMA_STREAM_ERR;
	}
}

void DMA1_Stream3_IRQHandler()
{
	if (_DMA1->ISR[0] & (1<<27)) {	// Transfer Complete
		DMA_Stream_t* stream=DMAStream(_DMA1,3);
		if (dma1_cb[3]) dma1_cb[3](3,!(stream->CR & (1<<19)));
		_DMA1->IFCR[0] = 1<<27;
		dma1_stat[3] = DMA_TC;
	} else if (_DMA1->ISR[0] & (1<<22)) {	// FIFO error
		_DMA1->IFCR[0] = 1<<22;
		dma1_stat[3] = DMA_FIFO_ERR;
	} else if (_DMA1->ISR[0] & (1<<24)) {	// Direct mode error
		_DMA1->IFCR[0] = 1<<24;
		dma1_stat[3] = DMA_DIRECT_ERR;
	} else if (_DMA1->ISR[0] & (1<<25)) {	// Stream transfer error
		_DMA1->IFCR[0] = 1<<25;
		dma1_stat[3] = DMA_STREAM_ERR;
	}
}

void DMA1_Stream4_IRQHandler()
{
	if (_DMA1->ISR[1] & (1<<5)) {	// Transfer Complete
		DMA_Stream_t* stream=DMAStream(_DMA1,4);
		if (dma1_cb[4]) dma1_cb[4](4,!(stream->CR & (1<<19)));
		_DMA1->IFCR[1] = 1<<5;
		dma1_stat[4] = DMA_TC;
	} else if (_DMA1->ISR[1] & (1<<0)) {	// FIFO error
		_DMA1->IFCR[1] = 1<<0;
		dma1_stat[4] = DMA_FIFO_ERR;
	} else if (_DMA1->ISR[1] & (1<<2)) {	// Direct mode error
		_DMA1->IFCR[1] = 1<<2;
		dma1_stat[4] = DMA_DIRECT_ERR;
	} else if (_DMA1->ISR[1] & (1<<3)) {	// Stream transfer error
		_DMA1->IFCR[1] = 1<<3;
		dma1_stat[4] = DMA_STREAM_ERR;
	}
}

void DMA1_Stream5_IRQHandler()
{
	if (_DMA1->ISR[1] & (1<<11)) {	// Transfer Complete
		DMA_Stream_t* stream=DMAStream(_DMA1,5);
		if (dma1_cb[5]) dma1_cb[5](5,!(stream->CR & (1<<19)));
		_DMA1->IFCR[1] = 1<<11;
		dma1_stat[5] = DMA_TC;
	} else if (_DMA1->ISR[1] & (1<<6)) {	// FIFO error
		_DMA1->IFCR[1] = 1<<6;
		dma1_stat[5] = DMA_FIFO_ERR;
	} else if (_DMA1->ISR[1] & (1<<8)) {	// Direct mode error
		_DMA1->IFCR[1] = 1<<8;
		dma1_stat[5] = DMA_DIRECT_ERR;
	} else if (_DMA1->ISR[1] & (1<<9)) {	// Stream transfer error
		_DMA1->IFCR[1] = 1<<9;
		dma1_stat[5] = DMA_STREAM_ERR;
	}
}

void DMA1_Stream6_IRQHandler()
{
	if (_DMA1->ISR[1] & (1<<21)) {	// Transfer Complete
		DMA_Stream_t* stream=DMAStream(_DMA1,6);
		if (dma1_cb[6]) dma1_cb[6](6,!(stream->CR & (1<<19)));
		_DMA1->IFCR[1] = 1<<21;
		dma1_stat[6] = DMA_TC;
	} else if (_DMA1->ISR[1] & (1<<16)) {	// FIFO error
		_DMA1->IFCR[1] = 1<<16;
		dma1_stat[6] = DMA_FIFO_ERR;
	} else if (_DMA1->ISR[1] & (1<<18)) {	// Direct mode error
		_DMA1->IFCR[1] = 1<<18;
		dma1_stat[6] = DMA_DIRECT_ERR;
	} else if (_DMA1->ISR[1] & (1<<19)) {	// Stream transfer error
		_DMA1->IFCR[1] = 1<<19;
		dma1_stat[6] = DMA_STREAM_ERR;
	}
}

void DMA1_Stream7_IRQHandler()
{
	if (_DMA1->ISR[1] & (1<<27)) {	// Transfer Complete
		DMA_Stream_t* stream=DMAStream(_DMA1,7);
		if (dma1_cb[7]) dma1_cb[7](7,!(stream->CR & (1<<19)));
		_DMA1->IFCR[1] = 1<<27;
		dma1_stat[7] = DMA_TC;
	} else if (_DMA1->ISR[1] & (1<<22)) {	// FIFO error
		_DMA1->IFCR[1] = 1<<22;
		dma1_stat[7] = DMA_FIFO_ERR;
	} else if (_DMA1->ISR[1] & (1<<24)) {	// Direct mode error
		_DMA1->IFCR[1] = 1<<24;
		dma1_stat[7] = DMA_DIRECT_ERR;
	} else if (_DMA1->ISR[1] & (1<<25)) {	// Stream transfer error
		_DMA1->IFCR[1] = 1<<25;
		dma1_stat[7] = DMA_STREAM_ERR;
	}
}

/* Interrupt Service Routines DMA2 */
static OnTC dma2_cb[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
static volatile int dma2_stat[8];

void DMA2_Stream0_IRQHandler()
{
	if (_DMA2->ISR[0] & (1<<5)) {	// Transfer Complete
		DMA_Stream_t* stream=DMAStream(_DMA2,0);
		if (dma2_cb[0]) dma2_cb[0](0,!(stream->CR & (1<<19)));
		_DMA2->IFCR[0] = 1<<5;
		dma2_stat[0] = DMA_TC;
	} else if (_DMA2->ISR[0] & (1<<0)) {	// FIFO error
		_DMA2->IFCR[0] = 1<<0;
		dma2_stat[0] = DMA_FIFO_ERR;
	} else if (_DMA2->ISR[0] & (1<<2)) {	// Direct mode error
		_DMA2->IFCR[0] = 1<<2;
		dma2_stat[0] = DMA_DIRECT_ERR;
	} else if (_DMA2->ISR[0] & (1<<3)) {	// Stream transfer error
		_DMA2->IFCR[0] = 1<<3;
		dma2_stat[0] = DMA_STREAM_ERR;
	}
}

void DMA2_Stream1_IRQHandler()
{
	if (_DMA2->ISR[0] & (1<<11)) {	// Transfer Complete
		DMA_Stream_t* stream=DMAStream(_DMA2,1);
		if (dma2_cb[1]) dma2_cb[1](1,!(stream->CR & (1<<19)));
		_DMA2->IFCR[0] = 1<<11;
		dma2_stat[1] = DMA_TC;
	} else if (_DMA2->ISR[0] & (1<<6)) {	// FIFO error
		_DMA2->IFCR[0] = 1<<6;
		dma2_stat[1] = DMA_FIFO_ERR;
	} else if (_DMA2->ISR[0] & (1<<8)) {	// Direct mode error
		_DMA2->IFCR[0] = 1<<8;
		dma2_stat[1] = DMA_DIRECT_ERR;
	} else if (_DMA2->ISR[0] & (1<<9)) {	// Stream transfer error
		_DMA2->IFCR[0] = 1<<9;
		dma2_stat[1] = DMA_STREAM_ERR;
	}
}

void DMA2_Stream2_IRQHandler()
{
	if (_DMA2->ISR[0] & (1<<21)) {	// Transfer Complete
		DMA_Stream_t* stream=DMAStream(_DMA2,2);
		if (dma2_cb[2]) dma2_cb[2](2,!(stream->CR & (1<<19)));
		_DMA2->IFCR[0] = 1<<21;
		dma2_stat[2] = DMA_TC;
	} else if (_DMA2->ISR[0] & (1<<16)) {	// FIFO error
		_DMA2->IFCR[0] = 1<<16;
		dma2_stat[2] = DMA_FIFO_ERR;
	} else if (_DMA2->ISR[0] & (1<<18)) {	// Direct mode error
		_DMA2->IFCR[0] = 1<<18;
		dma2_stat[2] = DMA_DIRECT_ERR;
	} else if (_DMA2->ISR[0] & (1<<19)) {	// Stream transfer error
		_DMA2->IFCR[0] = 1<<19;
		dma2_stat[2] = DMA_STREAM_ERR;
	}
}

void DMA2_Stream3_IRQHandler()
{
	if (_DMA2->ISR[0] & (1<<27)) {	// Transfer Complete
		DMA_Stream_t* stream=DMAStream(_DMA2,3);
		if (dma2_cb[3]) dma2_cb[3](3,!(stream->CR & (1<<19)));
		_DMA2->IFCR[0] = 1<<27;
		dma2_stat[3] = DMA_TC;
	} else if (_DMA2->ISR[0] & (1<<22)) {	// FIFO error
		_DMA2->IFCR[0] = 1<<22;
		dma2_stat[3] = DMA_FIFO_ERR;
	} else if (_DMA2->ISR[0] & (1<<24)) {	// Direct mode error
		_DMA2->IFCR[0] = 1<<24;
		dma2_stat[3] = DMA_DIRECT_ERR;
	} else if (_DMA2->ISR[0] & (1<<25)) {	// Stream transfer error
		_DMA2->IFCR[0] = 1<<25;
		dma2_stat[3] = DMA_STREAM_ERR;
	}
}

void DMA2_Stream4_IRQHandler()
{
	if (_DMA2->ISR[1] & (1<<5)) {	// Transfer Complete
		DMA_Stream_t* stream=DMAStream(_DMA2,4);
		if (dma2_cb[4]) dma2_cb[4](4,!(stream->CR & (1<<19)));
		_DMA2->IFCR[1] = 1<<5;
		dma2_stat[4] = DMA_TC;
	} else if (_DMA2->ISR[1] & (1<<0)) {	// FIFO error
		_DMA2->IFCR[1] = 1<<0;
		dma2_stat[4] = DMA_FIFO_ERR;
	} else if (_DMA2->ISR[1] & (1<<2)) {	// Direct mode error
		_DMA2->IFCR[1] = 1<<2;
		dma2_stat[4] = DMA_DIRECT_ERR;
	} else if (_DMA2->ISR[1] & (1<<3)) {	// Stream transfer error
		_DMA2->IFCR[1] = 1<<3;
		dma2_stat[4] = DMA_STREAM_ERR;
	}
}

void DMA2_Stream5_IRQHandler()
{
	if (_DMA2->ISR[1] & (1<<11)) {	// Transfer Complete
		DMA_Stream_t* stream=DMAStream(_DMA2,5);
		if (dma2_cb[5]) dma2_cb[5](5,!(stream->CR & (1<<19)));
		_DMA2->IFCR[1] = 1<<11;
		dma2_stat[5] = DMA_TC;
	} else if (_DMA2->ISR[1] & (1<<6)) {	// FIFO error
		_DMA2->IFCR[1] = 1<<6;
		dma2_stat[5] = DMA_FIFO_ERR;
	} else if (_DMA2->ISR[1] & (1<<8)) {	// Direct mode error
		_DMA2->IFCR[1] = 1<<8;
		dma2_stat[5] = DMA_DIRECT_ERR;
	} else if (_DMA2->ISR[1] & (1<<9)) {	// Stream transfer error
		_DMA2->IFCR[1] = 1<<9;
		dma2_stat[5] = DMA_STREAM_ERR;
	}
}

void DMA2_Stream6_IRQHandler()
{
	if (_DMA2->ISR[1] & (1<<21)) {	// Transfer Complete
		DMA_Stream_t* stream=DMAStream(_DMA2,6);
		if (dma2_cb[6]) dma2_cb[6](6,!(stream->CR & (1<<19)));
		_DMA2->IFCR[1] = 1<<21;
		dma2_stat[6] = DMA_TC;
	} else if (_DMA2->ISR[1] & (1<<16)) {	// FIFO error
		_DMA2->IFCR[1] = 1<<16;
		dma2_stat[6] = DMA_FIFO_ERR;
	} else if (_DMA2->ISR[1] & (1<<18)) {	// Direct mode error
		_DMA2->IFCR[1] = 1<<18;
		dma2_stat[6] = DMA_DIRECT_ERR;
	} else if (_DMA2->ISR[1] & (1<<19)) {	// Stream transfer error
		_DMA2->IFCR[1] = 1<<19;
		dma2_stat[6] = DMA_STREAM_ERR;
	}
}

void DMA2_Stream7_IRQHandler()
{
	if (_DMA2->ISR[1] & (1<<27)) {	// Transfer Complete
		DMA_Stream_t* stream=DMAStream(_DMA2,7);
		if (dma2_cb[7]) dma2_cb[7](7,!(stream->CR & (1<<19)));
		_DMA2->IFCR[1] = 1<<27;
		dma2_stat[7] = DMA_TC;
	} else if (_DMA2->ISR[1] & (1<<22)) {	// FIFO error
		_DMA2->IFCR[1] = 1<<22;
		dma2_stat[7] = DMA_FIFO_ERR;
	} else if (_DMA2->ISR[1] & (1<<24)) {	// Direct mode error
		_DMA2->IFCR[1] = 1<<24;
		dma2_stat[7] = DMA_DIRECT_ERR;
	} else if (_DMA2->ISR[1] & (1<<25)) {	// Stream transfer error
		_DMA2->IFCR[1] = 1<<25;
		dma2_stat[7] = DMA_STREAM_ERR;
	}
}

/* API */
#define STRM_P2M				(0<<6)	
#define STRM_M2P				(1<<6)
#define STRM_M2M				(2<<6)

#define XFERMODE(s) ((s->CR) & (3<<6))

DMA_Stream_t* dma_stream_init(DMA_t* dma, uint32_t stream, DMAEndPoint_t *src, DMAEndPoint_t *dest, uint32_t mode, OnTC cb)
{
	DMA_Stream_t* s;
	IRQn_t irqn;
	uint32_t irqprio;
	uint32_t xfer = 0xFFFFFFFF;
	
	if (dma==_DMA1) {
		if (stream>7) {		/* bad stream number */
			return NULL;
		}
		_RCC->AHB1ENR |= 1<<21;
		switch (stream) {
			case 0: irqn=DMA1_Stream0_IRQn; irqprio = DMA1_STREAM0_PRIORITY; break;
			case 1: irqn=DMA1_Stream1_IRQn; irqprio = DMA1_STREAM1_PRIORITY; break;
			case 2: irqn=DMA1_Stream2_IRQn; irqprio = DMA1_STREAM2_PRIORITY; break;
			case 3: irqn=DMA1_Stream3_IRQn; irqprio = DMA1_STREAM3_PRIORITY; break;
			case 4: irqn=DMA1_Stream4_IRQn; irqprio = DMA1_STREAM4_PRIORITY; break;
			case 5: irqn=DMA1_Stream5_IRQn; irqprio = DMA1_STREAM5_PRIORITY; break;
			case 6: irqn=DMA1_Stream6_IRQn; irqprio = DMA1_STREAM6_PRIORITY; break;
			case 7: irqn=DMA1_Stream7_IRQn; irqprio = DMA1_STREAM7_PRIORITY; break;
		}
		
		s = (DMA_Stream_t*)((char*)dma+0x10+0x18*stream);
		
	} else if (dma==_DMA2) {
		if (stream>7) {		/* bad stream number */
			return NULL;
		}
		_RCC->AHB1ENR |= 1<<22;
		switch (stream) {
			case 0: irqn=DMA2_Stream0_IRQn; irqprio = DMA2_STREAM0_PRIORITY; break;
			case 1: irqn=DMA2_Stream1_IRQn; irqprio = DMA2_STREAM1_PRIORITY; break;
			case 2: irqn=DMA2_Stream2_IRQn; irqprio = DMA2_STREAM2_PRIORITY; break;
			case 3: irqn=DMA2_Stream3_IRQn; irqprio = DMA2_STREAM3_PRIORITY; break;
			case 4: irqn=DMA2_Stream4_IRQn; irqprio = DMA2_STREAM4_PRIORITY; break;
			case 5: irqn=DMA2_Stream5_IRQn; irqprio = DMA2_STREAM5_PRIORITY; break;
			case 6: irqn=DMA2_Stream6_IRQn; irqprio = DMA2_STREAM6_PRIORITY; break;
			case 7: irqn=DMA2_Stream7_IRQn; irqprio = DMA2_STREAM7_PRIORITY; break;
		}
	
		s = (DMA_Stream_t*)((char*)dma+0x10+0x18*stream);
	} else {
		return NULL;
	}
	
	/* clear interrupt flags if any */
	dma->IFCR[stream>>2] = 0x3D<<(((stream&2)<<3)+((stream&1)*6));
	
	/* process stream general options */
	s->CR = mode & 0xFFFFFFF8;
	
	// set FCR[2]: FIFO on(1)/off(0)=DIRECT mode
	s->FCR = mode & 0x7;
	
	/* process src endpoint options */
	if (src->type==EP_MEM) {
		/* cfg<<2 corresponds to CR but bit 9 */
		s->CR |= ((src->cfg & ~(1<<9))<<2) | ((src->cfg & (1<<9))<<1);
		s->M0AR = (uint32_t)src->addr0;
		if (src->cfg & (1<<16)) {					// Double buffering
			s->M1AR = (uint32_t)src->addr1;
		}
	} else {
		xfer = STRM_P2M;
		s->CR |= (src->channel<<25) | STRM_P2M;		// P2M transfer
		s->CR |= (src->cfg & ~((1<<16) | (1<<6)));	// src->cfg corresponds to CR but bits 6 & 16
		s->PAR = (uint32_t)src->addr0;
	}
	
	/* process dest endpoint options */
	if (dest->type==EP_MEM) {
		/* cfg<<2 corresponds to CR but bit 9 */
		s->CR |= ((dest->cfg & ~(1<<9))<<2) | ((dest->cfg & (1<<9))<<1);
		if (xfer==STRM_P2M) {						// P2M transfer
			s->M0AR = (uint32_t)dest->addr0;
			if (dest->cfg & (1<<16)) {
				s->M1AR = (uint32_t)dest->addr1;
			}
		} else {									// M2M transfer
			if (dma==_DMA1) {
				dma1_stat[stream]=DMA_UNSUPPORTED;
				return NULL;
			}
			s->CR |= STRM_M2M;
			s->PAR = (uint32_t)dest->addr0;
		}
	} else {
		if (xfer==STRM_P2M) return NULL;			// cannot do Periph2Periph
		s->CR |= (dest->channel<<25) | STRM_M2P;		// M2P transfer
		s->CR |= (dest->cfg & ~((1<<16) | (1<<6)));
		s->PAR = (uint32_t)dest->addr0;
	}
	
	if (cb) {
		if (dma==_DMA1) {
			dma1_cb[stream] = cb;
		} else if (dma==_DMA2) {
			dma2_cb[stream] = cb;
		}
		s->CR |= 1<<4;			// Transfer Complete Interrupt enabled
		NVIC_SetPriority(irqn, irqprio);
		NVIC_EnableIRQ(irqn);
	}

	return s;
}


int dma_start(DMA_Stream_t* s, uint16_t size)
{
	if (!s) return DMA_BAD_STREAM;
	
	if ( size==0 && ((s->CR>>6) & 3)!=2 ) {	// size==0 and not Mem2Mem
		s->CR |= (1<<5);					// Peripheral Flow Control
	} else {
		s->NDTR = size;						// DMA Flow control
	}
	
	s->CR |= 1;		// start DMA
	return 0;
}

int dma_stop(DMA_Stream_t* s)
{
	if (!s) return DMA_BAD_STREAM;
	
	s->CR &= ~1;	// stop DMA
	return 0;
}

int dma_complete(DMA_Stream_t *s)
{
	return (s->CR & 1)==0;
}

int dma_status(DMA_t* dma, uint32_t stream)
{
	if (stream>7) return DMA_BAD_STREAM;
	
	if (dma==_DMA1) return dma1_stat[stream];
	if (dma==_DMA2) return dma2_stat[stream];
	
	return DMA_BAD_DMA;
}
