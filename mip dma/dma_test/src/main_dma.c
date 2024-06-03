#include <stdio.h>
#include <string.h>
#include "include/board.h"
#include "lib/io.h"

#define MAIN1

#ifdef MAIN1
/***************************************************************************
 * UART with DMA transmit and receive test (raw)
 *
 ***************************************************************************/

char buf_tx[64]="\r\nThis message comes from STM32F411 :-)\r\n";
char buf_rx[64];

int main()
{
	// enable USART2 clocking
	_RCC->APB1ENR |= 1<<17;
	
	// configure Tx/Rx pins : Tx --> PA2, Rx --> PA3
	io_configure(USART2_GPIO_PORT, USART2_GPIO_PINS, USART2_GPIO_CONFIG, NULL);
	
	// transmission speed : 115200 bauds
    _USART2->BRR = (sysclks.apb1_freq / 115200);
    
	_USART2->GTPR = 0;
	
	// config 8 bit data, 1 stop bit, no parity
	_USART2->CR1 = (1<<3) | (1<<2) | (1<<13);
	_USART2->CR2 = 0;
	_USART2->CR3 = 3<<6; // DMA Transmit and receive
	
	// Config DMA to transmit a string
	_RCC->AHB1ENR |= 1<<21;
	_DMA1->IFCR[1]      = 0x3D<<16;
	_DMA1_Stream6->M0AR = (uint32_t)buf_tx;
	_DMA1_Stream6->PAR  = (uint32_t)&_USART2->DR;		// USART2_BASE+4 = DR address 0x40004404;
	_DMA1_Stream6->NDTR = strlen(buf_tx);
	_DMA1_Stream6->FCR  = 0;
	_DMA1_Stream6->CR   =  (4<<25) | (1<<10) | (1<<6);
	_DMA1_Stream6->CR   |=  (1<<0);
	
	while(_DMA1_Stream6->CR & 1) ;
	
	while(1) {
		// read 10 char data
		_DMA1->IFCR[1]      = 0x3D<<6;
		_DMA1_Stream5->M0AR = (uint32_t)buf_rx;
		_DMA1_Stream5->PAR  = (uint32_t)&_USART2->DR;	// USART2_BASE+4 = DR address 0x40004404;
		_DMA1_Stream5->NDTR = 10;
		_DMA1_Stream5->FCR  = 0;
		_DMA1_Stream5->CR   = (4<<25) | (1<<10);
		_DMA1_Stream5->CR   |= (1<<0);
		while(_DMA1_Stream5->CR & 1) ;
		buf_rx[10]=0;
		// echo the data
		_DMA1->IFCR[1]      = 0x3D<<16;
		_DMA1_Stream6->M0AR = (uint32_t)buf_rx;
		_DMA1_Stream6->PAR  = (uint32_t)&_USART2->DR;	// USART2_BASE+4 = DR address 0x40004404;
		_DMA1_Stream6->NDTR = strlen(buf_rx);
		_DMA1_Stream6->FCR  = 0;
		_DMA1_Stream6->CR   =  (4<<25) | (1<<10) | (1<<6);
		_DMA1_Stream6->CR   |=  (1<<0);
		while(_DMA1_Stream6->CR & 1) ; 
	}
	return 1;
}
#endif

#ifdef MAIN2
/***************************************************************************
 * UART with DMA transmit and receive test (API)
 *
 ***************************************************************************/
#include <string.h>
#include "lib/uart.h"
#include "lib/dma.h"

/*
 * uart_init : polling Tx and IRQ Rx
 */
int uart_init(USART_t *u, uint32_t baud, uint32_t mode)
{
	if (u == _USART1) {
#ifdef USE_USART1
		// Reset USART
		_RCC->APB2RSTR |= 1<<4;
		_RCC->APB2RSTR &= ~(1<<4);
		// enable USART clocking
		_RCC->APB2ENR |= 1<<4;
		
		// configure Tx/Rx pins : Tx -->, Rx --> 
		io_configure(USART1_GPIO_PORT, USART1_GPIO_PINS, USART1_GPIO_CONFIG, NULL);

        //u->BRR = (sysclks.apb2_freq / baud);
        u->BRR = ((2 * sysclks.apb2_freq) + baud) / (2 * baud);
#else
		return -1;
#endif
	 } else if (u == _USART2) {
#ifdef USE_USART2
		// Reset USART
		_RCC->APB1RSTR |= 1<<17;
		_RCC->APB1RSTR &= ~(1<<17);
		// enable USART clocking
		_RCC->APB1ENR |= 1<<17;
	
		// configure Tx/Rx pins : Tx --> PA2, Rx --> PA3
		io_configure(USART2_GPIO_PORT, USART2_GPIO_PINS, USART2_GPIO_CONFIG, NULL);
		
        //u->BRR = (sysclks.apb1_freq / baud);
        u->BRR = ((2 * sysclks.apb1_freq) + baud) / (2 * baud);
#else
		return -1;
#endif
	} else if (u == _USART6) {
#ifdef USE_USART6
		// Reset USART
		_RCC->APB2RSTR |= 1<<5;
		_RCC->APB2RSTR &= ~(1<<5);
		// enable USART clocking
		_RCC->APB2ENR |= 1<<5;
		
		// configure Tx/Rx pins
		io_configure(USART6_GPIO_PORT, USART6_GPIO_PINS, USART6_GPIO_CONFIG, NULL);

        //u->BRR = (sysclks.apb2_freq / baud);
        u->BRR = ((2 * sysclks.apb2_freq) + baud) / (2 * baud);
#else
		return -1;
#endif
	} else {
		return -1;
	}
	
	/* Configure peripheral */
	u->GTPR = 0;
	u->CR3 = 0;
	u->CR2 = (((mode >> 4) & 0x3) << 12);
	u->CR1 = ((mode & 0x1) << 12) | (((mode >> 8) & 0x7) << 8) |
	         (1<<3) | (1<<2) | (1<<13);
	
    return 0;
}

int uart_puts(USART_t *u, char *s)
{
	int len = 0;
	
	DMAEndPoint_t ep_buf = {			// src
		.type    = EP_MEM,
		//.addr0   = A COMPLETER,
		.addr1   = NULL,
		.channel = -1,
		//.cfg     = A COMPLETER
	};
	
	DMAEndPoint_t dma1s6_uart2_tx = {	// dest
		.type    = EP_UART_TX,
		//.addr0   = A COMPLETER,
		.addr1   = NULL,
		//.channel = A COMPLETER,
		//.cfg     = A COMPLETER
	};
	
	if (!s || s[0]==0) return 0;

	DMA_Stream_t *strm = dma_stream_init(_DMA1, /*A MODIFIER*/0, &ep_buf, &dma1s6_uart2_tx, STRM_FIFO_TH4, NULL);
	
	len = strlen(s);
	u->CR3 |= 1<<7;					// enable DMA Transmit
	dma_start(strm,len);
	while (!dma_complete(strm)) ;	// wait until transmission complete
	u->CR3 &= ~(1<<7);				// disable DMA Transmit
	return len;
}

int uart_gets(USART_t *u, char *buf, int len)
{
	DMAEndPoint_t ep_buf = {			// dest
		.type    = EP_MEM,
		//.addr0   = A COMPLETER,
		.addr1   = NULL,
		.channel = -1,
		//.cfg     = A COMPLETER
	};
	DMAEndPoint_t dma1s5_uart2_rx = {	// src
		.type    = EP_UART_RX,
		//.addr0   = A COMPLETER,
		.addr1   = NULL,
		//.channel = A COMPLETER,
		//.cfg     = A COMPLETER
	};

	if (!buf) return 0;
	if (!len) {
		buf[0]=0;
		return 0;
	}

	DMA_Stream_t* strm = dma_stream_init(_DMA1, /*A MODIFIER*/0, &dma1s5_uart2_rx, &ep_buf, 0, NULL);
	u->CR3 |= 1<<6;					// enable DMA Receive
	dma_start(strm,len);
	while (!dma_complete(strm)) ;	// wait until transmission complete
	u->CR3 &= ~(1<<6);				// disable DMA Receive
	return len;
}


char buf_tx[64]="This message comes from STM32F411 :-)\r\n";
char buf_rx[64];


int main()
{
	uart_init(_USART2,115200,UART_8N1);
	uart_puts(_USART2,buf_tx);
		
	while (1) {
		uart_gets(_USART2,buf_rx,10);
		uart_puts(_USART2,"\r\nLe texte : ");
		uart_puts(_USART2,buf_rx);
		uart_puts(_USART2,"\r\n");
	}

	return 1;
}
#endif
