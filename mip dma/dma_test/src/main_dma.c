#include <stdio.h>
#include <string.h>
#include "include/board.h"
#include "lib/io.h"
#include "lib/uart.h"
#include "lib/dma.h"

#define MAIN2

#ifdef MAIN2
/***************************************************************************
 * UART with DMA transmit and receive test (API)
 *
 ***************************************************************************/

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

    DMAEndPoint_t ep_buf = {            // src
        .type    = EP_MEM,
        .addr0   = s,
        .addr1   = NULL,
        .channel = -1,
        .cfg     = (1<<10)
    };

    DMAEndPoint_t dma1s6_uart2_tx = {    // dest
        .type    = EP_UART_TX,
        .addr0   = ((void*)&u->DR),
        .addr1   = NULL,
        .channel = 4,
        .cfg     = (0<<10)
    };

    if (!s || s[0]==0) return 0;

    DMA_Stream_t *strm = dma_stream_init(_DMA1, 6, &ep_buf, &dma1s6_uart2_tx, STRM_FIFO_TH4, NULL);

    len = strlen(s);
    u->CR3 |= 1<<7;                    // enable DMA Transmit
    dma_start(strm,len);
    while (!dma_complete(strm)) ;    // wait until transmission complete
    u->CR3 &= ~(1<<7);                // disable DMA Transmit
    return len;
}

int uart_gets(USART_t *u, char *buf, int len)
{
    DMAEndPoint_t ep_buf = {            // dest
        .type    = EP_MEM,
        .addr0   = buf,
        .addr1   = NULL,
        .channel = -1,
        .cfg     = (1<<10)
    };
    DMAEndPoint_t dma1s5_uart2_rx = {    // src
        .type    = EP_UART_RX,
        .addr0   = ((void*)&u->DR),
        .addr1   = NULL,
        .channel = 4,
        .cfg     = (0<<10)
    };

    if (!buf) return 0;
    if (!len) {
        buf[0]=0;
        return 0;
    }

    DMA_Stream_t* strm = dma_stream_init(_DMA1, 5, &dma1s5_uart2_rx, &ep_buf, 0, NULL);
    u->CR3 |= 1<<6;                    // enable DMA Receive
    dma_start(strm,len);
    while (!dma_complete(strm)) ;    // wait until reception complete
    u->CR3 &= ~(1<<6);                // disable DMA Receive
    return len;
}

char buf_tx[64]="This message comes from STM32F411 :-)\r\n";
char buf_rx[64];

int main()
{
    uart_init(_USART2, 115200, UART_8N1);
    uart_puts(_USART2, buf_tx);

    while (1) {
        uart_gets(_USART2, buf_rx, 10);
        buf_rx[10] = '\0';  // Ensure null-termination
        uart_puts(_USART2, "\r\nLe texte : ");
        uart_puts(_USART2, buf_rx);
        uart_puts(_USART2, "\r\n");
    }

    return 1;
}
#endif
