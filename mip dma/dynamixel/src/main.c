#include <stdio.h>
#include "include/board.h"
#include "lib/io.h"
#include "lib/uart.h"
#include "lib/timer.h"
#include "lib/tick.h"
#include "dynamixel.h"

#define MAIN3

#ifdef MAIN1
/****************************************************************
 * UART with Tx and Rx IRQs
 *  allow to watch "by hardware" the extent of
 *  - uart_puts function when watching the PA10 signal
 *  - USART2 ISR when watching the PB5 signal (add the flag SHOWIRQ2
 *    ( -DSHOWIRQ2) in the makefile.
 ****************************************************************/

#define delay(us)	timer_wait_us(_TIM2,(us),NULL)

char mystr[]="Ceci est un message ...\r\n";
char mystr2[]="Hello World hé! hé!\r\n";

int main()
{
	io_configure(_GPIOA,PIN_10,PIN_MODE_OUTPUT|PIN_OPT_OUTPUT_SPEED_FAST,NULL);
	io_clear(_GPIOA,PIN_10);
	
#ifdef SHOWIRQ2
	io_configure(_GPIOB,PIN_5,PIN_MODE_OUTPUT|PIN_OPT_OUTPUT_SPEED_FAST,NULL);
	io_clear(_GPIOB,PIN_5);
#endif

	uart_init(_USART2, 115200, UART_8N1);

#if 0
	delay(100);
	
	io_set(_GPIOA, PIN_10);
	uart_putc(_USART2, 'A');
	uart_putc(_USART2, 'B');
	uart_putc(_USART2, 'D');
	io_clear(_GPIOA, PIN_10);
#endif

	uart_puts(_USART2, "\x1B[2J\x1B[H");

	delay(100);
	
	io_set(_GPIOA, PIN_10);
	uart_puts(_USART2, mystr);
	while (!uart_tx_completed(_USART2)) ;
	io_clear(_GPIOA, PIN_10);
	
#if 0
	uart_puts(_USART2, mystr2);
	uart_printf(_USART2, "_USART2 = %x\r\n", _USART2);
#endif

	while (1) {
		char c= uart_getc(_USART2);
		uart_putc(_USART2, c);
	}

	return 1;
}
#endif

#ifdef MAIN2
/***************************************************************************
 * DYNAMIXEL test : LED / Velocity Mode / Position Mode
 *
 *  USART6 is used for motor communication. watching Tx and Rx lines may be
 *  of some interest.
 *  You can watch "by hardware" the extent of
 *  - dxl_write_packet function when watching the PA10 signal
 *  - USART6 ISR when watching the PB5 signal (add the flag SHOWIRQ2
 *    ( -DSHOWIRQ2) in the makefile.
 ***************************************************************************/
#define DYN_ID	1

#define delay(ms)	timer_wait_ms(_TIM2,ms,NULL)

int main()
{
	uint32_t pos;
	
#ifdef SHOWIRQ6
	io_configure(_GPIOB,PIN_5,PIN_MODE_OUTPUT|PIN_OPT_OUTPUT_SPEED_FAST,NULL);
	io_clear(_GPIOB,PIN_5);
#endif

	uart_init(_USART2, 115200, UART_8N1);
	dxl_init();
	
	uart_puts(_USART2, "\x1B[2J\x1B[H");
	
#if 0
	uint8_t firmware_version = dxl_get_firmware_version(DYN_ID);
	uint32_t baudrate = dxl_get_baudrate(DYN_ID);
	uint8_t model_number = dxl_get_model_number(DYN_ID);
#endif
	
	dxl_LED(DYN_ID, LED_ON);
	delay(1000);
	dxl_LED(DYN_ID, LED_OFF);
	delay(1000);
	dxl_LED(DYN_ID, LED_ON );
	delay(1000);
	dxl_LED(DYN_ID, LED_OFF);
	delay(1000);

	dxl_set_operating_mode(DYN_ID, POSITION_MODE);
	dxl_set_goal_position(DYN_ID,1024);
	delay(2000);
	pos = dxl_get_cur_position(DYN_ID);
	uart_printf(_USART2, "pos = %d\r\n", pos);
	dxl_set_goal_position(DYN_ID,0);
	delay(2000);
	pos = dxl_get_cur_position(DYN_ID);
	uart_printf(_USART2, "pos = %d\r\n", pos);

	dxl_set_operating_mode(DYN_ID, VELOCITY_MODE);
	dxl_set_goal_velocity(DYN_ID, -140);

	uint32_t velocity = 0;

	for (int i=0 ; i<5 ; i++) {
		velocity = dxl_get_cur_velocity(DYN_ID);
		uart_printf(_USART2, "vel = %d\r\n", velocity);
		delay(1000); // 1000 ms
	}
	dxl_set_goal_velocity(DYN_ID, 0);
	
	uart_puts(_USART2, "... end of demo ...\r\n");
	
	while (1) {		
	}

	return 1;
}
#endif

#ifdef MAIN3
/***************************************************************************
 * DYNAMIXEL remote control : LED / Velocity Mode / Position Mode
 *
 *  USART6 is used for motor communication. watching Tx and Rx lines may be
 *  of some interest.
 *  You can watch "by hardware" the extent of
 *  - dxl_write_packet function when watching the PA10 signal
 *  - USART6 ISR when watching the PB5 signal (add the flag SHOWIRQ2
 *    ( -DSHOWIRQ2) in the makefile.
 *
 *  USART2 is used for remote control from a PC. Accepted commands:
 *  - X                : broadcast ping for enumeration
 *  - R<ID>\n          : factory reset
 *  - I<ID>;<newID>\n  : set new ID
 *  - B<ID>\n          : run a blink test
 *  - P<ID>;<newID>\n  : run a position test
 *  - V<ID>;<newID>\n  : run a velocity test
 *
 *  On the PC, run
 *
 *    tclsh dynamixel.tcl
 *
 ***************************************************************************/

#define delay(ms)	timer_wait_ms(_TIM2,ms,NULL)

void test_blink(uint8_t id, int delay_ms)
{
	dxl_LED(id, LED_ON);
	delay(delay_ms);
	dxl_LED(id, LED_OFF);
	delay(delay_ms);
	dxl_LED(id, LED_ON );
	delay(delay_ms);
	dxl_LED(id, LED_OFF);
	delay(delay_ms);
}

int get_number(char *end)
{
	int val = 0;
	char c;
	while ((c=uart_getc(_USART2))>='0' && c<='9') {
		val = 10*val + c - '0';
	}
	*end=c;
	return val;
}

int main()
{
	char c;
	int pos = 0;
	int vel = 0;
	uint8_t id = 1, nid;
	int n;
	uint8_t idbuf[10];
	
#ifdef SHOWIRQ6
	io_configure(_GPIOB,PIN_5,PIN_MODE_OUTPUT|PIN_OPT_OUTPUT_SPEED_FAST,NULL);
	io_clear(_GPIOB,PIN_5);
#endif
	// initialize USART2 & USART6 (DXL)
	uart_init(_USART2, 115200, UART_8N1);
	dxl_init();
	
	// start tick counter 1ms
	tick_init(1,NULL);
	
	n=dxl_ping(BROADCAST, idbuf);
	if (n) {
		uart_printf(_USART2, "X%d", idbuf[0]);
		for (int i=1; i<n; i++) {
			uart_printf(_USART2, ",%d", idbuf[i]);
		}
		uart_printf(_USART2, "\n");
	}
	
	while (1) {
		c = uart_getc(_USART2);
		switch (c) {
			case 'X':
#if 1
				n=dxl_ping(BROADCAST, idbuf);
#else
				// for test purpose only
				idbuf[0]=1;
				idbuf[1]=3;
				idbuf[2]=4;
				n=3;
#endif
				if (n) {
					uart_printf(_USART2, "X%d", idbuf[0]);
					for (int i=1; i<n; i++) {
						uart_printf(_USART2, ",%d", idbuf[i]);
					}
					uart_printf(_USART2, "\n");
				}
				break;
			case 'R':
				id = (uint8_t)get_number(&c);
				if (c=='\n') dxl_factory_reset(id);
				delay(4000);
				n=dxl_ping(BROADCAST, idbuf);
				if (n) {
					uart_printf(_USART2, "X%d", idbuf[0]);
					for (int i=1; i<n; i++) {
						uart_printf(_USART2, ",%d", idbuf[i]);
					}
					uart_printf(_USART2, "\n");
				}
				break;
			case 'I':
				id = (uint8_t)get_number(&c);
				if (c==';') {
					nid=get_number(&c);
					if (c=='\n') {
						dxl_set_id(id, nid);
						test_blink(nid,400);
						n=dxl_ping(BROADCAST, idbuf);
						if (n) {
							uart_printf(_USART2, "X%d", idbuf[0]);
							for (int i=1; i<n; i++) {
								uart_printf(_USART2, ",%d", idbuf[i]);
							}
							uart_printf(_USART2, "\n");
						}
					}
				}
				break;
			case 'B':
				id = (uint8_t)get_number(&c);
				if (c=='\n') test_blink(id,1000);
				break;
			case 'P':
				id = (uint8_t)get_number(&c);
				if (c==';') {
					pos=get_number(&c);
					if (c=='\n') {
						dxl_set_operating_mode(id, POSITION_MODE);
						dxl_set_goal_position(id,pos);
						delay(500);
						pos = dxl_get_cur_position(id);
						uart_printf(_USART2, "pos = %d\n", pos);
					}
				}
				break;
			case 'V':
				id = (uint8_t)get_number(&c);
				if (c==';') {
					vel=get_number(&c);
					if (c=='\n') {
						dxl_set_operating_mode(id, VELOCITY_MODE);
						dxl_set_goal_velocity(id, vel);
						delay(500);
						vel = dxl_get_cur_velocity(id);
						uart_printf(_USART2, "vel = %d\n", vel);
					}
				}
				break;
			default:
				break;
		}
	}
	
#if 0
	uint8_t firmware_version = dxl_get_firmware_version(id);
	uint32_t baudrate = dxl_get_baudrate(id);
	uint8_t model_number = dxl_get_model_number(id);
#endif
	
	return 1;
}
#endif
