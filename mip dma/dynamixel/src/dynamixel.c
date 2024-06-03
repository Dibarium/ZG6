/*
 * dynamixel.c
 *
 *  Created on: Oct 10, 2019
 *      Author: kerhoas
 *		Modified on march 2022, Eric Boucharé
 */

#include <stdlib.h>
#include <stdarg.h>
#include "dynamixel.h"
#include "lib/io.h"
#include "lib/uart.h"
#include "lib/timer.h"

/****************************************************************
 * Macros
 ****************************************************************/
// Packet Write Enable signal : direction of DYNAMIXEL packet
#define PCKT_WREN_PORT	_GPIOA
#define PCKT_WREN_PIN	PIN_10
#define PCKT_WR_EN		io_set(PCKT_WREN_PORT,PCKT_WREN_PIN)
#define PCKT_WR_DIS		io_clear(PCKT_WREN_PORT,PCKT_WREN_PIN)

// used USART
#define USART			_USART6
#define BAUDRATE		57600
#define TIMEOUT			100

//################################################################
#define DXL_DEBUG		0

//################################################################
#define DXL_BYTE_1(w)      ((uint8_t)(((uint32_t)(w)) & 0xff))			// LOW BYTE
#define DXL_BYTE_2(w)      ((uint8_t)(((uint32_t)(w) >>8) & 0xff))
#define DXL_BYTE_3(w)      ((uint8_t)(((uint32_t)(w) >>16) & 0xff))
#define DXL_BYTE_4(w)      ((uint8_t)(((uint32_t)(w) >> 24) & 0xff))  	// HIGHT BYTE
//==================================================================
// https://emanual.robotis.com/docs/en/dxl/protocol2/
// https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table
// https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/
//==================================================================
#define INST_PING			1
#define INST_READ			2
#define INST_WRITE			3
#define INST_RESET			6

#define XL_TORQUE 			64
#define XL_LED				65

#define XL_FIRMWARE_VERSION	6
#define XL_ID				7
#define XL_BAUDRATE			8
#define XL_MODEL_NUMBER 	0

#define XL_OPERATING_MODE	11
#define XL_GOAL_VELOCITY	104
#define XL_GOAL_POSITION	116

#define XL_PRESENT_VELOCITY	128
#define XL_PRESENT_POSITION	132

#define XL_RESET 1

static int dxl_write_packet(int id, int instruction, int parameter_data_size, ...);
static int dxl_read_packet();

/************************************************************************
 * Configuration
 *   get info about firmwareversion and device model
 *   factory reset
 *   get/set baudrate
 *   set id
 *   set operating mode
 ************************************************************************/
int dxl_ping(uint8_t id, uint8_t *idbuf)
{
	int n, idn=0;
	uint8_t buf[4];
	// broadcast ping
	dxl_write_packet(id, INST_PING, 0);
	
	while ((n=dxl_read_packet(buf))!=-1) {
		idbuf[idn++]=buf[0];
	}
	return idn;
}
//==============================================================
uint8_t dxl_get_firmware_version(uint8_t id)
{
	uint8_t buf[20];
	dxl_write_packet(id, INST_READ, 4,  DXL_BYTE_1(XL_FIRMWARE_VERSION),  DXL_BYTE_2(XL_FIRMWARE_VERSION), 0x01,0x00 );
	if((dxl_read_packet(buf)!=-1) && (buf[1]==0x00)) {
		return buf[2];
	}
	return -1;
}
//==============================================================
uint8_t dxl_get_model_number(uint8_t id)
{
	uint8_t buf[20];
	dxl_write_packet(id, INST_READ, 4,  DXL_BYTE_1(XL_MODEL_NUMBER),  DXL_BYTE_2(XL_MODEL_NUMBER), 0x01,0x00 );
	if((dxl_read_packet(buf)!=-1) && (buf[1]==0x00)) {
		return buf[2];
	}
	return -1;
}
//==============================================================
void dxl_factory_reset(uint8_t id)
{
	uint8_t buf[20];
	dxl_write_packet(id, INST_RESET, 1, 0xFF );
	dxl_read_packet(buf);
}
//==============================================================
void dxl_set_baudrate(uint8_t id, uint8_t val)
{
	uint8_t buf[20];
	dxl_write_packet(id, INST_WRITE, 3, DXL_BYTE_1(XL_BAUDRATE), DXL_BYTE_2(XL_BAUDRATE), val );
	dxl_read_packet(buf);
}
//==============================================================
uint8_t dxl_get_baudrate(uint8_t id)
{
	uint8_t buf[20];
	dxl_write_packet(id, INST_READ, 4,  DXL_BYTE_1(XL_BAUDRATE),  DXL_BYTE_2(XL_BAUDRATE), 0x01,0x00 );
	if((dxl_read_packet(buf)!=-1) && (buf[1]==0x00)) {
		return buf[2];
	}
	return -1;
}
//==============================================================
void dxl_set_id(uint8_t id, uint8_t new_id)
{
	uint8_t buf[20];
	dxl_torque(id, TORQUE_OFF);
    dxl_write_packet(id, INST_WRITE, 3, DXL_BYTE_1(XL_ID), DXL_BYTE_2(XL_ID), new_id );
    dxl_read_packet(buf);
	dxl_torque(new_id, TORQUE_ON);
}
//==============================================================
void dxl_set_operating_mode(uint8_t id, uint8_t val)
{
	uint8_t buf[20];
	dxl_torque(id, TORQUE_OFF);
	dxl_write_packet(id, INST_WRITE, 3, DXL_BYTE_1(XL_OPERATING_MODE), DXL_BYTE_2(XL_OPERATING_MODE), val );
	dxl_read_packet(buf);
	dxl_torque(id, TORQUE_ON);
}
//==============================================================
void dxl_torque(uint8_t id, uint8_t val)
{
	uint8_t buf[20];
	dxl_write_packet(id, INST_WRITE, 3, DXL_BYTE_1(XL_TORQUE), DXL_BYTE_2(XL_TORQUE), val );
	dxl_read_packet(buf);
}

/************************************************************************
 * Control / status
 *   switch LED on/off
 *   define goal torque, position, velocity depending on operating
 *     mode.
 *   get current position / velocity
 ************************************************************************/
void dxl_LED(uint8_t id, uint8_t val )
{
	uint8_t buf[20];
	dxl_write_packet(id, INST_WRITE, 3, DXL_BYTE_1(XL_LED), DXL_BYTE_2(XL_LED), val );
	dxl_read_packet(buf);
}
//==============================================================
void dxl_set_goal_velocity(uint8_t id, int val)
{
	uint8_t buf[20];
	dxl_write_packet(id, INST_WRITE, 6, DXL_BYTE_1(XL_GOAL_VELOCITY), DXL_BYTE_2(XL_GOAL_VELOCITY),
					DXL_BYTE_1(val), DXL_BYTE_2(val), DXL_BYTE_3(val), DXL_BYTE_4(val) );
	dxl_read_packet(buf);
}
//==============================================================
void dxl_set_goal_position(uint8_t id, int val)
{
	uint8_t buf[20];
	dxl_write_packet(id, INST_WRITE, 6, DXL_BYTE_1(XL_GOAL_POSITION), DXL_BYTE_2(XL_GOAL_POSITION),
					DXL_BYTE_1(val), DXL_BYTE_2(val), DXL_BYTE_3(val), DXL_BYTE_4(val) );
	dxl_read_packet(buf);
}
//==============================================================
uint32_t dxl_get_cur_position(uint8_t id)
{
	uint8_t buf[20];
	dxl_write_packet(id, INST_READ, 4, DXL_BYTE_1(XL_PRESENT_POSITION),DXL_BYTE_2(XL_PRESENT_POSITION), 0x04,0x00 );
	if((dxl_read_packet(buf)!=-1) && (buf[1]==0x00)) {
		return (uint32_t)buf[5]<<24 | (uint32_t)buf[4]<<16 | (uint32_t)buf[3]<<8 | (uint32_t)buf[2];
	}
	return -1;
}
//==============================================================
int32_t dxl_get_cur_velocity(uint8_t id)
{
	uint8_t buf[20];
	dxl_write_packet(id, INST_READ, 4, DXL_BYTE_1(XL_PRESENT_VELOCITY),DXL_BYTE_2(XL_PRESENT_VELOCITY), 0x04,0x00 );
	if((dxl_read_packet(buf)!=-1) && (buf[1]==0x00)) {
		return (uint32_t)buf[5]<<24 | (uint32_t)buf[4]<<16 | (uint32_t)buf[3]<<8 | (uint32_t)buf[2];
	}
	return -1;
}


/***********************************************************************
 * Low level packet IO functions
 ***********************************************************************/
static uint16_t dxl_updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
	uint16_t i, j;
	uint16_t crc_table[256] = {
		0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
		0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
		0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
		0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
		0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
		0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
		0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
		0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
		0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
		0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
		0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
		0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2, 
		0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
		0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
		0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
		0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
		0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
		0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
		0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
		0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
		0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
		0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
		0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
		0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
		0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
		0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
		0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
		0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
		0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
		0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
		0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
		0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
	};

	for(j = 0; j < data_blk_size; j++) {
		i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
}

static int dxl_write_packet(int id, int instruction, int parameter_data_size, ...)
{
	uint8_t frame_to_write[50];
	uint16_t crc;
	uint32_t length=3+parameter_data_size;
	uint8_t arg;

	frame_to_write[0] = 0xFF;

	// A COMPLETER

	PCKT_WR_EN;
	//uart_write( A COMPLETER );
	// attendre que tout soit envoyé
	PCKT_WR_DIS;

	return 0;
}
//==================================================================
#define STATE_START			0
#define STATE_HEADER_1		1
#define STATE_HEADER_2		2
#define STATE_HEADER_3		3
#define STATE_HEADER_4		4
#define STATE_ID			5
#define STATE_LENGTH_LB 	6
#define STATE_LENGTH_HB 	7
#define STATE_INSTRUCTION 	8
#define STATE_PARAM 		9
#define STATE_CRC1 			10
#define STATE_CRC2 			11
#define STATE_FINAL 		12
/* dxl_read_packet(uint8_t *buf)
 *  reads result packet
 *  buf : filled with incoming data
 *         buf[0]  : ID
 *         buf[1]  : Err (0 ==> no Error)
 *         buf[...]: extra return parameters
 *        buf size must be 2+nb of expected parameters
 *  returns the number of values in buf or -1 if an error occurs
 */
static int dxl_read_packet(uint8_t *buf)
{
	typedef int STATE_FSM;
	static  STATE_FSM state = STATE_START;
	int d;
	uint8_t tmp;
	uint16_t length;

	int i=0;

	state = STATE_START;
	i=0;

	while(state!=STATE_FINAL) {
		if((d = uart_getc_with_timeout(USART, TIMEOUT))==-1) {
			return -1;
		} else {
			tmp=(uint8_t)d;
		}
	
		switch(state) {
			case STATE_START:
				if(tmp == 0xFF) {state = STATE_HEADER_1;}
				break;
			case STATE_HEADER_1:
				if( tmp == 0xFF) {state = STATE_HEADER_2;}
				break;
			case STATE_HEADER_2:
				if( tmp == 0xFD) {state = STATE_HEADER_3;}
				break;
			case STATE_HEADER_3:
				if( tmp == 0x00) {state = STATE_ID;}
				break;
			case STATE_ID:
				state = STATE_LENGTH_LB;
				buf[i++] = tmp;
#if DXL_DEBUG==1
				uart_printf(_USART2,"id = %d | ",tmp);	// id
#endif
				break;
			case STATE_LENGTH_LB:
				length = tmp ; state = STATE_LENGTH_HB;
				break;
			case STATE_LENGTH_HB:
				length = (uint16_t)tmp << 8 | length ;
				state = STATE_INSTRUCTION;
#if DXL_DEBUG==1
				uart_printf(_USART2,"length = %d | ",length);
#endif
				length = length - 2;
				break;
			case STATE_INSTRUCTION :
				state = STATE_PARAM;
				length--;
#if DXL_DEBUG==1
				uart_printf(_USART2,"instruction = 0x%x | ",tmp); // iinstruction
#endif
				break;
			case STATE_PARAM:
				if( length > 0 ){
					buf[i] = tmp ;
					length-- ;
#if DXL_DEBUG==1
					uart_printf(_USART2,"param[%d] = 0x%x | ",i,tmp);
#endif
					i++;
					state = STATE_PARAM;
				}
				if (length == 0) {
					state = STATE_CRC1;
				}
				break;
			case STATE_CRC1:
				state = STATE_CRC2;
#if DXL_DEBUG==1
				uart_printf(_USART2,"CRC1 = 0x%x | ",tmp);
#endif
				break;
			case STATE_CRC2:
				state = STATE_FINAL;
#if DXL_DEBUG==1
				uart_printf(_USART2,"CRC2 = 0x%x ",tmp);
#endif
				break;
			/*case STATE_FINAL:
				return i;
				break;*/
			default:
				return -1;
		}
	}
#if DXL_DEBUG==1
	uart_puts(_USART2,"\r\n");
#endif
	return i;
}
//==============================================================
int dxl_init()
{
	io_configure(PCKT_WREN_PORT,PCKT_WREN_PIN,PIN_MODE_OUTPUT|PIN_OPT_OUTPUT_SPEED_FAST,NULL);
	PCKT_WR_DIS;
	
	return uart_init(USART, BAUDRATE, UART_8N1);
}
