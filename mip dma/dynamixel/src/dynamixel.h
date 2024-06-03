/*
 * dxlamixel.h
 *
 *  Created on: Oct 10, 2019
 *      Author: kerhoas
 */

#ifndef _DYNAMIXEL_H_
#define _DYNAMIXEL_H_

#include "include/board.h"

#define DXL_ERR_DEV		-1

#define BROADCAST		0xFE

#define TORQUE_ON 		1
#define TORQUE_OFF 		0
#define LED_ON 			1
#define LED_OFF			0

/* Operation modes */
#define VELOCITY_MODE 	1
#define POSITION_MODE	3
#define EXT_POS_MODE	4
#define PWM_MODE		16

int dxl_init();

/****************************************************************
 * Configuration
 *   get info about firmwareversion and device model
 *   factory reset
 *   get/set baudrate
 *   set id
 *   set operating mode
 ****************************************************************/
int dxl_ping(uint8_t id, uint8_t *idbuf);

uint8_t dxl_get_firmware_version(uint8_t id);
uint8_t dxl_get_model_number(uint8_t id);

/* reset to default factory settings
 *   id = 1
 *   baudrate = 57600 bauds
 */
void dxl_factory_reset(uint8_t id);

/* set / get baudrate
 *  id : current id
 *  val: new baudrate code:
 *			7			4.5M [bps]
 *			6			4M [bps]
 *			5			3M [bps]
 *			4			2M [bps]
 *			3			1M [bps]
 *			2			115,200 [bps]
 *			1(Default)	57,600 [bps]
 *			0			9,600 [bps]
 */
void dxl_set_baudrate(uint8_t id, uint8_t val);
uint8_t dxl_get_baudrate(uint8_t id);

/* allow write access to the EEPROM: 0=OFF / 1=ON */
void dxl_torque(uint8_t id, uint8_t val);

/* set new id
 *  id : current id
 *  val: new id (0..252)
 */
void dxl_set_id(uint8_t id, uint8_t new_id);

/* change operating mode
 *  id  : motor id
 *  val : one of VELOCITY_MODE/POSITION_MODE/EXT_POS_MODE/PWM_MODE
 */
void dxl_set_operating_mode(uint8_t id, uint8_t val);

/****************************************************************
 * Control / status
 *   switch LED on/off
 *   define goal position / velocity depending on operating mode
 *   get current position / velocity
 ****************************************************************/
/* switch the LED on/off */
void dxl_LED(uint8_t id, uint8_t val );

/* set the velocity order / get the current velocity when operating
 * mode is VELOCITY_MODE
 *  id : motor id
 *  val: velocity order (-265 .. 265)
 */
void dxl_set_goal_velocity(uint8_t id, int val);
int32_t dxl_get_cur_velocity(uint8_t id);

/* set the position order / get the current position when operating
 * mode is POSITION_MODE.
 *  id : motor id
 *  val: position order (0 .. 4095)
 */
void dxl_set_goal_position(uint8_t id, int val);
uint32_t dxl_get_cur_position(uint8_t id);

#endif /* _DYNAMIXEL_H_ */
