#ifndef _TICK_H_
#define _TICK_H_

#ifdef __cplusplus
extern "C" {
#endif 

#include "include/board.h"

typedef void (*OnTick)(void);

/* tick_init
 *   setup systick timer
 */
int tick_init(uint32_t ms, OnTick cb);

/* tick_start
 *   start the systick timer
 */
void tick_start();

/* tick_stop
 *   stop the systick timer
 */
void tick_stop();

/* ticks
 *   get current ticks
 */
uint32_t ticks();

/* tick_delay
 *   delay with systick timer (polling)
 */
int tick_delay(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif
