#include "lib/io.h"
#include "leds.h"

/* led RGB :
 *   LED_RED   --> PB4
 *   LED_GREEN --> PC7
 *   LED_BLUE  --> PA9
 */
#define LED_RED_MASK	(1 << 4)
#define LED_GREEN_MASK	(1 << 7)
#define LED_BLUE_MASK	(1 << 9)
#define LED_RED_OFST    4
#define LED_GREEN_OFST  7
#define LED_BLUE_OFST   9
 
#define IO_CFG_LEDS (    < A COMPLETER >    )


uint32_t leds_init(void)
{
    //          < A COMPLETER >
    io_configure(_GPIOB, LED_RED_MASK, PIN_MODE_OUTPUT, NULL);
    io_configure(_GPIOC, LED_GREEN_MASK, PIN_MODE_OUTPUT, NULL);
    io_configure(_GPIOA, LED_BLUE_MASK, PIN_MODE_OUTPUT, NULL);
    return 0;
}

void leds(uint16_t val)
{
    //          < A COMPLETER >
    red_led(val & 0x4); //décalage de 2 bits pour récup 1 et non 100
    green_led(val & 0x2);//pareil on récup 1 et non 10
    blue_led(val & 0x1);
}

void red_led(uint32_t on)
{
    //          < A COMPLETER >
    if (on) {
        io_clear(_GPIOB, LED_RED_MASK);
    }
    else {
        io_set(_GPIOB, LED_RED_MASK);
    }
}

void green_led(uint32_t on)
{
    //          < A COMPLETER >
    if (on) {
        io_clear(_GPIOC, LED_GREEN_MASK);
    }
    else {
        io_set(_GPIOC, LED_GREEN_MASK);
    }
}

void blue_led(uint32_t on)
{
    //          < A COMPLETER >
    if (on) {
        io_clear(_GPIOA, LED_BLUE_MASK); //io_set(_GPIOA, (1 << 9));
    }
    else {
        io_set(_GPIOA, LED_BLUE_MASK);
    }
}
