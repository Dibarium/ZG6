#include <stdlib.h>

#include "io.h"

static OnIO io_cb[16]={
	NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL
};

void EXTI0_IRQHandler() {
	if (io_cb[0]) (io_cb[0])();
	
	_EXTI->PR = 1<<0;
}

void EXTI1_IRQHandler() {
	if (io_cb[1]) (io_cb[1])();
	
	_EXTI->PR = 1<<1;
}

void EXTI2_IRQHandler() {
	if (io_cb[2]) (io_cb[2])();
	
	_EXTI->PR = 1<<2;
}

void EXTI3_IRQHandler() {
	if (io_cb[3]) (io_cb[3])();
	
	_EXTI->PR = 1<<3;
}

void EXTI4_IRQHandler() {
	if (io_cb[4]) (io_cb[4])();
	
	_EXTI->PR = 1<<4;
}

void EXTI9_5_IRQHandler() {
	if (_EXTI->PR & (1<<5)) {
		if (io_cb[5]) (io_cb[5])();
		_EXTI->PR = 1<<5;
	} else if (_EXTI->PR & (1<<6)) {
		if (io_cb[6]) (io_cb[6])();
		_EXTI->PR = 1<<6;
	} else if (_EXTI->PR & (1<<7)) {
		if (io_cb[7]) (io_cb[7])();
		_EXTI->PR = 1<<7;
	} else if (_EXTI->PR & (1<<8)) {
		if (io_cb[8]) (io_cb[8])();
		_EXTI->PR = 1<<8;
	} else if (_EXTI->PR & (1<<9)) {
		if (io_cb[9]) (io_cb[9])();
		_EXTI->PR = 1<<9;
	}
}

void EXTI15_10_IRQHandler() {
	if (_EXTI->PR & (1<<10)) {
		if (io_cb[10]) (io_cb[10])();
		_EXTI->PR = 1<<10;
	} else if (_EXTI->PR & (1<<11)) {
		if (io_cb[11]) (io_cb[11])();
		_EXTI->PR = 1<<11;
	} else if (_EXTI->PR & (1<<12)) {
		if (io_cb[12]) (io_cb[12])();
		_EXTI->PR = 1<<12;
	} else if (_EXTI->PR & (1<<13)) {
		if (io_cb[13]) (io_cb[13])();
		_EXTI->PR = 1<<13;
	} else if (_EXTI->PR & (1<<14)) {
		if (io_cb[14]) (io_cb[14])();
		_EXTI->PR = 1<<14;
	} else if (_EXTI->PR & (1<<15)) {
		if (io_cb[15]) (io_cb[15])();
		_EXTI->PR = 1<<15;
	}

}

/* Definitions for EXTI configuration */
#define SYSCFG_EXTI_PA_MASK		0
#define SYSCFG_EXTI_PB_MASK		1
#define SYSCFG_EXTI_PC_MASK		2
#define SYSCFG_EXTI_PD_MASK		3
#define SYSCFG_EXTI_PE_MASK		4
#define SYSCFG_EXTI_PH_MASK		7

int io_configure(GPIO_t *gpio, uint16_t pin_mask, uint32_t pin_cfg, OnIO cb)
{
	// enable GPIOx subsystem clocking
	if (gpio == _GPIOA) _RCC->AHB1ENR |= 1;
	else if (gpio == _GPIOB) _RCC->AHB1ENR |= 1<<1;
	else if (gpio == _GPIOC) _RCC->AHB1ENR |= 1<<2;
	else if (gpio == _GPIOD) _RCC->AHB1ENR |= 1<<3;
	else if (gpio == _GPIOE) _RCC->AHB1ENR |= 1<<4;
	else if (gpio == _GPIOH) _RCC->AHB1ENR |= 1<<7;
	
	uint32_t af = pin_cfg & 0xF;
	uint32_t ospeed = (pin_cfg >> 4) & 3;
	uint32_t otype = (pin_cfg >> 6) & 1;
	uint32_t pupd = (pin_cfg >> 8) & 3;
	uint32_t pin_mode = (pin_cfg & (3<<16));
	
	uint32_t moder = gpio->MODER;
	uint32_t pupdr = gpio->PUPDR;
	uint32_t otyper = gpio->OTYPER;
	uint32_t ospeedr = gpio->OSPEEDR;
	
	
	uint16_t bit_mask = 1;
	for (int i=0; i<16; i++) {
		if (pin_mask & bit_mask) {
			moder   = (moder & ~(3 << (i<<1))) | ((pin_mode>>16) << (i<<1));
			pupdr   = (pupdr & ~(3 << (i<<1))) | (pupd << (i<<1));
			otyper  = (otyper & ~(1 << i)) | (otype << i);
			ospeedr = (ospeedr & ~(3 << (i<<1))) | (ospeed << (i<<1));
		}
		bit_mask = bit_mask<<1;
	}
	gpio->MODER = moder;
	gpio->PUPDR = pupdr;
	gpio->OTYPER = otyper;
	gpio->OSPEEDR = ospeedr;

	if (pin_mode == PIN_MODE_ALTFUNC) {
		uint32_t afrl = gpio->AFR[0];
		uint32_t afrh = gpio->AFR[1];
		
		bit_mask = 1;
		for (int i=0; i<8; i++) {
			if (pin_mask & bit_mask) {
				afrl = (afrl & ~(0xF << (i<<2))) | (af << (i<<2));
			}
			bit_mask = bit_mask<<1;
		}
		gpio->AFR[0] = afrl;
		
		bit_mask = 0x100;
		for (int i=0; i<8; i++) {
			if (pin_mask & bit_mask) {
				afrh = (afrh & ~(0xF << (i<<2))) | (af << (i<<2));
			}
			bit_mask = bit_mask<<1;
		}
		gpio->AFR[1] = afrh;
	}
		
	if (!cb) return 0;				// GPIO or Alt only Function
	
	if (pin_mode != PIN_MODE_INPUT) return -1;

	// ************* Input GPIO + External IRQ *************
	
	uint32_t port_mask = 0;
	
	if (gpio == _GPIOA) port_mask = SYSCFG_EXTI_PA_MASK;
	else if (gpio == _GPIOB) port_mask = SYSCFG_EXTI_PB_MASK;
	else if (gpio == _GPIOC) port_mask = SYSCFG_EXTI_PC_MASK;
	else if (gpio == _GPIOD) port_mask = SYSCFG_EXTI_PD_MASK;
	else if (gpio == _GPIOE) port_mask = SYSCFG_EXTI_PE_MASK;
	else if (gpio == _GPIOH) port_mask = SYSCFG_EXTI_PH_MASK;
	
	bit_mask = 0x1;
	
	for (int i=0; i<16; i++) {
		if (pin_mask & bit_mask) {
			// enable clock for SYSCFG, no need for EXTI (interface not clocked)
			_RCC->APB2ENR = _RCC->APB2ENR | (1<<14);
			
			// configure pin Px_i (4 pin config per EXTICR[] register, 4 bits per pin)
			//   use port Px and bind Px_i --> EXTIi
			// i>>2 = i/4 ; i & 0x3 = i%4
			_SYSCFG->EXTICR[i>>2] = (_SYSCFG->EXTICR[i>>2] & ~(0x0000000F << ((i & 3)<<2))) | (port_mask << ((i & 3)<<2));
			
			// allow pin EXTIi to send an IRQ
			_EXTI->IMR = _EXTI->IMR | bit_mask;
			//   not a wakeup event
			_EXTI->EMR = _EXTI->EMR & (~bit_mask);
			
			// Configure pin event IRQ on rising (RTSR)/falling (FTSR) edge (rising only here)
			if (pin_cfg & PIN_OPT_IRQ_EDGE_RISE) {
				_EXTI->RTSR = _EXTI->RTSR | bit_mask;
			} else {
				_EXTI->RTSR = _EXTI->RTSR & (~bit_mask);
			}
			
			if (pin_cfg & PIN_OPT_IRQ_EDGE_FALL) {
				_EXTI->FTSR = _EXTI->FTSR | bit_mask;
			} else {
				_EXTI->FTSR = _EXTI->FTSR & (~bit_mask);
			}
			
			io_cb[i] = cb;
			
			// reset any pending IRQ on PC13
			_EXTI->PR = bit_mask;
			
			// Setup NVIC
			switch (i) {
			case 0:
				NVIC_SetPriority(EXTI0_IRQn, EXTI0_IRQ_PRIORITY);
				NVIC_EnableIRQ(EXTI0_IRQn);
				break;
			case 1:
				NVIC_SetPriority(EXTI1_IRQn, EXTI1_IRQ_PRIORITY);
				NVIC_EnableIRQ(EXTI1_IRQn);
				break;
			case 2:
				NVIC_SetPriority(EXTI2_IRQn, EXTI2_IRQ_PRIORITY);
				NVIC_EnableIRQ(EXTI2_IRQn);
				break;
			case 3:
				NVIC_SetPriority(EXTI3_IRQn, EXTI3_IRQ_PRIORITY);
				NVIC_EnableIRQ(EXTI3_IRQn);
				break;
			case 4:
				NVIC_SetPriority(EXTI4_IRQn, EXTI4_IRQ_PRIORITY);
				NVIC_EnableIRQ(EXTI4_IRQn);
				break;
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
				NVIC_SetPriority(EXTI9_5_IRQn, EXTI9_5_IRQ_PRIORITY);
				NVIC_EnableIRQ(EXTI9_5_IRQn);
				break;
			case 10:
			case 11:
			case 12:
			case 13:
			case 14:
			case 15:
				NVIC_SetPriority(EXTI15_10_IRQn, EXTI15_10_IRQ_PRIORITY);
				NVIC_EnableIRQ(EXTI15_10_IRQn);
				break;
			default:
				return 0;
			}
		}
		bit_mask = bit_mask<<1;
	}
	
	return 0;
}


uint32_t io_read(GPIO_t *gpio, uint16_t mask)
{
	return gpio->IDR & (uint32_t)mask;
}

void io_write(GPIO_t *gpio, uint16_t val, uint16_t mask)
{
	gpio->BSRR = ((uint32_t)val & (uint32_t)mask) | ((~(uint32_t)val & (uint32_t)mask) << 16);
}

void io_write_n(GPIO_t *gpio, uint16_t val, uint16_t mask)
{
	gpio->BSRR = (~(uint32_t)val & (uint32_t)mask) | (((uint32_t)val & (uint32_t)mask) << 16);
}

void io_set(GPIO_t *gpio, uint16_t mask)
{
	gpio->BSRR = (uint32_t)mask;
}

void io_clear(GPIO_t *gpio, uint16_t mask)
{
	gpio->BSRR = (uint32_t)mask<<16;
}
