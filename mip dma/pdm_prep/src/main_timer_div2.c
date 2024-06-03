#include "include/board.h"
#include "lib/io.h"


int main(void)
{
	
	/* Configure Timer as a mere frequency divider
	 *               +-------+
	 *  input ch1 -->|  F/2  |--> output chan2
	 *     PB4       +-------+       PB5
	 */
	// Configure GPIOs AF2: PB4 --> TIM3 chan 1, PB5 --> TIM3 chan 2 
	//io_configure(/* A COMPLETER */);

	// Enable Timer 3 clocking
	//_RCC->APB1ENR |= A COMPLETER;

	_TIM3->PSC = 0;
	_TIM3->ARR = 1;
	
	/* configure
	   - input  chan 1: IC1 -> TI1, no filter
	   - output chan 2: PWM1, output is set when CNT<CCR2
	 */
	//_TIM3->CCR[1] = A COMPLETER
	//_TIM3->CCMR[0] = A COMPLETER
	
	/* CCER (capture/compare enable reg) 
	 *   select trigger from rising edge from TI1
	 *   PB5  PB4
	 *   00x1|00x0
	 * allow | rising edge,
	 * ouput | don't capture CNT
	 */
	//_TIM3->CCER = A COMPLETER
	
	// external clock mode 1 from edge (SMS=111) of TI1 (TS=101)
	//_TIM3->SMCR = A COMPLETER
	
	/* reset and start */
	_TIM3->EGR = 1;
	_TIM3->CR1 = 1;

	while (1) {
	
	}
	
	return 0;
}
