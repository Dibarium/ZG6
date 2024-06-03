/****************************************************************************
 * IRQ Priority
 ****************************************************************************/
#define SYSTICK_PRIORITY		0
#define SVC_PRIORITY			0

#define EXTI0_IRQ_PRIORITY		8
#define EXTI1_IRQ_PRIORITY		8
#define EXTI2_IRQ_PRIORITY		8
#define EXTI3_IRQ_PRIORITY		8
#define EXTI4_IRQ_PRIORITY		8
#define EXTI9_5_IRQ_PRIORITY	8
#define EXTI15_10_IRQ_PRIORITY	8

#define TIM2_IRQ_PRIORITY		4
#define TIM3_IRQ_PRIORITY		4
#define TIM4_IRQ_PRIORITY		4
#define TIM5_IRQ_PRIORITY		4

#define	USART1_IRQ_PRIORITY		3
#define	USART2_IRQ_PRIORITY		3
#define	USART6_IRQ_PRIORITY		3

#define DMA1_STREAM0_PRIORITY	2
#define DMA1_STREAM1_PRIORITY	2
#define DMA1_STREAM2_PRIORITY	2
#define DMA1_STREAM3_PRIORITY	6
#define DMA1_STREAM4_PRIORITY	2
#define DMA1_STREAM5_PRIORITY	2
#define DMA1_STREAM6_PRIORITY	2
#define DMA1_STREAM7_PRIORITY	2

#define DMA2_STREAM0_PRIORITY	2
#define DMA2_STREAM1_PRIORITY	2
#define DMA2_STREAM2_PRIORITY	2
#define DMA2_STREAM3_PRIORITY	2
#define DMA2_STREAM4_PRIORITY	2
#define DMA2_STREAM5_PRIORITY	2
#define DMA2_STREAM6_PRIORITY	2
#define DMA2_STREAM7_PRIORITY	2

/****************************************************************************
 * GPIO Pin and use configuration 
 ****************************************************************************/

/****************************************************************************
 * TIMER Pin and use configuration:
 ****************************************************************************/
#define USE_TIM3
#define TIM3_GPIO_PORT		_GPIOB
#define TIM3_GPIO_PINS		PIN_4|PIN_5
#define TIM3_GPIO_CONFIG	PIN_MODE_ALTFUNC | PIN_OPT_AF2 | PIN_OPT_OUTPUT_SPEED_HIGH

/****************************************************************************
 * USART Pin and use configuration:
 ****************************************************************************/

//#define USE_USART1
//#define USART1_GPIO_PORT
//#define USART1_GPIO_PINS
//#define USART1_GPIO_CONFIG

// USART2: Tx --> PA2, Rx --> PA3
#define USE_USART2
#define USART2_GPIO_PORT	_GPIOA
#define USART2_GPIO_PINS	PIN_2|PIN_3
#define USART2_GPIO_CONFIG	PIN_MODE_ALTFUNC | PIN_OPT_AF7

//#define USE_USART6
//#define USART6_GPIO_PORT
//#define USART6_GPIO_PINS
//#define USART6_GPIO_CONFIG

/****************************************************************************
 *  I2C Pin and use configuration:
 ****************************************************************************/

// I2C1 : SCL --> PB8, SDA --> PB9
//#define USE_I2C1
//#define I2C1_GPIO_PORT		_GPIOB
//#define I2C1_GPIO_PINS		PIN_8|PIN_9
//#define I2C1_GPIO_CONFIG	PIN_MODE_ALTFUNC | PIN_OPT_OUTPUT_OPENDRAIN | PIN_OPT_AF4

//#define USE_I2C2
//#define I2C2_GPIO_PORT
//#define I2C2_GPIO_PINS
//#define I2C2_GPIO_CONFIG

//#define USE_I2C3
//#define I2C3_GPIO_PORT
//#define I2C3_GPIO_PINS
//#define I2C3_GPIO_CONFIG

/****************************************************************************
 *  SPI pin and use configuration:
 ****************************************************************************/

// SPI1 pins : LCD_SCK   --> D13 --> PA5 : SPI1_SCK
//             LCD_MOSI  --> D11 --> PA7 : SPI1_MOSI
//#define USE_SPI1
//#define SPI1_GPIO_PORT		_GPIOA
//#define SPI1_GPIO_PINS		PIN_5|PIN_7
//#define SPI1_GPIO_CONFIG	PIN_MODE_ALTFUNC|PIN_OPT_OUTPUT_SPEED_MEDIUM|PIN_OPT_AF5

// SPI2/I2S2 pins: PB13 --> I2S2_CK, PB15 --> I2S2_MOSI
//#define USE_SPI2
//#define SPI2_GPIO_PORT			_GPIOB
//#define SPI2_GPIO_PINS			PIN_13|PIN_15
//#define SPI2_GPIO_CONFIG		PIN_MODE_ALTFUNC|PIN_OPT_AF5

//#define USE_SPI3
//#define SPI3_GPIO_PORT
//#define SPI3_GPIO_PINS			
//#define I2C3_GPIO_CONFIG

//#define USE_SPI4
//#define SPI4_GPIO_PORT
//#define SPI4_GPIO_PINS			
//#define I2C4_GPIO_CONFIG

//#define USE_SPI5
//#define SPI5_GPIO_PORT
//#define SPI5_GPIO_PINS			
//#define I2C5_GPIO_CONFIG

/****************************************************************************
 *  ADC pin and use configuration:
 ****************************************************************************/

// ADC1: ADC1_0 --> PA0, ADC1_1 -->PA1
//#define USE_ADC1
//#define ADC1_GPIO_PORT		_GPIOA
//#define ADC1_GPIO_PINS		PIN_0|PIN_1
//#define ADC1_GPIO_CONFIG	PIN_MODE_ANALOG
