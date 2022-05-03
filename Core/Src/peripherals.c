/*
 * peripherals.c
 *
 *  Created on: 3 May 2022
 *      Author: nathan
 */

#include "peripherals.h"


void GPIO_Setup(void){
	/*
	 * GPIO Setup
	 *
	 * SPI
	 *
	 * SCLK - PB3  - AF5 - SPI1_SCK
	 * MOSI - PB5  - AF5 - SPI1_MOSI
	 * MISO - PB4  - AF5 - SPI1_MISO
	 * SS   - PA4  - Output   (AF5 - SPI1_NSS)
	 *
	 * CKSC - PC7  - AF2 - TIM3_CH2
	 * SYNC - PA15 - AF1 - TIM2_CH1
	 *
	 *
	 * UART
	 *
	 * Tx   - PD8  - AF7 - USART3_TX
	 * Rx   - PD9  - AF7 - USART3_RX
	 *
	 * Pulse - PD15 - AF2 - TIM4_CH4
	 */

	// GPIO Clock Enable
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;

	// Set Port Mode
	//				PA15 - AF            | PA7 - Analog      | PA6 - Analog      | PA5 - Analog      | PA4 - Output
	GPIOA->MODER |= GPIO_MODER_MODER15_1 | GPIO_MODER_MODER7 | GPIO_MODER_MODER6 | GPIO_MODER_MODER5 | GPIO_MODER_MODER4_0;
	//              PB5 - AF            | PB4 - AF            | PB3 - AF
	GPIOB->MODER |= GPIO_MODER_MODER5_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER3_1;
	//				PC7 - AF
	GPIOC->MODER |= GPIO_MODER_MODER7_1;
	//				PD15 - AF            | PD9 - AF            | PD0 - AF
	GPIOD->MODER |= GPIO_MODER_MODER15_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER8_1;

	// Set Alternate Function
	//               PA4 - AF5
	GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFRL4_Pos);
	//               PA15 - AF1
	GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFRH7_Pos);
	//               PB5 - AF5                  | PB4 - AF5                  | PB3 - AF5
	GPIOB->AFR[0] |= (5 << GPIO_AFRL_AFRL5_Pos) | (5 << GPIO_AFRL_AFRL4_Pos) | (5 << GPIO_AFRL_AFRL3_Pos);
	//               PC7 - AF2
	GPIOC->AFR[0] |= (2 << GPIO_AFRL_AFRL7_Pos);
	//               PD15 - AF2                 | PD9 - AF7                  | PD8 - AF7
	GPIOD->AFR[1] |= (2 << GPIO_AFRH_AFRH7_Pos) | (7 << GPIO_AFRH_AFRH1_Pos) | (7 << GPIO_AFRH_AFRH0_Pos);

	// Set Output Type
	//               PB5 - Open Drain | PB4 - Open Drain | PB3 - Open Drain
//	GPIOB->OTYPER |= GPIO_OTYPER_OT5  | GPIO_OTYPER_OT4  | GPIO_OTYPER_OT3;

	// Set Output Speed
	//                PB5 - Very high speed | PB4 - Very high speed | PB3 - Very high speed
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR5 | GPIO_OSPEEDR_OSPEEDR4 | GPIO_OSPEEDR_OSPEEDR3;

	// Set Pull-up/pull-down
	//              PB5 - Pull-up       | PB4 - Pull-up       | PB3 - Pull-up
//	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR5_0 | GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR3_0;

	// SS high
	GPIOA->BSRR |= (GPIO_BSRR_BS4);

}




// Pulse LED
void TIMER4_Setup(void){
	// Timer4 Clock Enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	// frequency = CLK/(PSC + 1)/(ARR + 1)
	//Duty_Cycle = CCR1/(ARR + 1);
	TIM4->PSC = 9600-1;
	TIM4->ARR = 10000-1;
	TIM4->CCR4 = 5000;

	// Update registers
	TIM4->EGR |= TIM_EGR_UG;

	//             Enable PWM Mode 1            | Enable CCR4 preload register
	TIM4->CCMR2 |= (0x06 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE; //

	//           ARR preload  | enable counter
	TIM4->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;

	//            Output on CH4
	TIM4->CCER |= TIM_CCER_CC4E;

//	TIM4->BDTR |= TIM_BDTR_MOE;
}


// LFO Prescaler
void TIMER5_Setup(void){
	// Timer5  Clock Enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

	// update event at 2khz
	TIM5->PSC = 96-1;
	TIM5->ARR = 500-1;

	//           update as TRGO
	TIM5->CR2 |= (2 << TIM_CR2_MMS_Pos);

	//           enable timer
	TIM5->CR1 |= TIM_CR1_CEN;
}


// Sampling timer
void TIMER6_Setup(void){
	// Timer6  Clock Enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	// f = 32khz
	TIM6->PSC = 2-1;
	TIM6->ARR = 1500-1;

	//           update as TRGO
	TIM6->CR2 |= (2 << TIM_CR2_MMS_Pos);

	//           enable timer
	TIM6->CR1 |= TIM_CR1_CEN;

}

// LFO for effects - count value is index for sin lookup table
void TIMER12_Setup(void){
	// Timer9  Clock Enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;

	TIM12->ARR = 255;

	// TIM5 used as a clock source (2 khz)
	//             timer 10 TRGI          | external clock mode
	TIM12->SMCR |= (1 << TIM_SMCR_TS_Pos) | (7 << TIM_SMCR_SMS_Pos);


	TIM12->PSC = 6;

	//           enable timer
	TIM12->CR1 |= TIM_CR1_CEN;
}

void ADC_Setup(void){
	// ADC  Clock Enable
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	//          /4 prescaler for adc clock
	ADC->CCR |= ADC_CCR_ADCPRE_0;

	//           start adc
	ADC1->CR2 |= ADC_CR2_ADON;

	 //           Hardware trigger | external trigger TIM6 TRGO   | left align    | power on ADC1
	ADC1->CR2 |=  ADC_CR2_EXTEN_0  | (0x0D << ADC_CR2_EXTSEL_Pos) | ADC_CR2_ALIGN;// |  ADC_CR2_ADON;

	//             1 conversions
	ADC1->SQR1 |= ((1 - 1) << ADC_SQR1_L_Pos);
	//            conversion 1: ADC5: PA5
	ADC1->SQR3 |= (5 << ADC_SQR3_SQ1_Pos);

	//             28 cycles ADC5
	ADC1->SMPR1 |= (2 << ADC_SMPR2_SMP5_Pos);

//	//            circular more   | DMA enable
//	ADC1->CFGR |=  |  | ADC_CFGR_DMACFG | ADC_CFGR_DMAEN;

	//           Start ADC Conversion
	ADC1->CR2 |= ADC_CR2_SWSTART;



	// ADC  Clock Enable
//	RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
//
//	ADC2->CR2 |= ADC_CR2_ADON;
//
//	 //           Hardware trigger | external trigger TIM5 TRGO   | left align    | circular DMA | DMA enable
//	ADC2->CR2 |=  ADC_CR2_EXTEN_0  | (0x04 << ADC_CR2_EXTSEL_Pos) | ADC_CR2_ALIGN;// | /*ADC_CR2_DDS  |*/ ADC_CR2_DMA;
//
//	//             2 conversions
//	ADC2->SQR1 |= ((2 - 1) << ADC_SQR1_L_Pos);
//
//	//            conversion 2: ADC7: PA7 | conversion 1: ADC6: PA6
//	ADC2->SQR3 |= (7 << ADC_SQR3_SQ2_Pos) | (6 << ADC_SQR3_SQ1_Pos);
//
//	//             480 cycles ADC7           | 480 cycles ADC6
//	ADC2->SMPR1 |= (7 << ADC_SMPR2_SMP7_Pos) | (7 << ADC_SMPR2_SMP6_Pos);
//
//	//           Start ADC Conversion
//	ADC2->CR2 |= ADC_CR2_SWSTART;
//
//
//	ADC_DMA_Setup();
}


void ADC_DMA_Setup(void){
	// DMA Clock Enable
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;


	// ADC2 - Stream 3 - Channel 1
	DMA2_Stream2->PAR = (long)&(ADC2->DR);
//	DMA2_Stream2->M0AR = (long)(control);
	//                  channel 1                 | medium proir  | mmry inc      | 16 bit transfer  | circular mode | enable
	DMA2_Stream2->CR |= (1 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_0 | DMA_SxCR_MINC | DMA_SxCR_MSIZE_0 | DMA_SxCR_CIRC | DMA_SxCR_EN;
	DMA2_Stream2->NDTR = 2;
}

