#include<stm32f10x_rcc.h>

#include<stm32f10x_gpio.h>
#include "stm32f10x_usart.h"
#include <stm32f10x_exti.h>
#include <stm32f10x_dma.h>
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include <misc.h>
#include <stm32f10x_tim.h> //IRQ TIM
#include <stdio.h>
//#include <stdio/printf.c>
#include <stm32f10x_dbgmcu.h>
#include "UtilsOut.h"
#include "usart_func.c"
#include "enc_v2_func.c"
//#include "encoder_zumo.c"

uint8_t dataBuffer[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

NVIC_InitTypeDef  NVIC_InitStructure1; // interruptions
GPIO_InitTypeDef GPIO_InitStructure;
void Delay(volatile uint32_t nCount);
uint16_t e_func(int16_t QEx, uint8_t TickWheel);
uint16_t v_func(int16_t QEx, uint16_t Input_LEN_TO_LEN_PER_TICK);

volatile int main(void)
{
	// init for GPIO (LED)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN , ENABLE); //for irq
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9 ;       // two LED (guess on what pin!!)
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	init_gpio();
	init_timer();
	USART1_Config();
	TIM4_Config();
	while(1)
	{
		Delay(8000000);

		vUtils_DebugNoZero("Encoder1: ");
		vUtils_DisplayDec(TIM2->CNT);
		vUtils_DisplayDec(v_func(TIM2->CNT, LEN_TO_LEN_PER_TICK));
		vUtils_DebugNoZero("Encoder2: ");
		vUtils_DisplayDec(TIM3->CNT);
		vUtils_DisplayDec(v_func(TIM3->CNT, LEN_TO_LEN_PER_TICK));
		TIM2->EGR = TIM_EGR_UG;
		TIM3->EGR = TIM_EGR_UG;


	}

}

//-------
void Delay(volatile uint32_t nCount)
{
    for (; nCount > 0; nCount--);
}

uint16_t e_func(int16_t QEx, uint8_t TickWheel)
{
	return abs(QEx)*360/TickWheel;
}

uint16_t v_func(int16_t QEx, uint16_t Input_LEN_TO_LEN_PER_TICK)
{
	return (uint16_t)QEx*Input_LEN_TO_LEN_PER_TICK;
}
