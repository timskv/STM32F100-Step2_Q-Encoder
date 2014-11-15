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
#include "enc_v1_func.c"





uint8_t pr;
uint8_t dataBuffer[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

NVIC_InitTypeDef  NVIC_InitStructure1; // interruptions


void Delay(volatile uint32_t nCount);







volatile int main(void)
{
	uint8_t uart_rx_data;

	// init for GPIO (LED)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN , ENABLE); //for irq
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9 ;       // two LED (guess on what pin!!)
	GPIO_Init(GPIOC, &GPIO_InitStructure);




    TIM2_Config();
    TIM3_Config();
    TIM4_Config();
    USART1_Config();
    while(1)
    {
    	Delay(8000000);
    	vUtils_DisplayMsg("Encoder #1 count: ", QE1);
    	vUtils_DisplayMsg("Encoder #2 count: ", QE2);
    }
    while(0)
    {
        /*GPIO_WriteBit(GPIOC,GPIO_Pin_8,Bit_RESET);
        GPIO_WriteBit(GPIOC,GPIO_Pin_9,Bit_SET);
        Delay(8000000);
        GPIO_WriteBit(GPIOC,GPIO_Pin_9,Bit_RESET);
        GPIO_WriteBit(GPIOC,GPIO_Pin_8,Bit_SET);*/
        Delay(8000000);
    	send_to_uart('P');
        send_to_uart('i');
        send_to_uart('z');
        send_to_uart('d');
        send_to_uart('e');
        send_to_uart('t');
        send_to_uart('s');
        send_to_uart(':');
        send_to_uart(')');
        send_to_uart('\n');
        send_to_uart('\r');
//        if (USART1->SR & USART_SR_RXNE)
//        { // ... не пришло ли что-то в UART ?
//            uart_rx_data=USART1->DR; //Считываем то что пришло в переменную...
        /*    switch(uart_rx_data)
            { //И выполняем определённое действие...
                case '0':
                	if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_9))
                	{
                		GPIO_WriteBit(GPIOC,GPIO_Pin_9,Bit_RESET);
                	}
                	else
                	{
                		GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
                	}
                	//GPIOC->ODR^=GPIO_Pin_9;
                	break;
                case '1':
                	GPIOC->ODR^=GPIO_Pin_8;
                	break;
            }*/
      //  }
    }
}

//-------
void Delay(volatile uint32_t nCount)
{
    for (; nCount > 0; nCount--);
}
