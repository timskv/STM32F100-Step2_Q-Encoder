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

#define DIAMETER_WHEEL_MM 20 //диаметр колеса
#define TICK_FOR_WHEEL 10 //щелчков на один оборот
#define TIM_OVERRUN 255 //размерность счетчика энкодера
#define TIMER_WAIT_MS 1000 // раз во сколько вемени отсчитываем энкодеры.

int16_t tmp,rx,tx_end;

uint16_t Enc1_Interrupted = 0;
uint32_t Enc1_TotalCount = 0;

uint8_t pr;
DMA_InitTypeDef dma;uint8_t dataBuffer[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
GPIO_InitTypeDef GPIO_InitStructure, GPIO_InitStructure1;
NVIC_InitTypeDef  NVIC_InitStructure, NVIC_InitStructure1; // interruptions
NVIC_InitTypeDef NVIC_Int;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
void Delay(volatile uint32_t nCount);
void send_to_uart(uint8_t data)
{
	//while(!(USART1->SR & USART_SR_TC)); //Ждем пока бит TC в регистре SR станет 1
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
	}
	USART1->DR=data; //Отсылаем байт через UART
}

//Функция отправляет строку в UART
void send_str(char * string) {
	uint8_t i=0;
	while(string[i]) {
		send_to_uart(string[i]);
		i++;
	}
	send_to_uart('\r');
	send_to_uart('\n');
}



void mcu_tim2_init(void)
{
	RCC->APB1ENR = RCC_APB1ENR_TIM2EN; //тактирование
	TIM2->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0; //Настраиваем второй мультиплексор (для первого и второго входа):
	TIM2->CCER = TIM_CCER_CC1P | TIM_CCER_CC2P; //С детектора фронтов возьмем не инверсный, т.е. активный уровень высокий (для первого и второго входа):
	TIM2->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; //Всё, можно разрешать работу счетчика:
	TIM2->ARR = TIM_OVERRUN;
	TIM2->CR1 = TIM_CR1_CEN;
}

void TIM4_Config(void)
{
	RCC->APB1ENR += RCC_APB1ENR_TIM4EN; //аккуратно!!
	TIM4->PSC = 24000-1;
	TIM4->ARR = TIMER_WAIT_MS;
	TIM4->DIER = TIM_DIER_UIE;
	TIM4->CR1 = TIM_CR1_CEN;
	//NVIC_EnableIRQ(TIM4_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

void mcu_tim3_init(void)
{
	RCC->APB1ENR = RCC_APB1ENR_TIM3EN; //тактирование
	TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0; //Настраиваем второй мультиплексор (для первого и второго входа):
	TIM3->CCER = TIM_CCER_CC1P | TIM_CCER_CC2P; //С детектора фронтов возьмем не инверсный, т.е. активный уровень высокий (для первого и второго входа):
	TIM3->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; //Всё, можно разрешать работу счетчика:
	TIM3->ARR = TIM_OVERRUN;
	TIM3->CR1 = TIM_CR1_CEN;
}

uint32_t Enc1_GetCount(void)
{
	uint32_t count = 0;
	uint32_t tmpTotalCount = 0;
	uint32_t tmpEncCount = 0;

//	do {
			Enc1_Interrupted = 1;
			tmpTotalCount = Enc1_TotalCount;
			tmpEncCount = TIM2->CNT;
	//} while ( Enc1_Interrupted == 0 );

	count = tmpTotalCount + tmpEncCount;

	return count;
}

void TIM2_IRQHandler(void)
{
	uint16_t cnt = TIM2->CNT;
	GPIOC->ODR^=GPIO_Pin_8;

	//send_to_uart('T');
	 /* if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	  {
	    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	  }*/
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

	if ( cnt < 0x7FFF ) {
		Enc1_TotalCount += (uint32_t)0x10000;
	} else {
		Enc1_TotalCount -= (uint32_t)0x10000;
	}
	Enc1_Interrupted = 0;
	//send_to_uart('S');
	//USART1->DR = (cnt & (uint16_t)0x01FF);
	//USART_SendData(USART1, 0x01);
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(DMA1_Channel4, ENABLE);
}

void TIM3_IRQHandler(void)
{
	uint16_t cnt = TIM3->CNT;
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

void TIM4_IRQHandler(void)
{
	//vUtils_Debug("FF");
	uint16_t cnt = TIM4->CNT;
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	send_to_uart('S');
	//send_to_uart('G');
}

void VirtualEncoderInc()
{
	GPIO_WriteBit(GPIOC,GPIO_Pin_9,Bit_RESET);
	GPIO_WriteBit(GPIOC,GPIO_Pin_8,Bit_RESET);
	Delay(800000);
	GPIO_WriteBit(GPIOC,GPIO_Pin_9,Bit_SET);
	Delay(800000);
	GPIO_WriteBit(GPIOC,GPIO_Pin_8,Bit_SET);
	Delay(800000);
	GPIO_WriteBit(GPIOC,GPIO_Pin_9, Bit_RESET);
	Delay(800000);
	GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);
	Delay(800000);
}

void USART1_Config(void)
{
	GPIO_InitTypeDef PORTA_init_struct;
	uint8_t DMA_Test=0xFE;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_StructInit(&dma);
	dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
	dma.DMA_MemoryBaseAddr = (uint32_t)&(TIM2->CNT);
	dma.DMA_DIR = DMA_DIR_PeripheralDST;
	dma.DMA_BufferSize = 1;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Disable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_Init(DMA1_Channel4, &dma);


	// Включаем тактирование порта А и USART1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
	GPIO_InitTypeDef gpio_port;

	// Настраиваем ногу TxD (PA9) как выход push-pull c альтернативной функцией
	PORTA_init_struct.GPIO_Pin = GPIO_Pin_9;
	PORTA_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	PORTA_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &PORTA_init_struct);

	// Настраиваем ногу PA10 как вход UARTа (RxD)
	gpio_port.GPIO_Pin   = GPIO_Pin_10;
	gpio_port.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio_port);

	/*//Настраиваем UART
	USART1->BRR=0x9c4; //BaudRate 9600
	USART1->CR1 |= USART_CR1_UE; //Разрешаем работу USART1
	USART1->CR1 |= USART_CR1_TE; //Включаем передатчик*/
	USART_InitTypeDef uart_struct;
	uart_struct.USART_BaudRate            = 9600;
	uart_struct.USART_WordLength          = USART_WordLength_8b;
	uart_struct.USART_StopBits            = USART_StopBits_1;
	uart_struct.USART_Parity              = USART_Parity_No ;
	uart_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uart_struct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

	//Инициализируем UART
	USART_Init(USART1, &uart_struct);
	//Включаем UART
	USART_Cmd(USART1, ENABLE);





    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
   // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
   // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART1, USART_IT_TC, ENABLE);
	/*while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	   {
	   }*/
    printf("Test");
}

void TIM2_Config(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 ;       // two LED (guess on what pin!!)
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	NVIC_Int.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Int.NVIC_IRQChannelSubPriority = 0;
	NVIC_Int.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Int.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_Init(&NVIC_Int);

	TIM_TimeBaseStructure.TIM_Period = 200;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

}

void TIM3_Config(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 ;       // two LED (guess on what pin!!)
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	NVIC_Int.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Int.NVIC_IRQChannelSubPriority = 0;
	NVIC_Int.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Int.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_Init(&NVIC_Int);

	TIM_TimeBaseStructure.TIM_Period = 200;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_Cmd(TIM3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

}

void EINT_Config(void)
{

	GPIO_StructInit(&GPIO_InitStructure1);
	GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure1);


	//SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_StructInit(&EXTI_InitStructure);
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure1;
	NVIC_InitStructure1.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure1.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure1.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure1.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure1);
}

void EXTI0_IRQHandler(void)
{
	 if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	    {
	        //Handle the interrupt
		 GPIOC->ODR^=GPIO_Pin_8;
	        EXTI_ClearITPendingBit(EXTI_Line0);
	    }
}

void USART1_IRQHandler(void)
{
	//send_to_uart('J');
	//Receive Data register not empty interrupt
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		rx=1;
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        tmp=USART_ReceiveData (USART1);

        switch(tmp)
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
            case '2':
            	VirtualEncoderInc();
            	break;
            case '4':
            	//USART_SendData(USART1, Enc1_GetCount());
            	USART_SendData(USART1, TIM2->CNT);
            	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
            	   {
            	   }
            	USART_SendData(USART1, TIM3->CNT);
            	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
            	   {
            	   }
            	vUtils_DisplayMsg("test", TIM3->CNT);
            	//send_to_uart('F');
            	break;

        }

	}
    //Transmission complete interrupt
	/*if(USART1->SR == USART_SR_TC)
	{
		USART1->SR -= USART_SR_TC;
	}*/
	if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_TC);
		tx_end=1;
	}
}


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
    	;
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
