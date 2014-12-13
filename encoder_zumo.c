#include <stm32f10x_tim.h> //IRQ TIM
#include<stm32f10x_gpio.h>

#define DIAMETER_WHEEL_MM 20 //диаметр колеса
#define TICK_FOR_WHEEL 10 //щелчков на один оборот
#define TIM_OVERRUN 255 //размерность счетчика энкодера
#define TIMER_WAIT_MS 1000 // раз во сколько вемени отсчитываем энкодеры.

#define LED_GREEN	B, 7, HIGH, MODE_OUTPUT_PUSH_PULL, SPEED_400KHZ, AF_NO


NVIC_InitTypeDef NVIC_Int;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
GPIO_InitTypeDef GPIO_InitStructure1;
int16_t QE1 = 0, QE2 = 0;
int16_t PreviousQE1 = 0, PreviousQE2 = 0;

uint16_t Enc1_Interrupted = 0;
uint32_t Enc1_TotalCount = 0;

void mcu_tim2_init(void)
{
	RCC->APB1ENR = RCC_APB1ENR_TIM2EN; //тактирование
	TIM2->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0; //Настраиваем второй мультиплексор (для первого и второго входа):
	TIM2->CCER = TIM_CCER_CC1P | TIM_CCER_CC2P; //С детектора фронтов возьмем не инверсный, т.е. активный уровень высокий (для первого и второго входа):
	TIM2->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; //Всё, можно разрешать работу счетчика:
	TIM2->ARR = TIM_OVERRUN;
	TIM2->CR1 = TIM_CR1_CEN;
	PIN_CONFIGURATION(LED_GREEN);
}

void TIM2_Config(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 ;       // two LED (guess on what pin!!)
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	NVIC_Int.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_Int.NVIC_IRQChannelSubPriority = 2;
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

	NVIC_Int.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_Int.NVIC_IRQChannelSubPriority = 2;
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
void TIM4_Config(void)
{
	RCC->APB1ENR += RCC_APB1ENR_TIM4EN; //аккуратно!!
	TIM4->PSC = 24000-1;
	TIM4->ARR = TIMER_WAIT_MS;
	TIM4->DIER = TIM_DIER_UIE;
	TIM4->CR1 = TIM_CR1_CEN;
	//NVIC_EnableIRQ(TIM4_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
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
	//uint16_t cnt = TIM4->CNT;
	QE1+=TIM2->CNT;
	QE2+=TIM3->CNT;
	/*TIM2->CR1 =(TIM2->CR1) & ~(TIM_CR1_CEN);
	TIM3->CR1 =(TIM3->CR1) & ~ (TIM_CR1_CEN);

	TIM2->CR1 =TIM_CR1_CEN;
	TIM3->CR1 = TIM_CR1_CEN;
	*/
	//send_to_uart('S');
	//send_to_uart('G');
	/*TIM2->ARR = 0;
	TIM2->ARR= 1000;
	TIM3->ARR = 0;
	TIM3->ARR= 1000;*/
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

	//mcu_tim2_init();
	//mcu_tim3_init();


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
