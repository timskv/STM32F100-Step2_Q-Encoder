#include "stm32f10x_usart.h"
#include <stm32f10x_rcc.h>
#include <stm32f10x_dma.h>
DMA_InitTypeDef dma;
NVIC_InitTypeDef  NVIC_InitStructure;
int16_t tmp,rx,tx_end;

void send_to_uart(uint8_t data)
{
	//while(!(USART1->SR & USART_SR_TC)); //Ждем пока бит TC в регистре SR станет 1
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART1, USART_IT_TC, ENABLE);
	/*while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	   {
	   }*/
    printf("Test");
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
