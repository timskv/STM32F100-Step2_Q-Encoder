#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>
#include <misc.h>
#include <math.h>
#define DIAMETER_WHEEL_MM 20 //диаметр колеса
#define TICK_FOR_WHEEL 10 //щелчков на один оборот

//begin new defines
#define TIM_TICK_MS 500
#define LEN_TO_LEN_PER_TICK 1
uint16_t ENC1_TICK = 0;
uint16_t ENC2_TICK = 0;
//end new defines

GPIO_InitTypeDef GPIO_InitStructure1;
NVIC_InitTypeDef NVIC_Int;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
typedef enum { FORWARD, BACKWARD } Direction;

double pi = acos(-1.0);
volatile uint8_t capture_is_first_t3 = 1, capture_is_ready_t3 = 0;
volatile uint8_t capture_is_first_t2 = 1, capture_is_ready_t2 = 0;
volatile Direction captured_direction_t2 = FORWARD;
volatile Direction captured_direction_t3 = FORWARD;

uint16_t QE1_F_Inc = 0, QE2_F_Inc = 0;
uint16_t QE1_B_Inc = 0, QE2_B_Inc = 0;
int16_t QE1=0, QE2=0;
uint8_t Tick_For_Wheel = TICK_FOR_WHEEL;
void init_gpio(void);
void init_timer(void);

void init_gpio(void)
{
//  GPIO_InitTypeDef gpio_cfg;
//  GPIO_StructInit(&gpio_cfg);
  uint32_t test;
  /* РљР°РЅР°Р»С‹ 1 Рё 2 С‚Р°Р№РјРµСЂР° TIM3 - РЅР° РІС…РѕРґ, РїРѕРґС‚СЏРЅСѓС‚СЊ Рє РїРёС‚Р°РЅРёСЋ */
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
//  gpio_cfg.GPIO_Mode = GPIO_Mode_IPU;
//  gpio_cfg.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIOA->BSRR|= GPIO_BSRR_BS6|GPIO_BSRR_BS7; //not work without it
  GPIOA->CRL |= GPIO_CRL_CNF6_1;
  GPIOA->CRL |= GPIO_CRL_CNF7_1;
  GPIOA->CRL &= ~GPIO_CRL_CNF6_0;
  GPIOA->CRL &= ~GPIO_CRL_CNF7_0;


//  gpio_cfg.GPIO_Mode = GPIO_Mode_IPU;
//  gpio_cfg.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;

  GPIOA->BSRR|= GPIO_BSRR_BS0|GPIO_BSRR_BS1; //not work without it
  GPIOA->CRL |= GPIO_CRL_CNF0_1;
  GPIOA->CRL |= GPIO_CRL_CNF1_1;
  GPIOA->CRL &= ~GPIO_CRL_CNF0_0;
  GPIOA->CRL &= ~GPIO_CRL_CNF1_0;

  //GPIO_Init(GPIOA, &gpio_cfg);
  //PIN_CONFIGURATION(LED_GREEN);
}

void init_timer(void)
{


  RCC->APB1ENR = RCC_APB1ENR_TIM2EN; //тактирование
 // 	TIM2->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0; //Настраиваем второй мультиплексор (для первого и второго входа):
 // 	TIM2->CCER = TIM_CCER_CC1P | TIM_CCER_CC2P; //С детектора фронтов возьмем не инверсный, т.е. активный уровень высокий (для первого и второго входа):
 // 	TIM2->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; //Всё, можно разрешать работу счетчика:
  	TIM2->ARR = 200;
 // 	TIM2->CR1 = TIM_CR1_CEN;

  	RCC->APB1ENR |=  RCC_APB1ENR_TIM3EN; //тактирование
//		TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0; //Настраиваем второй мультиплексор (для первого и второго входа):
// 		TIM3->CCER = TIM_CCER_CC1P | TIM_CCER_CC2P; //С детектора фронтов возьмем не инверсный, т.е. активный уровень высокий (для первого и второго входа):
 // 		TIM3->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; //Всё, можно разрешать работу счетчика:
  		TIM3->ARR = 200;
 // 		TIM3->CR1 = TIM_CR1_CEN;


//
	RCC->APB2ENR = RCC_APB2ENR_AFIOEN;


	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
		TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
		TIM_Cmd(TIM3, ENABLE);
		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
		  NVIC_EnableIRQ(TIM3_IRQn);
		  NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM4_Config(void)
{
	RCC->APB1ENR += RCC_APB1ENR_TIM4EN; //аккуратно!!
	TIM4->PSC = 24000-1;
	TIM4->ARR = 1000;
	TIM4->DIER = TIM_DIER_UIE;
	TIM4->CR1 = TIM_CR1_CEN;
	//NVIC_EnableIRQ(TIM4_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    if (!capture_is_first_t3)
      capture_is_ready_t3 = 1;

    capture_is_first_t3 = 0;
    /* Р’ Р±РёС‚Рµ TIM_CR1_DIR СЂРµРіРёСЃС‚СЂР° TIM3_CR1 С…СЂР°РЅРёС‚СЃСЏ
       РЅР°РїСЂР°РІР»РµРЅРёРµ РІСЂР°С‰РµРЅРёСЏ СЌРЅРєРѕРґРµСЂР°, Р·Р°РїРѕРјРёРЅР°РµРј РµРіРѕ. */
    captured_direction_t3 = (TIM3->CR1 & TIM_CR1_DIR ? FORWARD : BACKWARD);
    if(captured_direction_t3 == FORWARD) {QE2_F_Inc++; QE2++;}
    else {QE2_B_Inc++; QE2--;}

  }
}

void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    if (!capture_is_first_t2)
      capture_is_ready_t2 = 1;

    capture_is_first_t2 = 0;
    /* Р’ Р±РёС‚Рµ TIM_CR1_DIR СЂРµРіРёСЃС‚СЂР° TIM3_CR1 С…СЂР°РЅРёС‚СЃСЏ
       РЅР°РїСЂР°РІР»РµРЅРёРµ РІСЂР°С‰РµРЅРёСЏ СЌРЅРєРѕРґРµСЂР°, Р·Р°РїРѕРјРёРЅР°РµРј РµРіРѕ. */
    captured_direction_t2 = (TIM2->CR1 & TIM_CR1_DIR ? FORWARD : BACKWARD);
    if(captured_direction_t2 == FORWARD) {QE1_F_Inc++; QE1++;}
    else {QE1_B_Inc++, QE1--;}

  }
}

void TIM4_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);


}

