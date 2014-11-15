#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>
#include <misc.h>

typedef enum { FORWARD, BACKWARD } Direction;

volatile uint8_t capture_is_first_t3 = 1, capture_is_ready_t3 = 0;
volatile uint8_t capture_is_first_t2 = 1, capture_is_ready_t2 = 0;
volatile Direction captured_direction_t2 = FORWARD;
volatile Direction captured_direction_t3 = FORWARD;

void init_gpio(void);
void init_timer(void);

void init_gpio(void)
{
  GPIO_InitTypeDef gpio_cfg;
  GPIO_StructInit(&gpio_cfg);

  /* Каналы 1 и 2 таймера TIM3 - на вход, подтянуть к питанию */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  gpio_cfg.GPIO_Mode = GPIO_Mode_IPU;
  gpio_cfg.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_Init(GPIOA, &gpio_cfg);

  gpio_cfg.GPIO_Mode = GPIO_Mode_IPU;
  gpio_cfg.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_Init(GPIOA, &gpio_cfg);
}

void init_timer(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  /* Разрешаем счёт в обе стороны, период ставим 4 */
  TIM_TimeBaseInitTypeDef timer_base;
  TIM_TimeBaseStructInit(&timer_base);
  timer_base.TIM_Period = 4;
  timer_base.TIM_CounterMode = TIM_CounterMode_Down | TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &timer_base);
  TIM_TimeBaseInit(TIM2, &timer_base);

  /* Считать будем все переходы лог. уровня с обоих каналов */
  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
      TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
      TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, ENABLE);

  NVIC_EnableIRQ(TIM3_IRQn);
  NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    if (!capture_is_first_t3)
      capture_is_ready_t3 = 1;

    capture_is_first_t3 = 0;
    /* В бите TIM_CR1_DIR регистра TIM3_CR1 хранится
       направление вращения энкодера, запоминаем его. */
    captured_direction_t3 = (TIM3->CR1 & TIM_CR1_DIR ? FORWARD : BACKWARD);

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
    /* В бите TIM_CR1_DIR регистра TIM3_CR1 хранится
       направление вращения энкодера, запоминаем его. */
    captured_direction_t2 = (TIM2->CR1 & TIM_CR1_DIR ? FORWARD : BACKWARD);

  }
}
