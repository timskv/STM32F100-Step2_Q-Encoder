#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>
#include <misc.h>

typedef enum { FORWARD, BACKWARD } Direction;

volatile uint8_t capture_is_first = 1, capture_is_ready = 0;
volatile Direction captured_direction = FORWARD;

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
}

void init_timer(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* Разрешаем счёт в обе стороны, период ставим 4 */
  TIM_TimeBaseInitTypeDef timer_base;
  TIM_TimeBaseStructInit(&timer_base);
  timer_base.TIM_Period = 4;
  timer_base.TIM_CounterMode = TIM_CounterMode_Down | TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &timer_base);

  /* Считать будем все переходы лог. уровня с обоих каналов */
  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
      TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM3, ENABLE);

  NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    if (!capture_is_first)
      capture_is_ready = 1;

    capture_is_first = 0;
    /* В бите TIM_CR1_DIR регистра TIM3_CR1 хранится
       направление вращения энкодера, запоминаем его. */
    captured_direction = (TIM3->CR1 & TIM_CR1_DIR ? FORWARD : BACKWARD);

  }
}
