/*
 * delay.c
 *
 *  Created on: 22 mar 2023
 *      Author: Ja
 */
#include "delay.h"
#include "stm32f1xx.h"
uint32_t tick;
  void Delay(uint32_t Delay)
  {
    uint32_t tickstart = tick;
    uint32_t wait = Delay;

    /* Add a freq to guarantee minimum wait */
    if (wait < 0xFFFFFFFFU)
    {
      wait += (uint32_t)(1);
    }

    while ((tick - tickstart) < wait)
    {
    }
  }

  void SysTick_Handler(void)
  {
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    tick++;
    /* USER CODE BEGIN SysTick_IRQn 1 */
Loop1ms();
    /* USER CODE END SysTick_IRQn 1 */
  }
