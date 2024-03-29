/**
  ******************************************************************************
  * @file    hal_tick.h
  * @author  MCD Application Team
  * @brief   Initialization of HAL tick
  ******************************************************************************  
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************  
  */ 
#ifndef __HAL_TICK_H
#define __HAL_TICK_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f3xx.h"
#include "stm32f3xx_ll_tim.h"
#include "cmsis_nvic.h"
   
#define TIM_MST      TIM2
#define TIM_MST_IRQ  TIM2_IRQn
#define TIM_MST_RCC  __TIM2_CLK_ENABLE()

#define TIM_MST_RESET_ON   __TIM2_FORCE_RESET()
#define TIM_MST_RESET_OFF  __TIM2_RELEASE_RESET()

#define TIM_MST_16BIT  0 // 1=16-bit timer, 0=32-bit timer

#define TIM_MST_PCLK  1 // Select the peripheral clock number (1 or 2)

#define HAL_TICK_DELAY (1000) // 1 ms

#ifdef __cplusplus
}
#endif

#endif // __HAL_TICK_H

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
