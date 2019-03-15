/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "config.h"
#include "hallinterrupts.h"
#include "softwareserial.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/////////////////////////////////////////
// EXTI interrupts - used for HallInterrupt, Softwarewareserial, and PWM

void EXTI0_IRQHandler(void)
{
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
#ifdef SOFTWARE_SERIAL
  if (SOFTWARE_SERIAL_RX_PIN == GPIO_PIN_0)
    softwareserialRXInterrupt();
#endif
}

void EXTI1_IRQHandler(void)
{
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
#ifdef SOFTWARE_SERIAL
  if (SOFTWARE_SERIAL_RX_PIN == GPIO_PIN_1)
    softwareserialRXInterrupt();
#endif
}

void EXTI2_IRQHandler(void)
{
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
#ifdef SOFTWARE_SERIAL
  if (SOFTWARE_SERIAL_RX_PIN == GPIO_PIN_2)
    softwareserialRXInterrupt();
#endif
}

void EXTI3_IRQHandler(void)
{

  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
#ifdef SOFTWARE_SERIAL
  if (SOFTWARE_SERIAL_RX_PIN == GPIO_PIN_3)
    softwareserialRXInterrupt();
#endif
}

void EXTI4_IRQHandler(void)
{
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
#ifdef SOFTWARE_SERIAL
  if (SOFTWARE_SERIAL_RX_PIN == GPIO_PIN_4)
    softwareserialRXInterrupt();
#endif
}

/////////////////////////////////////////////////////////////////////
// actual IRQ for LEFT pins 5,6,7
void EXTI9_5_IRQHandler(void)
{
  unsigned long triggered = 0;
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9) != RESET)
  {
    /* Clear the EXTI line 8 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
    triggered |= GPIO_PIN_9;
  }

  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET)
  {
    /* Clear the EXTI line 9 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
    triggered |= GPIO_PIN_9;
  }

  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
    triggered |= GPIO_PIN_7;
  }

  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
    triggered |= GPIO_PIN_6;
  }
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
    triggered |= GPIO_PIN_5;
  }

#ifdef HALL_INTERRUPTS
  if (triggered & HALL_PIN_MASK)
    HallInterruptsInterrupt();
#endif

// shared interrupt for these pins, depending on where the sfotware serial pin is
#ifdef SOFTWARE_SERIAL
  if (triggered & SOFTWARE_SERIAL_RX_PIN)
  {
    softwareserialRXInterrupt();
  }
#endif
}

/////////////////////////////////////////////////////////////////////
// actual IRQ for RIGHT pins 10, 11, 12
void EXTI15_10_IRQHandler(void)
{
  unsigned long triggered = 0;
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15) != RESET)
  {
    /* Clear the EXTI line 8 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
    triggered |= GPIO_PIN_15;
  }

  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_14) != RESET)
  {
    /* Clear the EXTI line 9 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);
    triggered |= GPIO_PIN_14;
  }

  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
    triggered |= GPIO_PIN_13;
  }
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
    triggered |= GPIO_PIN_12;
  }
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
    triggered |= GPIO_PIN_11;
  }
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
    triggered |= GPIO_PIN_10;
  }

#ifdef HALL_INTERRUPTS
  if (triggered & HALL_PIN_MASK)
    HallInterruptsInterrupt();
#endif
// shared interrupt for these pins, depending on where the sfotware serial pin is
#ifdef SOFTWARE_SERIAL
  if (triggered & SOFTWARE_SERIAL_RX_PIN)
  {
    softwareserialRXInterrupt();
  }
#endif
}
// end EXTI
/////////////////////////////////////////

/////////////////////////////////////////
// timer interrupts

//
// TIM2 is used in softwareserial, but without interrupts.
// void TIM3_IRQHandler(void) - defined in softwareserial.c
// void TIM4_IRQHandler(void) - defined in hallinterrupts.c
//

// end timer interrupts
/////////////////////////////////////////

/////////////////////////////////////////

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
