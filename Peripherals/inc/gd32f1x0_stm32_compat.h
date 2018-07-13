/**
  ******************************************************************************
  * @file    gd32f1x0_stm32_compat.h
  * @author  Emerick Herve
  * @version V1.0.0
  * @date    13-Jul-2018
  * @brief   STM32 compatibility lib
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_STM32_COMPAT_H
#define __GD32F1X0_STM32_COMPAT_H

typedef uint16_t uint16;
typedef uint32_t uint32;

#ifndef __WEAK
	#define __WEAK __attribute__((weak))
#endif
#ifndef __weak
	#define __weak __attribute__((weak))
#endif
