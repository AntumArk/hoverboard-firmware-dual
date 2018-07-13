/**
  ******************************************************************************
  * @file    gd32f1x0_stm32_compat.c
  * @author  Emerick Herve
  * @version V1.0.0
  * @date    13-Jul-2018
  * @brief   STM32 compatibility lib
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0_stm32_compat.h"

#ifndef __WEAK
	#define __WEAK attribute((weak))
#endif
#ifndef __weak
	#define __weak attribute((weak))
#endif
