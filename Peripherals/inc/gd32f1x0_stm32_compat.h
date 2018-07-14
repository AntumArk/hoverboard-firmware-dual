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

//#ifndef uint16_t
//	#define uint16_t uint16
//#endif
//#define uint32_t uint32

#ifndef __WEAK
	#define __WEAK __attribute__((weak))
#endif
#ifndef __weak
	#define __weak __attribute__((weak))
#endif

#define USART_CR1_RE          USART_CTLR1_REN
#define USART_CR1_TE          USART_CTLR1_TEN
#define USART_CR1_DMAT        USART_CTLR1_DEM
#define CR1                   CTLR1

#define USART_CR2_RE          USART_CTLR2_REN
#define USART_CR2_TE          USART_CTLR2_TEN
#define USART_CR2_DMAT        USART_CTLR2_DEM
#define CR2                   CTLR2

#define USART_CR3_RE          USART_CTLR3_REN
#define USART_CR3_TE          USART_CTLR3_TEN
#define USART_CR3_DMAT        USART_CTLR3_DEM
#define CR3                   CTLR3


#define DMA1_Channel1         DMA1_CHANNEL1
#define DMA1_Channel2         DMA1_CHANNEL2
#define DMA1_Channel3         DMA1_CHANNEL3
#define DMA1_Channel4         DMA1_CHANNEL4
#define DMA1_Channel5         DMA1_CHANNEL5
#define DMA1_Channel6         DMA1_CHANNEL6
#define DMA1_Channel7         DMA1_CHANNEL7

#define DMA_MemoryInc_Disable MemInc_DISABLE
#define DMA_CCR_MINC          MemInc_ENABLE
#define DMA_MemoryInc_Enable  MemInc_ENABLE

#define DMA_DIR_PeripheralSRC Direction_PERIPHERALSRC
#define DMA_DIR_PeripheralDST Direction_PERIPHERALDST
#define DMA_CCR_DIR           Direction_PERIPHERALDST


#define DMA_IFCR_CGIF1        DMA_IFR_GIF1
#define DMA_IFCR_CTCIF1       DMA_IFR_TCIF1
#define DMA_IFCR_CHTIF1       DMA_IFR_HTIF1

#define DMA_IFCR_CGIF2        DMA_IFR_GIF2
#define DMA_IFCR_CTCIF2       DMA_IFR_TCIF2
#define DMA_IFCR_CHTIF2       DMA_IFR_HTIF2

#define DMA_IFCR_CGIF3        DMA_IFR_GIF3
#define DMA_IFCR_CTCIF3       DMA_IFR_TCIF3
#define DMA_IFCR_CHTIF3       DMA_IFR_HTIF3

#define DMA_IFCR_CGIF4        DMA_IFR_GIF4
#define DMA_IFCR_CTCIF4       DMA_IFR_TCIF4
#define DMA_IFCR_CHTIF4       DMA_IFR_HTIF4

#define DMA_IFCR_CGIF5        DMA_IFR_GIF5
#define DMA_IFCR_CTCIF5       DMA_IFR_TCIF5
#define DMA_IFCR_CHTIF5       DMA_IFR_HTIF5

#define DMA_IFCR_CGIF6        DMA_IFR_GIF6
#define DMA_IFCR_CTCIF6       DMA_IFR_TCIF6
#define DMA_IFCR_CHTIF6       DMA_IFR_HTIF6

#define DMA_IFCR_CGIF7        DMA_IFR_GIF7
#define DMA_IFCR_CTCIF7       DMA_IFR_TCIF7
#define DMA_IFCR_CHTIF7       DMA_IFR_HTIF7

#define IFCR         		  IFR


#define CCR                   CTLRx
#define CNDTR                 RCNTx
#define CPAR                  PBARx
#define CMAR                  MBARx


#define TIMER_COUNTERMODE_CENTERALIGNED1 TIMER_COUNTER_CENTER_ALIGNED1
#define TIMER_CLOCKDIVISION_DIV1         TIMER_CDIV_DIV1
#define TIMER_AUTORELOAD_PRELOAD_DISABLE TIMER_OC_PRELOAD_DISABLE
#define TIMER_MASTERSLAVEMODE_ENABLE     TIMER_MASTER_SLAVE_MODE_ENABLE

#define TIMER_CR2_MMS_1                  TIMER_CTLR2_MMC_1


#define USART1->DR                       USART1->RDTR

#endif
