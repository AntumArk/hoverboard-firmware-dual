/**
  ******************************************************************************
  * @file    gd32f1x0_gpio.h
  * @author  MCU SD
  * @version V1.0.1
  * @date    6-Sep-2014
  * @brief   GPIO header file of the firmware library.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD32F1X0_GPIO_H
#define __GD32F1X0_GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gd32f1x0.h"

/** @addtogroup GD32F1x0_Firmware
  * @{
  */

/** @addtogroup GPIO
  * @{
  */

/** @defgroup GPIO_Exported_Types
  * @{
  */

/** @defgroup Mode_enumeration 
  * @{
  */
  
#define  GPIO_MODE_INPUT                        0x00   /*!< Input Floating Mode                   */
#define  GPIO_MODE_OUTPUT_PP                    0x01   /*!< Output Push Pull Mode                 */
#define  GPIO_MODE_OUTPUT_OD                    GPIO_MODE_OUTPUT_PP   /*!< Output Open Drain Mode                */
#define  GPIO_MODE_AF_PP                        0x02   /*!< Alternate Function Push Pull Mode     */
#define  GPIO_MODE_AF_OD                        GPIO_MODE_AF_OD   /*!< Alternate Function Open Drain Mode    */
#define  GPIO_MODE_AF_INPUT                     GPIO_MODE_INPUT          /*!< Alternate Function Input Mode         */

#define  GPIO_MODE_ANALOG                       0x03   /*!< Analog Mode  */
    
/* Next defines are not tested! */
#define  GPIO_MODE_IT_RISING                    0x10110000U   /*!< External Interrupt Mode with Rising edge trigger detection          */
#define  GPIO_MODE_IT_FALLING                   0x10210000U   /*!< External Interrupt Mode with Falling edge trigger detection         */
#define  GPIO_MODE_IT_RISING_FALLING            0x10310000U   /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */
 
#define  GPIO_MODE_EVT_RISING                   0x10120000U   /*!< External Event Mode with Rising edge trigger detection               */
#define  GPIO_MODE_EVT_FALLING                  0x10220000U   /*!< External Event Mode with Falling edge trigger detection              */
#define  GPIO_MODE_EVT_RISING_FALLING           0x10320000U   /*!< External Event Mode with Rising/Falling edge trigger detection       */

typedef enum
{
    GPIO_MODE_IN   = 0x00,
    GPIO_MODE_OUT  = 0x01,
    GPIO_MODE_AF   = 0x02,
    GPIO_MODE_AN   = 0x03
}ModePara;

/**
  * @}
  */

/** @defgroup Output_type_enumeration
  * @{
  */

typedef enum
{
    GPIO_OTYPE_PP = 0x00,
    GPIO_OTYPE_OD = 0x01
}GPIO_OTypePara;

/**
  * @}
  */

/** @defgroup Output_Maximum_frequency_enumeration 
  * @{
  */
  
#define  GPIO_SPEED_FREQ_LOW              0x01 /*!< Low speed */
#define  GPIO_SPEED_FREQ_MEDIUM           0x02 /*!< Medium speed */
#define  GPIO_SPEED_FREQ_HIGH             0x03 /*!< High speed */

typedef enum
{
    GPIO_SPEED_10MHZ  = 0x01,
    GPIO_SPEED_2MHZ   = 0x02,
    GPIO_SPEED_50MHZ  = 0x03
}SpeedPara;

/**
  * @}
  */

/** @defgroup GPIO_Pull-Up_Pull-Down_enumeration 
  * @{
  */

#define  GPIO_NOPULL        0x00   /*!< No Pull-up or Pull-down activation  */
#define  GPIO_PULLUP        0x01   /*!< Pull-up activation                  */
#define  GPIO_PULLDOWN      0x02   /*!< Pull-down activation                */

typedef enum
{
    GPIO_PUPD_NOPULL     = 0x00,
    GPIO_PUPD_PULLUP     = 0x01,
    GPIO_PUPD_PULLDOWN   = 0x02
}PullPara;

/**
  * @}
  */

/** @defgroup Bit_State_enumeration
  * @{
  */
typedef enum
{ 
    Bit_RESET = 0,
    Bit_SET
}GPIO_PinState;

/**
  * @}
  */

/** 
  * @brief  GPIO Initial Parameters
  */
typedef struct
{
    uint32_t Pin;              /*!< The GPIO pins to be configured. choose several from @ref GPIO_pins_define */
    ModePara Mode;        /*!< The operating mode for the selected pins. choose one from @ref ModePara   */
    SpeedPara Speed;      /*!< The speed for the selected pins.choose one from @ref SpeedPara  */
    GPIO_OTypePara GPIO_OType;      /*!< The operating output type for the selected pins.choose one from @ref GPIO_OTypePara  */
    PullPara Pull;        /*!< The operating Pull-up/Pull down for the selected pins.choose one from @ref PullPara   */
}GPIO_InitTypeDef;

/**
  * @}
  */

/** @defgroup GPIO_Exported_Constants
  * @{
  */

/** @defgroup GPIO_pins_define 
  * @{
  */
#define GPIO_PIN_0                  ((uint16_t)0x0001)
#define GPIO_PIN_1                  ((uint16_t)0x0002)
#define GPIO_PIN_2                  ((uint16_t)0x0004)
#define GPIO_PIN_3                  ((uint16_t)0x0008)
#define GPIO_PIN_4                  ((uint16_t)0x0010)
#define GPIO_PIN_5                  ((uint16_t)0x0020)
#define GPIO_PIN_6                  ((uint16_t)0x0040)
#define GPIO_PIN_7                  ((uint16_t)0x0080)
#define GPIO_PIN_8                  ((uint16_t)0x0100)
#define GPIO_PIN_9                  ((uint16_t)0x0200)
#define GPIO_PIN_10                 ((uint16_t)0x0400)
#define GPIO_PIN_11                 ((uint16_t)0x0800)
#define GPIO_PIN_12                 ((uint16_t)0x1000)
#define GPIO_PIN_13                 ((uint16_t)0x2000)
#define GPIO_PIN_14                 ((uint16_t)0x4000)
#define GPIO_PIN_15                 ((uint16_t)0x8000)
#define GPIO_PIN_ALL                ((uint16_t)0xFFFF)

/**
  * @}
  */

/** @defgroup Pin_sources 
  * @{
  */
#define GPIO_PINSOURCE0             ((uint8_t)0x00)
#define GPIO_PINSOURCE1             ((uint8_t)0x01)
#define GPIO_PINSOURCE2             ((uint8_t)0x02)
#define GPIO_PINSOURCE3             ((uint8_t)0x03)
#define GPIO_PINSOURCE4             ((uint8_t)0x04)
#define GPIO_PINSOURCE5             ((uint8_t)0x05)
#define GPIO_PINSOURCE6             ((uint8_t)0x06)
#define GPIO_PINSOURCE7             ((uint8_t)0x07)
#define GPIO_PINSOURCE8             ((uint8_t)0x08)
#define GPIO_PINSOURCE9             ((uint8_t)0x09)
#define GPIO_PINSOURCE10            ((uint8_t)0x0A)
#define GPIO_PINSOURCE11            ((uint8_t)0x0B)
#define GPIO_PINSOURCE12            ((uint8_t)0x0C)
#define GPIO_PINSOURCE13            ((uint8_t)0x0D)
#define GPIO_PINSOURCE14            ((uint8_t)0x0E)
#define GPIO_PINSOURCE15            ((uint8_t)0x0F)

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup GPIO_Private_Macros GPIO Private Macros
  * @{
  */
#define IS_GPIO_PIN_ACTION(ACTION) (((ACTION) == GPIO_PIN_RESET) || ((ACTION) == GPIO_PIN_SET))
#define IS_GPIO_PIN(PIN)           ((((PIN) & GPIO_PIN_MASK ) != 0x00U) && (((PIN) & ~GPIO_PIN_MASK) == 0x00U))
#define IS_GPIO_MODE(MODE) (((MODE) == GPIO_MODE_INPUT)              ||\
                            ((MODE) == GPIO_MODE_OUTPUT_PP)          ||\
                            ((MODE) == GPIO_MODE_OUTPUT_OD)          ||\
                            ((MODE) == GPIO_MODE_AF_PP)              ||\
                            ((MODE) == GPIO_MODE_AF_OD)              ||\
                            ((MODE) == GPIO_MODE_IT_RISING)          ||\
                            ((MODE) == GPIO_MODE_IT_FALLING)         ||\
                            ((MODE) == GPIO_MODE_IT_RISING_FALLING)  ||\
                            ((MODE) == GPIO_MODE_EVT_RISING)         ||\
                            ((MODE) == GPIO_MODE_EVT_FALLING)        ||\
                            ((MODE) == GPIO_MODE_EVT_RISING_FALLING) ||\
                            ((MODE) == GPIO_MODE_ANALOG))
#define IS_GPIO_SPEED(SPEED) (((SPEED) == GPIO_SPEED_FREQ_LOW) || \
                              ((SPEED) == GPIO_SPEED_FREQ_MEDIUM) || ((SPEED) == GPIO_SPEED_FREQ_HIGH))
#define IS_GPIO_PULL(PULL) (((PULL) == GPIO_NOPULL) || ((PULL) == GPIO_PULLUP) || \
                            ((PULL) == GPIO_PULLDOWN))
/**
  * @}
  */
  
/* Exported macro ------------------------------------------------------------*/
/** @defgroup GPIO_Exported_Macros GPIO Exported Macros
  * @{
  */

/**
  * @brief  Checks whether the specified EXTI line flag is set or not.
  * @param  __EXTI_LINE__: specifies the EXTI line flag to check.
  *         This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval The new state of __EXTI_LINE__ (SET or RESET).
  */
#define __HAL_GPIO_EXTI_GET_FLAG(__EXTI_LINE__) (EXTI->PR & (__EXTI_LINE__))

/**
  * @brief  Clears the EXTI's line pending flags.
  * @param  __EXTI_LINE__: specifies the EXTI lines flags to clear.
  *         This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
  * @retval None
  */
#define __HAL_GPIO_EXTI_CLEAR_FLAG(__EXTI_LINE__) (EXTI->PR = (__EXTI_LINE__))

/**
  * @brief  Checks whether the specified EXTI line is asserted or not.
  * @param  __EXTI_LINE__: specifies the EXTI line to check.
  *          This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval The new state of __EXTI_LINE__ (SET or RESET).
  */
#define __HAL_GPIO_EXTI_GET_IT(__EXTI_LINE__) (EXTI->PD & (__EXTI_LINE__)) // PD instead of PR? TO FIX ?

/**
  * @brief  Clears the EXTI's line pending bits.
  * @param  __EXTI_LINE__: specifies the EXTI lines to clear.
  *          This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
  * @retval None
  */
#define __HAL_GPIO_EXTI_CLEAR_IT(__EXTI_LINE__) (EXTI->PD = (__EXTI_LINE__))  // PD instead of PR? TO FIX ?

/**
  * @brief  Generates a Software interrupt on selected EXTI line.
  * @param  __EXTI_LINE__: specifies the EXTI line to check.
  *          This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval None
  */
#define __HAL_GPIO_EXTI_GENERATE_SWIT(__EXTI_LINE__) (EXTI->SWIER |= (__EXTI_LINE__))
/**
  * @}
  */
  
#ifndef GPIO_PIN_MASK
	#define GPIO_PIN_MASK              0x0000FFFFU /* PIN mask for assert test */
#endif

/** @defgroup GPIO_Alternate_function_selection_define 
  * @{
  */

#define GPIO_AF_0                   ((uint8_t)0x00) 
#define GPIO_AF_1                   ((uint8_t)0x01)
#define GPIO_AF_2                   ((uint8_t)0x02)
#define GPIO_AF_3                   ((uint8_t)0x03)
#define GPIO_AF_4                   ((uint8_t)0x04)
#define GPIO_AF_5                   ((uint8_t)0x05)
#define GPIO_AF_6                   ((uint8_t)0x06)
#define GPIO_AF_7                   ((uint8_t)0x07)

/**
  * @}
  */

/**
  * @}
  */

/* Include GPIO HAL Extension module */
#include "stm32f1xx_hal_gpio_ex.h"

/** @defgroup GPIO_Exported_Functions
  * @{
  */
void HAL_GPIO_DeInit(GPIO_TypeDef* GPIOx);
void HAL_GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitTypeDefStruct);
void GPIO_ParaInit(GPIO_InitTypeDef* GPIO_InitTypeDefStruct);
void HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t Pin);
uint8_t HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadOutputBit(GPIO_TypeDef* GPIOx, uint16_t Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t Pin, GPIO_PinState BitVal);
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t Pin);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
void PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t PinSource, uint8_t GPIO_AF);
void HAL_GPIO_EXTI_IRQHandler(uint16_t Pin);
void HAL_GPIO_EXTI_Callback(uint16_t Pin);

#ifdef __cplusplus
}
#endif

#endif /* __GD32F1X0_GPIO_H */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2014 GIGADEVICE *****END OF FILE****/
