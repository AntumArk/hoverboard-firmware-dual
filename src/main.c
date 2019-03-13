/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "comms.h"
#include "sensorcoms.h"
#include "flashaccess.h"
#include "protocol.h"
#include "bldc.h"
#include "hallinterrupts.h"
#include "softwareserial.h"
//#include "hd44780.h"
#include "pid.h"
#include "flashcontent.h"


#include <memory.h>

void SystemClock_Config(void);

extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;
//LCD_PCF8574_HandleTypeDef lcd;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;

int cmd1;  // normalized input values. -1000 to 1000
int cmd2;
int cmd3;

typedef struct{
  uint8_t mode;
   uint8_t steer;
   uint8_t speed;
   //uint32_t crc;
} Serialcommand;
int scale[2] = {15, 15};

volatile Serialcommand command;
uint8_t buffer[9];
   
enum Mode 
{
    MANUAL_MODE = 0,
    AUTOMATIC_MODE = 1
};
static int mode = AUTOMATIC_MODE;
int packagePos = 0;
static int speedValue = 0;
static int steerValue = 0;

int disablepoweroff = 0;
int powerofftimer = 0;


extern volatile unsigned int timerval;
extern volatile unsigned int ssbits;

uint8_t button1, button2;

int steer; // global variable for steering. -1000 to 1000
int speed; // global variable for speed. -1000 to 1000

extern volatile int pwml;  // global variable for pwm left. -1000 to 1000
extern volatile int pwmr;  // global variable for pwm right. -1000 to 1000
extern volatile int weakl; // global variable for field weakening left. -1000 to 1000
extern volatile int weakr; // global variable for field weakening right. -1000 to 1000

extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...
int buzzerLen = 0;

extern uint8_t enable; // global variable for motor enable

extern volatile uint32_t timeout; // global variable for timeout
extern float batteryVoltage; // global variable for battery voltage

uint32_t inactivity_timeout_counter;
uint32_t debug_counter = 0;
int milli_vel_error_sum = 0;

///////////////////////////////////////////////////////////////
// define where to get serial data for protocol.c
// #if (INCLUDE_PROTOCOL == INCLUDE_PROTOCOL1) || (INCLUDE_PROTOCOL == INCLUDE_PROTOCOL2)
//   #ifdef SOFTWARE_SERIAL
//     int (*serial_available)() = softwareserial_available;
//     SERIAL_USART_IT_BUFFERTYPE serial_getrx() { return softwareserial_getrx();} 
//   #elif defined(SERIAL_USART2_IT) && !defined(READ_SENSOR) // READ_SENSOR uses SERIAL_USART2_IT
//     int serial_available() { return serial_usart_buffer_count(&usart2_it_RXbuffer); }
//     SERIAL_USART_IT_BUFFERTYPE serial_getrx() { return serial_usart_buffer_pop(&usart2_it_RXbuffer);} 
//   #elif defined(SERIAL_USART3_IT) && !defined(READ_SENSOR) // READ_SENSOR uses SERIAL_USART3_IT
//     int serial_available() { return serial_usart_buffer_count(&usart3_it_RXbuffer); }
//     SERIAL_USART_IT_BUFFERTYPE serial_getrx() { return serial_usart_buffer_pop(&usart3_it_RXbuffer);} 
//   #else
//     int serial_available() { return 0; }
//     SERIAL_USART_IT_BUFFERTYPE serial_getrx() { return 0; } 
//   #endif 
// #endif


void poweroff() {
  
        buzzerPattern = 0;
        enable = 0;
        for (int i = 0; i < 8; i++) {
            buzzerFreq = i;
            HAL_Delay(100);
        }
        HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 0);

        // if we are powered from sTLink, this bit allows the system to be started again with the button.
        while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {

        }

        while (1){
          if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)){
            HAL_NVIC_SystemReset();
          }
        }
    
}

// actually 'power'
int pwms[2] = {0, 0};

int dirs[2] = {-1, 1};
int dspeeds[2] = {0,0};


int main(void) {
  char tmp[200];
  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);

  SystemClock_Config();

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

  #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
    UART_Init();
  #endif

  #if (INCLUDE_PROTOCOL == INCLUDE_PROTOCOL1) || (INCLUDE_PROTOCOL == INCLUDE_PROTOCOL2)
    protocol_init();
  #endif


  memset((void*)&electrical_measurements, 0, sizeof(electrical_measurements));
//Possibly pin to keep board on
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 1);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  #ifdef SERIAL_USART2_IT
  //USART2_IT_init();
  #endif
  #ifdef SERIAL_USART3_IT
  USART3_IT_init();
  #endif

  for (int i = 8; i >= 0; i--) {
    buzzerFreq = i;
    HAL_Delay(100);
  }
  buzzerFreq = 0;
  //Turns on board LED
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

  #ifdef HALL_INTERRUPTS
    // enables interrupt reading of hall sensors for dead reconing wheel position.
    HallInterruptinit();
  #endif


  #ifdef CONTROL_SERIAL_USART2
    UART_Control_Init();
   // HAL_UART_Receive_DMA(&huart2, (uint8_t *)&buffer, 9);

  #endif

  float board_temp_adc_filtered = (float)adc_buffer.temp;
  float board_temp_deg_c;

  enable = 1;  // enable motors

  // ####### POWEROFF BY POWER-BUTTON #######
  int power_button_held = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);
  
  unsigned int startup_counter = 0;


  while(1) {
    //startup_counter++;
   if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) ) {
    

    cmd1 = 0;
    cmd2 = 0;


    //Control protocol.
    #ifdef CONTROL_SERIAL_USART2
   HAL_UART_Receive(&huart2,buffer,sizeof(buffer),100);
uint8_t temp_buffer[9];
 memcpy(temp_buffer, buffer, sizeof(buffer));
//*temp_buffer=&buffer; //Maybe?
if(checkMessage(&temp_buffer[0], SERIAL_USART_BUFFER_SIZE))
 {
 command.mode=temp_buffer[2];
     command.speed=temp_buffer[3];
     command.steer=temp_buffer[4];

    // if (buffer[0]=='A'||buffer[0]=='M') {
    //  command.mode=buffer[0];
    //  command.speed=buffer[1];
    //  command.steer=buffer[2];
    // }
    // if (buffer[1]=='A'||buffer[1]=='M') {
    //   command.mode=buffer[1];
    //  command.speed=buffer[2];
    //  command.steer=buffer[0];
    // }
    // if (buffer[2]=='A'||buffer[2]=='M') {
    //   command.mode=buffer[2];
    //  command.speed=buffer[0];
    //  command.steer=buffer[1];
    // }
    
      switch(command.mode)
    {
        case 'M':
            mode = MANUAL_MODE;
            break;
        case 'A':
            mode = AUTOMATIC_MODE;
           cmd1 = 0;
           cmd2 = 0;
            break;
    }

    if (mode == MANUAL_MODE )
    {
            speedValue = command.speed - MAX_VALUE / 2;
            speedValue *= MAX_SPEED;
            speedValue /= (MAX_VALUE / 2);
            steerValue = command.steer- MAX_VALUE / 2;
            steerValue *= MAX_SPEED;
            steerValue /= (MAX_VALUE / 2);
          //  cmd1 = CLAMP(speedValue * SPEED_COEFFICIENT -  steerValue * STEER_COEFFICIENT, -MAX_SPEED, MAX_SPEED);
          // cmd2 = CLAMP(speedValue * SPEED_COEFFICIENT +  steerValue * STEER_COEFFICIENT, -MAX_SPEED, MAX_SPEED);
           cmd1 = speedValue;
          cmd2 = steerValue;
      //   cmd1=200;
        }

         
        
    

      timeout = 0;
 }
 else
 {
  printf("Failed to receive packet");
 }
 
    #endif
//speed=-200;
      // ####### LOW-PASS FILTER #######
      steer = steer * (1.0 - FILTER) + cmd1 * FILTER;
      speed = speed * (1.0 - FILTER) + cmd2 * FILTER;
//steer=200;

      // ####### MIXER #######
      pwms[0] = CLAMP(speed * SPEED_COEFFICIENT -  steer * STEER_COEFFICIENT, -1000, 1000);
      pwms[1] = CLAMP(speed * SPEED_COEFFICIENT +  steer * STEER_COEFFICIENT, -1000, 1000);

    #ifdef ADDITIONAL_CODE
      ADDITIONAL_CODE;
    #endif
    #ifdef INVERT_R_DIRECTION
      pwmr = pwms[1];
    #else
      pwmr = -pwms[1];
    #endif
    #ifdef INVERT_L_DIRECTION
      pwml = -pwms[0];
    #else
      pwml = pwms[0];
    #endif

//    for (int i = 0; i < 2; i++){
//      lastspeeds[i] = pwms[i];
//    }

    if ((debug_counter++) % 100 == 0) {
      // ####### CALC BOARD TEMPERATURE #######
      board_temp_adc_filtered = board_temp_adc_filtered * 0.99 + (float)adc_buffer.temp * 0.01;
      board_temp_deg_c = ((float)TEMP_CAL_HIGH_DEG_C - (float)TEMP_CAL_LOW_DEG_C) / ((float)TEMP_CAL_HIGH_ADC - (float)TEMP_CAL_LOW_ADC) * (board_temp_adc_filtered - (float)TEMP_CAL_LOW_ADC) + (float)TEMP_CAL_LOW_DEG_C;
      
      electrical_measurements.board_temp_raw = adc_buffer.temp;
      electrical_measurements.board_temp_filtered = board_temp_adc_filtered;
      electrical_measurements.board_temp_deg_c = board_temp_deg_c;
      electrical_measurements.charging = !(CHARGER_PORT->IDR & CHARGER_PIN);

      // ####### DEBUG SERIAL OUT #######
  

      setScopeChannel(2, (int)pwms[1]);  // 3: output speed: 0-1000
      setScopeChannel(3, (int)pwms[0]);  // 4: output speed: 0-1000
      setScopeChannel(4, (int)adc_buffer.batt1);  // 5: for battery voltage calibration
      setScopeChannel(5, (int)(batteryVoltage * 100.0f));  // 6: for verifying battery voltage calibration
      setScopeChannel(6, (int)board_temp_adc_filtered);  // 7: for board temperature calibration
      setScopeChannel(7, (int)board_temp_deg_c);  // 8: for verifying board temperature calibration
      consoleScope();

//      SoftwareSerialReadTimer();
    }


    // if (power_button_held){
    
    //   // ####### POWEROFF BY POWER-BUTTON #######
    
      
       
    

    // if we plug in the charger, keep us alive
    // also if we have deliberately turned off poweroff over serial
    if (electrical_measurements.charging || disablepoweroff){
      inactivity_timeout_counter = 0;
    }

    // ####### BEEP AND EMERGENCY POWEROFF #######
    if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && ABS(speed) < 20) || (batteryVoltage < ((float)BAT_LOW_DEAD * (float)BAT_NUMBER_OF_CELLS) && ABS(speed) < 20)) {  // poweroff before mainboard burns OR low bat 3
      poweroff();
    } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {  // beep if mainboard gets hot
      buzzerFreq = 4;
      buzzerPattern = 1;
    } else if (batteryVoltage < ((float)BAT_LOW_LVL1 * (float)BAT_NUMBER_OF_CELLS) && batteryVoltage > ((float)BAT_LOW_LVL2 * (float)BAT_NUMBER_OF_CELLS) && BAT_LOW_LVL1_ENABLE) {  // low bat 1: slow beep
      buzzerFreq = 5;
      buzzerPattern = 42;
    } else if (batteryVoltage < ((float)BAT_LOW_LVL2 * (float)BAT_NUMBER_OF_CELLS) && batteryVoltage > ((float)BAT_LOW_DEAD * (float)BAT_NUMBER_OF_CELLS) && BAT_LOW_LVL2_ENABLE) {  // low bat 2: fast beep
      buzzerFreq = 5;
      buzzerPattern = 6;
    } else if (BEEPS_BACKWARD && speed < -50) {  // backward beep
      buzzerFreq = 5;
      buzzerPattern = 1;
    } else {  // do not beep
      if (buzzerLen > 0){
        buzzerLen--;
      } else {
        buzzerFreq = 0;
        buzzerPattern = 0;
      }
    }


    // ####### INACTIVITY TIMEOUT #######
    if (ABS(pwms[0]) > 50 || ABS(pwms[1]) > 50) {
      inactivity_timeout_counter = 0;
    } else {
      inactivity_timeout_counter ++;
    }

    // inactivity 10s warning; 1s bleeping
    if ((inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 50 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) &&
        (buzzerFreq == 0)) {
      buzzerFreq = 3;
      buzzerPattern = 1;
      buzzerLen = 1000;
    }

    // inactivity 5s warning; 1s bleeping
    if ((inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 55 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) &&
        (buzzerFreq == 0)) {
      buzzerFreq = 2;
      buzzerPattern = 1;
      buzzerLen = 1000;
    }

    // power off after ~60s of inactivity
    if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) {  // rest of main loop needs maybe 1ms
      inactivity_timeout_counter = 0;
      poweroff();
    }


    if (powerofftimer > 0){
      powerofftimer --;

      // spit a msg every 2 seconds
      if (!(powerofftimer % (2000/DELAY_IN_MAIN_LOOP))){
        char tmp[30];
        sprintf(tmp, "power off in %ds\r\n", (powerofftimer*DELAY_IN_MAIN_LOOP)/1000 );
        consoleLog(tmp);
      }

      if (powerofftimer <= 10000/DELAY_IN_MAIN_LOOP){
        buzzerFreq = 3;
        buzzerPattern = 1;
        buzzerLen = 1000;
      }

      if (powerofftimer <= 5000/DELAY_IN_MAIN_LOOP){
        buzzerFreq = 2;
        buzzerPattern = 1;
        buzzerLen = 1000;
      }

      if (powerofftimer <= 0){
        powerofftimer = 0;
        poweroff();
      }
    }
     HAL_Delay(DELAY_IN_MAIN_LOOP);
  }
 
  
else
 poweroff();
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;  // 8 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
}
