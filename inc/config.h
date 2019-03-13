#pragma once
#include "stm32f1xx_hal.h"

//////////////////////////////////////////////////////////
// macro types for the hoverboard control style
// add to add other control combinations
#define HOVERBOARD_WITH_SOFTWARE_SERIAL_B2_C9 1
#define USART2_CONTROLLED 2
#define USART3_CONTROLLED 3
#define SOFTWARE_SERIAL_A2_A3 4

// thoery says this is the only thing you need to change....
#define CONTROL_TYPE USART2_CONTROLLED
//////////////////////////////////////////////////////////


// implementaiton of specific for macro control types
#if (CONTROL_TYPE == USART2_CONTROLLED)
  // hoverboard sensor functionality is disabled
  // and control is via USART2
  #define SERIAL_USART2_IT
//  #define DEBUG_SERIAL_ASCII
#endif


// implementaiton of specific for macro control types
#if (CONTROL_TYPE == USART3_CONTROLLED)
  // hoverboard sensor functionality is disabled
  // and control is via USART3
  #define SERIAL_USART3_IT
  //#define DEBUG_SERIAL_ASCII
#endif


// ############################### DO-NOT-TOUCH SETTINGS ###############################

#define PWM_FREQ         16000      // PWM frequency in Hz
#define DEAD_TIME        32         // PWM deadtime

#define DELAY_IN_MAIN_LOOP 5        // in ms. default 5. it is independent of all the timing critical stuff. do not touch if you do not know what you are doing.

#define TIMEOUT          5          // number of wrong / missing input commands before emergency off

// ############################### GENERAL ###############################

// How to calibrate: connect GND and RX of a 3.3v uart-usb adapter to the right sensor board cable (be careful not to use the red wire of the cable. 15v will destroye verything.). if you are using nunchuck, disable it temporarily. enable DEBUG_SERIAL_USART3 and DEBUG_SERIAL_ASCII use asearial terminal.

// Battery voltage calibration: connect power source. see <How to calibrate>. write value nr 5 to BAT_CALIB_ADC. make and flash firmware. then you can verify voltage on value 6 (devide it by 100.0 to get calibrated voltage).
#define BAT_CALIB_REAL_VOLTAGE        41.5       // input voltage measured by multimeter  
#define BAT_CALIB_ADC                 1704       // adc-value measured by mainboard (value nr 4 on UART debug output)

#define BAT_NUMBER_OF_CELLS     10        // normal Hoverboard battery: 10s
#define BAT_LOW_LVL1_ENABLE      1       // to beep or not to beep, 1 or 0
#define BAT_LOW_LVL1            3.6       // gently beeps at this voltage level. [V/cell]
#define BAT_LOW_LVL2_ENABLE     1         // to beep or not to beep, 1 or 0
#define BAT_LOW_LVL2            3.5       // your battery is almost empty. Charge now! [V/cell]
#define BAT_LOW_DEAD            3.37      // undervoltage poweroff. (while not driving) [V/cell]

#define DC_CUR_LIMIT     15         // DC current limit in amps per motor. so 15 means it will draw 30A out of your battery. it does not disable motors, it is a soft current limit.

// Board overheat detection: the sensor is inside the STM/GD chip. it is very inaccurate without calibration (up to 45°C). so only enable this funcion after calibration! let your board cool down. see <How to calibrate>. get the real temp of the chip by thermo cam or another temp-sensor taped on top of the chip and write it to TEMP_CAL_LOW_DEG_C. write debug value 8 to TEMP_CAL_LOW_ADC. drive around to warm up the board. it should be at least 20°C warmer. repeat it for the HIGH-values. enable warning and/or poweroff and make and flash firmware.
#define TEMP_CAL_LOW_ADC        1655      // temperature 1: ADC value
#define TEMP_CAL_LOW_DEG_C      35.8      // temperature 1: measured temperature [°C]
#define TEMP_CAL_HIGH_ADC       1588      // temperature 2: ADC value
#define TEMP_CAL_HIGH_DEG_C     48.9      // temperature 2: measured temperature [°C]
#define TEMP_WARNING_ENABLE     0         // to beep or not to beep, 1 or 0, DO NOT ACTIVITE WITHOUT CALIBRATION!
#define TEMP_WARNING            60        // annoying fast beeps [°C]
#define TEMP_POWEROFF_ENABLE    0         // to poweroff or not to poweroff, 1 or 0, DO NOT ACTIVITE WITHOUT CALIBRATION!
#define TEMP_POWEROFF           65        // overheat poweroff. (while not driving) [°C]

#define INACTIVITY_TIMEOUT 8        // minutes of not driving until poweroff. it is not very precise.

// ############################### SERIAL DEBUG ###############################

//#define DEBUG_SERIAL_USART3         // right sensor board cable, disable if I2C (nunchuck or lcd) is used!
//#define DEBUG_SERIAL_SENSOR         // send to USART3 sensor board, without framing, at the CONTROL_SENSOR_BAUD rate
//#define DEBUG_SERIAL_SERVOTERM
//#define DEBUG_SERIAL_ASCII          // "1:345 2:1337 3:0 4:0 5:0 6:0 7:0 8:0\r\n"

// ############################### INPUT ###############################

// ###### CONTROL VIA UART (serial) ######
#define CONTROL_SERIAL_USART2       // left sensor board cable, disable if ADC or PPM is used!
 #define CONTROL_BAUD 19200
                                      // control via usart from eg an Arduino or raspberry
// for Arduino, use void loop(void){ Serial.write((uint8_t *) &steer, sizeof(steer)); Serial.write((uint8_t *) &speed, sizeof(speed));delay(20); }

//////////////////////////////////////////////////////////////////

//  #define SERIAL_USART2_IT
  #define USART2_BAUD       19200                  // UART baud rate
  #define USART2_WORDLENGTH UART_WORDLENGTH_8B      // UART_WORDLENGTH_8B or UART_WORDLENGTH_9B

// #define SERIAL_USART3_IT
  #define USART3_BAUD       19200                  // UART baud rate
  #define USART3_WORDLENGTH UART_WORDLENGTH_8B      // UART_WORDLENGTH_8B or UART_WORDLENGTH_9B

  #define SERIAL_USART_IT_BUFFERTYPE  unsigned char // char or short


// ############################### ENABLE INTERRUPT READING OF HALL SENSORS FOR POSITION ###############################
#define HALL_INTERRUPTS
#define WHEEL_SIZE_INCHES 9.8 //- set to your wheelsize to override the default 6.5



// ############################### SERIAL PROTOCOL ###############################
#define NO_PROTOCOL 0
#define INCLUDE_PROTOCOL1 1 // enables processing of input characters through 'protocol.c'
//#define INCLUDE_PROTOCOL2 2 // enables processing of input characters through 'machine_protocol.c'

#define INCLUDE_PROTOCOL NO_PROTOCOL
//#define INCLUDE_PROTOCOL INCLUDE_PROTOCOL1
//#define INCLUDE_PROTOCOL INCLUDE_PROTOCOL2

// ############################### DRIVING BEHAVIOR ###############################

// inputs:
// - cmd1 and cmd2: analog normalized input values. -1000 to 1000
// - button1 and button2: digital input values. 0 or 1
// - adc_buffer.l_tx2 and adc_buffer.l_rx2: unfiltered ADC values (you do not need them). 0 to 4095
// outputs:
// - speedR and speedL: normal driving -1000 to 1000
// - weakr and weakl: field weakening for extra boost at high speed (speedR > 700 and speedL > 700). 0 to ~400

#define FILTER              0.05  // lower value == softer filter. do not use values <0.01, you will get float precision issues.
#define SPEED_COEFFICIENT   0.8  // higher value == stronger. 0.0 to ~2.0?
#define STEER_COEFFICIENT  0.8  // higher value == stronger. if you do not want any steering, set it to 0.0; 0.0 to 1.0
#define MAX_VALUE           80
#define MAX_SPEED           700
#define INVERT_R_DIRECTION
//#define INVERT_L_DIRECTION
#define BEEPS_BACKWARD 0   // 0 or 1


#if (INCLUDE_PROTOCOL == NO_PROTOCOL)
  #undef INCLUDE_PROTOCOL
#endif

// end of macro control type definitions
//////////////////////////////////////////////////////////
