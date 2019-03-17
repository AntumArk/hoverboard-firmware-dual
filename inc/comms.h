#pragma once
#include "crc.h"
//Transmitted data count
#define BuffSize 9
#define R_BuffSize 128
//#define IS_MASTER //The one who sends

static volatile uint8_t H1 = 0xF1; //Header 1
static volatile uint8_t H2 = 0xF2; //Header 2
static volatile uint8_t S1 = 0xE3; //Stop 1
static volatile uint8_t S2 = 0xE4; //Stop 2
static volatile uint16_t ADC_values[3];
static volatile uint8_t Buffer[BuffSize];
static volatile uint8_t PosX = 0;
static volatile uint8_t PosY = 0;
static volatile uint8_t Mode = 0;
static volatile uint8_t Receive_Buffer[R_BuffSize];
static __IO uint32_t Ticks;
static __IO uint8_t flag_do_command = 0;
static __IO int32_t R_count = 0;
static __IO int32_t received_cmd = 0;
static __IO int32_t received_speed = 0;
static __IO int32_t received_steer = 0;
static uint16_t ticks_10s = 0;
static volatile uint32_t lost_packets = 0;
static volatile uint32_t lost_packets_per_sec = 0;
static volatile uint32_t lost_packets_count = 0;
static volatile uint32_t lost_packets_stats_count = 0;
static volatile uint32_t sent_packets_count = 0;
static volatile uint32_t received_packets_count = 0;
//Checks if the message is correct. Start and stop bytes also CRC.
uint8_t checkMessage(unsigned char *data_p, unsigned short length);
/////////////////////////////////////////////////////////
