#pragma once
#include "crc.h"
//Transmitted data count
#define BuffSize 9
#define headBodySize 5
#define R_BuffSize 128
//Header 1
static uint8_t H1 = 0xF1;
//Header 2
static uint8_t H2 = 0xF2;
//Stop 1
static uint8_t S1 = 0xE3;
//Stop 2
static uint8_t S2 = 0xE4;
//Received packets from USART buffer. May not be correct.
static uint32_t receivedPackets = 0;
//Received faulty packets. Headers or CRC does not match.
static uint32_t faultyPackets = 0;
//Received packets per 1s
static uint16_t packetsPerSecond = 0;
volatile uint8_t Receive_Buffer[R_BuffSize];
volatile static uint8_t *arr_buffer;
static __IO uint32_t Ticks;
static __IO uint8_t Uart_it = 0;
static __IO uint8_t flag_do_command = 0;
static __IO int32_t R_count = 0;
static __IO int32_t received_cmd = 0;
static __IO int32_t received_speed = 0;
static __IO int32_t received_steer = 0;
uint8_t logic1 = 0;
uint8_t logic2 = 0;
uint16_t ticks_10s = 0;
volatile uint32_t lost_packets = 0;
volatile uint32_t lost_packets_per_sec = 0;
volatile uint32_t lost_packets_count = 0;
volatile uint32_t lost_packets_stats_count = 0;
volatile uint32_t sent_packets_count = 0;
volatile uint32_t received_packets_count = 0;
//Checks if the message is correct. Start and stop bytes also CRC.
uint8_t checkMessage(unsigned char *data_p, unsigned short length);
/////////////////////////////////////////////////////////
