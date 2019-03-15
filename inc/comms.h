#pragma once
#include "crc.h"
#define SERIAL_USART_BUFFER_SIZE 9
#define SERIAL_USART_BUFFER_HEAD_SIZE 5 // Header + data bytes
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

//Checks if the message is correct. Start and stop bytes also CRC.
uint8_t checkMessage(unsigned char *data_p, unsigned short length);
/////////////////////////////////////////////////////////
