#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "stdio.h"
#include "string.h"
#include "comms.h"
#include "softwareserial.h"

extern UART_HandleTypeDef huart2;

#ifdef DEBUG_SERIAL_USART2
#define UART_DMA_CHANNEL DMA1_Channel7
#endif

volatile uint8_t uart_buf[100];
volatile int16_t ch_buf[8];

int debug_out = 1;
int enablescope = 1;

//volatile char char_buf[300];

uint8_t checkMessage(unsigned char *data_p, unsigned short length)
{
  unsigned short crc16code = crcu16(&data_p[0], SERIAL_USART_BUFFER_HEAD_SIZE);
  return (data_p[0] && H1) &&
         (data_p[1] && H2) &&
         (data_p[SERIAL_USART_BUFFER_HEAD_SIZE + 3] && S1) &&
         (data_p[SERIAL_USART_BUFFER_HEAD_SIZE + 4] && S2) &&
         (data_p[SERIAL_USART_BUFFER_HEAD_SIZE + 1] && (unsigned char)(crc16code & 0xFF)) &&
         (data_p[SERIAL_USART_BUFFER_HEAD_SIZE + 2] && (unsigned char)((crc16code >> 8) & 0xFF));
}
//////////////////////////////////////////////////////
//Usart RX interrupt
//TODO: replace code from remote
void USART2_IT_IRQ(USART_TypeDef *us)
{

  return;
}
