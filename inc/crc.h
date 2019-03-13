#pragma once
#include "stm32f1xx_hal.h"
//paketo ilgis - 
//du baitus 0xF1 ir 0xF2 tegul jie buna markeris srauto pradzios
//du baitai headeriui
//24 baitai etanolio matavimam
//crc - 16 bitu (2 baitai) (susirast internete)

/*

* 16 12 5
* this is the CCITT CRC 16 polynomial X + X + X + 1.
* This is 0x1021 when x is 2, but the way the algorithm works
* we use 0x8408 (the reverse of the bit pattern). The high
* bit is always assumed to be set, thus we only use 16 bits to
* represent the 17 bit value.
//*/
#define POLY 0x8408 /* 1021H bit reversed*/
unsigned short crcu16(unsigned char *data_p, unsigned short length);