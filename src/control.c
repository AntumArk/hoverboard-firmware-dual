
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include <stdbool.h>
#include <string.h>

TIM_HandleTypeDef TimHandle;
uint32_t timeout = 100;
#define IN_RANGE(x, low, up) (((x) >= (low)) && ((x) <= (up)))

