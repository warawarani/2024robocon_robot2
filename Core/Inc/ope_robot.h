#ifndef OPE_ROBOT
#define OPE_ROBOT
#include "main.h"
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim17;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
typedef struct
{
    uint8_t buttonSW_1;
    uint8_t buttonSW_2;
    uint8_t buttonSW_3;
    uint8_t buttonSW_4;
    uint8_t toggleSW;
    uint16_t Vartical;
    uint16_t Horizontal;
} inputState;
#define ROBOT2_1
// #define ROBOT2_2
// #define ROBOT2_3
// #define ROBOT2_4

#endif