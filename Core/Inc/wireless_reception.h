
#ifndef WIRELESS_RECEP
#define WIRELESS_RECEP
#include "ope_robot.h"
#define RX_LENGTH 4
uint8_t U1RXbuffer;
uint8_t controlerVarBuffer[RX_LENGTH];
uint8_t controlerFlag;
void uart_reception();

#endif