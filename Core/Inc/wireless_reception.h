
#ifndef WIRELESS_RECEP
#define WIRELESS_RECEP
#include "ope_robot.h"
#define RX_LENGTH 4
extern uint8_t U1RXbuffer;
extern uint8_t controlerVarBuffer[RX_LENGTH];
extern uint8_t controlerFlag;
void uart_reception();
void DecodeControlerVarBuffer(uint8_t *controlerBuffer, inputState *Data);
#endif