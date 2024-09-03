#include "wireless_reception.h"

static uint8_t con_cnt = 0;

/**
 * @brief receive valve
 *
 */
void uart_reception()
{
    controlerVarBuffer[con_cnt] = U1RXbuffer;
    if (controlerVarBuffer[0] == 0x80)
    {
        con_cnt++;
    }
    if (con_cnt == RX_LENGTH)
    {
        con_cnt = 0;
        controlerFlag = 1;
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    }
}
/**
 * @brief decode the received value
 *
 * @param controlerVarBuffer
 * @param Data
 */
void DecodeControlerVarBuffer(uint8_t *controlerVarBuffer, inputState *Data)
{
    Data->Horizontal = controlerVarBuffer[1];
    Data->Vartical = controlerVarBuffer[2];
    if (controlerVarBuffer[3] & (1 << 0))
    {
        Data->buttonSW_1 = 1;
    }
    else
    {
        Data->buttonSW_1 = 0;
    }
    if (controlerVarBuffer[3] & (1 << 1))
    {
        Data->buttonSW_2 = 1;
    }
    else
    {
        Data->buttonSW_2 = 0;
    }
    if (controlerVarBuffer[3] & (1 << 2))
    {
        Data->buttonSW_3 = 1;
    }
    else
    {
        Data->buttonSW_3 = 0;
    }
    if (controlerVarBuffer[3] & (1 << 3))
    {
        Data->buttonSW_4 = 1;
    }
    else
    {
        Data->buttonSW_4 = 0;
    }
    if (controlerVarBuffer[3] & (1 << 4))
    {
        Data->toggleSW = 1;
    }
    else
    {
        Data->toggleSW = 0;
    }
}