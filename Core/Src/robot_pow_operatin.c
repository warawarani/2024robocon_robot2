#include "robot_pow_operation.h"
void powerConverter(TIM_HandleTypeDef *htimx, int pin, int pow)
{
    uint16_t pwm, dir;
    pwm = (pow <= 500) ? (1000 - pow * 2) : pow * 2 - 1000;
    dir = (pow <= 500) ? 0 : 1;
    if (htimx == &htim2)
    {
        if (pin == TIM_CHANNEL_1)
        {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
            HAL_GPIO_WritePin(MOTER1_DIR_GPIO_Port, MOTER1_DIR_Pin, dir);
        }
    }
    if (htimx == &htim3)
    {
        if (pin == TIM_CHANNEL_2)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
            HAL_GPIO_WritePin(MOTER2_DIR_GPIO_Port, MOTER2_DIR_Pin, dir);
        }

        if (pin == TIM_CHANNEL_3)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm);
            HAL_GPIO_WritePin(MOTER3_DIR_GPIO_Port, MOTER3_DIR_Pin, dir);
        }
        if (pin == TIM_CHANNEL_4)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm);
            HAL_GPIO_WritePin(MOTER4_DIR_GPIO_Port, MOTER4_DIR_Pin, dir);
        }
    }
    if(htimx == &htim17){
        __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, pwm);
            HAL_GPIO_WritePin(MOTER5_DIR_GPIO_Port, MOTER5_DIR_Pin, dir);
    }
}

/**
 * @brief Control functions for differential two-wheel
 * @retval None
 *
 * @param Horizontal Horizontal axis value of stick
 * @param vertical Vertical axis value of stick
 */
void WheelPowControl(double Horizontal, double Vartical)
{
    const int STICK_CENTER_POSITION = 0x40;
    double leftWheelPow;
    double rightWheelPow;
    double radian;
    Horizontal -= (double)STICK_CENTER_POSITION;
    Vartical -= (double)STICK_CENTER_POSITION;
    radian = atan2(Horizontal, Vartical);
    double powerGain = (hypot(Vartical, Horizontal) / (2 * STICK_CENTER_POSITION));
    powerGain = (powerGain >= 1) ? 1 : powerGain;
    rightWheelPow = 500 - ((powerGain * 500) * 2 * sin(radian - M_3PI_4));
    leftWheelPow = 500 - ((powerGain * 500) * 2 * sin(radian + M_3PI_4));
    rightWheelPow = (rightWheelPow < 0) ? 0 : (rightWheelPow > 1000) ? 1000
                                                                    : rightWheelPow;
    leftWheelPow = (leftWheelPow < 0) ? 0 : (leftWheelPow > 1000) ? 1000
                                                                    : leftWheelPow;
    powerConverter(&htim2, TIM_CHANNEL_1, rightWheelPow);
    powerConverter(&htim3, TIM_CHANNEL_2, leftWheelPow);
}

#ifdef ROBOT2_1
/**
 * @brief This fanction is proglam for the robot2-X
 * @note for the robot2-1 (collecting the box)
 * @retval None
 *
 * @param Data
 */
void IndividualOpelation(inputState *Data)
{
    static uint16_t powerA = 500; // for locking mechanism
    static uint16_t powerB = 500; // for collection arm
    static uint16_t powerC = 500; // for vacuume pump
    // int limSwState1 = HAL_GPIO_ReadPin(LimitSW1_GPIO_Port, LimitSW1_Pin);
    static uint8_t swState1 = 0, lastSwState1 = 0;
    static uint8_t swState2 = 0, lastSwState2 = 0;

    /* for locking mechanism */
    if (Data->buttonSW_4 != lastSwState1)
    {
        if (Data->buttonSW_4 == 1)
        {
            swState1 = !swState1;
        }
        lastSwState1 = Data->buttonSW_4;
    }
    if (!swState1)
    {
        powerA = 500;
    }
    else if (swState1)
    {
        powerA = 0;
    }
    else
    {
        powerA = 500;
    }

    /* for collection arm */
    if (Data->buttonSW_1 != Data->buttonSW_2)
    {
        if (Data->buttonSW_1)
        {
            powerB = 825;
        }
        else if (Data->buttonSW_2)
        {
            powerB = 175;
        }
        else
        {
            powerB = 500;
        }
    }
    else
    {
        powerB = 500;
    }

    /* for vacuume pump */
    if (Data->buttonSW_3 != lastSwState2)
    {
        if (Data->buttonSW_3)
        {
            swState2 = !swState2;
        }
        lastSwState2 = Data->buttonSW_3;
    }
    if (swState2)
    {
        powerC = 1000;
    }
    else
    {
        powerC = 500;
    }

    /* set duty ratio */
    powerConverter(&htim3, TIM_CHANNEL_3, powerA);
    powerConverter(&htim3, TIM_CHANNEL_4, powerB);
    powerConverter(&htim17, TIM_CHANNEL_1, powerC);
}
#endif /*ROBOT2_1*/

#ifdef ROBOT2_2
/**
 * @brief This fanction is proglam for the robot2-X
 * @note for the robot2-2 (collecting the ball)
 * @retval none
 *
 * @param Data
 */
void IndividualOpelation(inputState *Data)
{
    static uint16_t powerA = 500; // for roller
    static uint16_t powerB = 500; // for arm
    static uint8_t swState1 = 0, lastSwState1 = 0;

    /* for roller */
    if (Data->buttonSW_3 != lastSwState1)
    {
        if (Data->buttonSW_3==1)
        {
            swState1 = !swState1;
        }
        lastSwState1 = Data->buttonSW_3;
    }

    if (swState1)
    {
        if (powerA >= 990)
        {
            powerA = 1000;
        }
        else
        {
            powerA += 2;
        }
    }
    else
    {
        if (powerA <= 510)
        {
            powerA = 500;
        }
        else
        {
            powerA -= 2;
        }
    }

    if (Data->buttonSW_1 != Data->buttonSW_2)
    {
        if (Data->buttonSW_1)
        {
            powerB = 300;
        }
        else if (Data->buttonSW_2)
        {
            powerB = 700;
        }
    }
    else
    {
        powerB = 500;
    }

    powerConverter(&htim3, TIM_CHANNEL_3, powerA);
    powerConverter(&htim3, TIM_CHANNEL_4, powerB);
}
#endif /*ROBOT2_2*/

#ifdef ROBOT2_3
/**
 * @brief This fanction is proglam for the robot2-X
 * @note for the robot2-3 (landing only)
 * @retval None
 * @param Data
 */
void IndividualOpelation(inputState *Data)
{
    static uint16_t powerA = 500;
    if (Data->buttonSW_1 != Data->buttonSW_2)
    {
        if (Data->buttonSW_1)
        {
            powerA = 0;
        }
        if (Data->buttonSW_2)
        {
            powerA = 1000;
        }
    }
    else
    {
        powerA = 500;
    }

    powerConverter(&htim3, TIM_CHANNEL_4, powerA);
}
#endif /*ROBOT2_3*/

#ifdef ROBOT2_4
/**
 * @brief This fanction is proglam for the robot2-X
 * @note for the robot2-4 (not use this function)
 * @retval None
 * @param Data
 */
void IndividualOpelation(inputState *Data)
{
    static uint16_t powerA = 500;
    static uint16_t powerB = 500;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, powerA);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, powerB);
}
#endif /*ROBOT2_4*/