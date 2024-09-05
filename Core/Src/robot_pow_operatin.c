#include "robot_pow_operation.h"
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
    double powerGain = (hypot(Vartical, Horizontal) / (2 * STICK_CENTER_POSITION - 1));
    powerGain = (powerGain >= 1) ? 1 : powerGain;
    rightWheelPow = 500 + ((powerGain * 500) * sin(radian - M_3PI_4));
    leftWheelPow = 500 + ((powerGain * 500) * sin(radian + M_3PI_4));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, rightWheelPow);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, leftWheelPow);
}

#ifdef ROBOT2_1_A
/**
 * @brief This fanction is proglam for the robot2-X
 * @note for the robot2-1 (collecting the box)
 * @retval None
 *
 * @param Data
 */
void IndividualOpelation(inputState *Data)
{
    static uint16_t powerB = 500; // for collection arm
    static uint16_t powerC = 500; // for vacuume pump
    uint32_t encoderVal = TIM1->CNT; //(0~2000)
    static uint8_t swState2, lastSwState2;

    /* for collection arm */
    if (Data->buttonSW_1 != Data->buttonSW_2)
    {
        if ((Data->buttonSW_1 && encoderVal <= 1000) || encoderVal >= 1900)
        {
            powerB = 800;
        }
        else if (Data->buttonSW_2 && encoderVal >= 50)
        {
            powerB = 200;
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
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, powerB);
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, powerC);
}
#endif /*ROBOT2_1*/

#ifdef ROBOT2_1_U
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
    int limSwState1 = HAL_GPIO_ReadPin(LimitSW1_GPIO_Port, LimitSW1_Pin);
    int limSwState2 = HAL_GPIO_ReadPin(LimitSW2_GPIO_Port, LimitSW2_Pin);
    static uint8_t swState1,lastSwState1;

    /* for locking mechanism */
    if (Data->buttonSW_4 != lastSwState1)
    {
        if (Data->buttonSW_4)
        {
            swState1 = !swState1;
        }
        lastSwState1 = Data->buttonSW_4;
    }
    if (swState1 && !limSwState1)
    {
        powerA = 0;
    }
    else if (!swState1 && !limSwState2)
    {
        powerA = 1000;
    }
    else
    {
        powerA = 500;
    }


    /* set duty ratio */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, powerA);
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
    static uint16_t powerA = 500; // for belt
    static uint16_t powerB = 500; // for roller

    if (Data->buttonSW_1 != Data->buttonSW_2)
    {
        if (Data->buttonSW_1)
        {
            powerA = 400;
            powerB = 350;
        }
        else if (Data->buttonSW_2)
        {
            powerA = 600;
            powerB = 650;
        }
        else if (Data->buttonSW_3)
        {
            powerA = 0;
            powerB = 350;
        }
    }
    else
    {
        powerA = 500;
        powerB = 500;
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, powerA);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, powerB);
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

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, powerA);
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