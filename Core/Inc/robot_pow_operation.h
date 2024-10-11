#ifndef RO_P_OPE
#define RO_P_OPE

#include "ope_robot.h"
#include <math.h>

void powerConverter(TIM_HandleTypeDef *htimx, int pin, int pow);
void MoterPowInit();
void WheelPowControl(double Horizontal, double Vartical);
void IndividualOpelation(inputState *Data);

#endif
