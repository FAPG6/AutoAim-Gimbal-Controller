#ifndef __MOTOR_H__
#define __MOTOR_H__



#include "system.h"


extern uint8_t Mode_Motor;

void motorTask(void* param);
void canTask(void* param);
void keyTask(void* param);

#endif
