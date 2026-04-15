#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "stdlib.h"
#include "stdbool.h"


#include "FreeRTOS.h"
#include "task.h"


typedef struct redrPos{
	uint16_t xCenter;
	uint16_t yCenter;
	bool upDate;
	
}redrPos;

extern redrPos redr;



void hardwareInit(void);
void createTask(void);

void sysTask(void* param);
void print(const char* format, ...);



#endif


