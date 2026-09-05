#ifndef __LED_H
#define __LED_H

#include "main.h"

void Led_Task(void);
void Led_Init(void);    /* 创建 LED 任务，须在 scheduler_init() 之后调用 */

#endif
