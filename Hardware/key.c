#include "key.h"

uint8_t Key_Val, Key_Down, Key_Old, Key_Up;

static uint8_t Key_GetNum(void)
{
    if (!HAL_GPIO_ReadPin(Key_GPIO_Port, Key_Pin)) return 1;
    return 0;
}

void Key_Task(void)
{
    Key_Val   = Key_GetNum();
    Key_Down  =  Key_Val       & (Key_Old ^ Key_Val);
    Key_Up    = ~Key_Val       & (Key_Old ^ Key_Val);
    Key_Old   =  Key_Val;

    if (Key_Down)
    {

    }
}
