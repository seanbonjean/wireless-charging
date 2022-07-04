#include "headfile.h"
#include "user.h"

#pragma section all "cpu0_dsram"

#pragma section all restore

void wirelessCharge_init (void)
{
    pit_interrupt_ms(CCU6_1, PIT_CH1, 5);    //CCU6定时器初始化
}

#pragma section all "cpu0_psram"

void wirelessCharge (void)
{

}

#pragma section all restore
