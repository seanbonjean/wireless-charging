#include "headfile.h"
#include "user.h"

#pragma section all "cpu0_dsram"

#pragma section all restore

void wirelessCharge_init (void)
{
    pit_interrupt_ms(CCU6_1, PIT_CH1, 5);    //CCU6��ʱ����ʼ��
}

#pragma section all "cpu0_psram"

void wirelessCharge (void)
{

}

#pragma section all restore
