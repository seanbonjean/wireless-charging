#include "headfile.h"
#include "user.h"

//灯珠引脚定义
#define L1 P13_0
#define L2 P13_1
#define L3 P13_2

#define LA P11_2
#define LB P11_3
#define LC P11_6
#define LD P11_9
#define LE P11_10

//电压监测引脚定义
#define IND_PIN ADC0_CH0_A0
#define CAP_PIN ADC0_CH7_A7
#define BAT_PIN ADC0_CH8_A8

#pragma section all "cpu0_dsram"

boolean nowCharging = TRUE; //充电标识符

float ind_voltage = 0; //电感电压
float cap_voltage = 0; //电容组电压
float bat_voltage = 0; //电池电压

uint8 LED_pointer = 0; //LED指针

#pragma section all restore

extern int16 overall_speed; //全局速度

void wirelessCharge_init (void)
{
    pit_interrupt_ms(CCU6_1, PIT_CH1, 1);    //CCU6定时器初始化

    //ADC初始化
    adc_init(ADC_0, ADC0_CH0_A0);
    adc_init(ADC_0, ADC0_CH1_A1);

    //LED引脚初始化
    gpio_init(L1, GPO, 1, PUSHPULL);
    gpio_init(L2, GPO, 1, PUSHPULL);
    gpio_init(L3, GPO, 1, PUSHPULL);
    gpio_init(LA, GPO, 0, PUSHPULL);
    gpio_init(LB, GPO, 0, PUSHPULL);
    gpio_init(LC, GPO, 0, PUSHPULL);
    gpio_init(LD, GPO, 0, PUSHPULL);
    gpio_init(LE, GPO, 0, PUSHPULL);
}

#pragma section all "cpu0_psram"

void wirelessCharge (void)
{
    //读取电压
    ind_voltage = adc_mean_filter(ADC_0, IND_PIN, ADC_8BIT, 10) * 3300 / 256;
    cap_voltage = (adc_mean_filter(ADC_0, CAP_PIN, ADC_8BIT, 10) * 3300 / 256) * 3;
    bat_voltage = (adc_mean_filter(ADC_0, BAT_PIN, ADC_8BIT, 10) * 3300 / 256) * 3;

    if (bat_voltage < BAT_MIN_VOT)
    {
        gpio_set(L1, 1);
        gpio_set(L2, 1);
        gpio_set(L3, 1);
        systick_delay_us(STM1, 1);
        gpio_set(LA, 0);
        gpio_set(LB, 0);
        gpio_set(LC, 0);
        gpio_set(LD, 0);
        gpio_set(LE, 0);

        while (TRUE)
        {
            systick_delay_ms(STM1, 500);
            gpio_toggle(L1);
            gpio_toggle(L2);
            gpio_toggle(L3);
            systick_delay_us(STM1, 1);
            gpio_toggle(LA);
            gpio_toggle(LB);
            gpio_toggle(LC);
            gpio_toggle(LD);
            gpio_toggle(LE);
        }
    }

    //充满电时
    if (nowCharging && cap_voltage > CAP_MAX_VOT)
    {
        overall_speed = SPEED_NORM;    //走起
        nowCharging = FALSE;
    }

    //遇到充电桩时
    if (!nowCharging && ind_voltage > IND_TRIG_VOT)
    {
        overall_speed = 0;    //停下
        nowCharging = TRUE;
    }

    //LED指针遍历控制
    LED_pointer++;
    if (LED_pointer > 14)
        LED_pointer = 0;

    //消隐
    gpio_set(L1, 1);
    gpio_set(L2, 1);
    gpio_set(L3, 1);
    systick_delay_us(STM1, 1);
    gpio_set(LA, 0);
    gpio_set(LB, 0);
    gpio_set(LC, 0);
    gpio_set(LD, 0);
    gpio_set(LE, 0);

    //点灯逻辑
    if (LED_pointer < (uint8) cap_voltage)    //只点亮低于当前电量的灯
    {
        //计算行
        switch (LED_pointer / 3)
        {
            //行高电平
            case 0 :
                gpio_set(LA, 1);
                break;
            case 1 :
                gpio_set(LB, 1);
                break;
            case 2 :
                gpio_set(LC, 1);
                break;
            case 3 :
                gpio_set(LD, 1);
                break;
            case 4 :
                gpio_set(LE, 1);
                break;
            default :
                break;
        }

        //计算列
        switch (LED_pointer % 3)
        {
            //列低电平
            case 0 :
                gpio_set(L1, 0);
                break;
            case 1 :
                gpio_set(L2, 0);
                break;
            case 2 :
                gpio_set(L3, 0);
                break;
            default :
                break;
        }
    }
}

#pragma section all restore
