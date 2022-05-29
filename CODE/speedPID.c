/*
 * 后轮电机速度控制
 * 采用增量式PID
 */
#include "headfile.h"
#include "user.h"

#define REAR_MOT_DIR P02_4//电机方向引脚
#define REAR_MOT_PWM ATOM0_CH5_P02_5//电机PWM引脚
#define REAR_MOT_nFAULT P02_6//电机过流保护引脚

#define REAR_ENC_LSB GPT12_T4INA_P02_8//编码器计数引脚
#define REAR_ENC_DIR GPT12_T4EUDA_P00_9//编码器方向引脚

#pragma section all "cpu1_dsram"

float re_Kp, re_Ki, re_Kd; //增量式PID参数
int16 re_delta_u; //增量式PID输出增量
int16 re_u; //输出量
int16 re_real_speed; //实际速度
int16 re_set_speed; //期望速度
int16 re_ek = 0, re_ek1, re_ek2; //前后三次误差
int32 re_mot_duty; //电机占空比

#pragma section all restore

//电机速度控制初始化
void speedPID_init (void)
{
    //电机初始化
    gpio_init(REAR_MOT_DIR, GPO, 0, PUSHPULL);    //电机方向引脚初始化
    gtm_pwm_init(REAR_MOT_PWM, 17000, 0);  //电机PWM引脚初始化
    gpio_init(REAR_MOT_nFAULT, GPI, 0, PULLUP);    //电机过流保护引脚初始化

    gpt12_init(GPT12_T4, REAR_ENC_LSB, REAR_ENC_DIR);    //编码器初始化
    pit_interrupt_ms(CCU6_1, PIT_CH0, 5);    //CCU6定时器初始化

    //设置PID系数
    re_Kp = REAR_KP;    //proportional
    re_Ki = REAR_KI;    //integral
    re_Kd = REAR_KD;    //derivative
}

#pragma section all "cpu1_psram"

void speedPID (void)
{
    re_real_speed = gpt12_get(GPT12_T4);    //获取电机速度
    gpt12_clear(GPT12_T4);    //重置编码器

    re_ek2 = re_ek1;    //更新历史误差
    re_ek1 = re_ek;
    re_ek = re_set_speed - re_real_speed;    //计算error term

    //增量式PID
    re_delta_u = (int16) (re_Kp * (re_ek - re_ek1) + re_Ki * re_ek + re_Kd * (re_ek - 2 * re_ek1 + re_ek2));  //计算增量
    re_u += re_delta_u;    //累加入增量
    re_u = (int16) limit(re_u, GTM_ATOM0_PWM_DUTY_MAX);    //输出限幅，防止输出超过100%占空比

    re_mot_duty = (int32) re_u;    //输出给电机占空比

    //将电机占空比转换为DRV8701E芯片逻辑
    if (0 <= re_mot_duty)    //若电机占空比>=0
    {
        gpio_set(REAR_MOT_DIR, 0);    //低电平正转
        pwm_duty(REAR_MOT_PWM, re_mot_duty);    //设置电机占空比
    }
    else
    {
        gpio_set(REAR_MOT_DIR, 1);    //高电平反转
        pwm_duty(REAR_MOT_PWM, -re_mot_duty);    //设置电机占空比
    }

    //printf("$%d %d;", re_real_speed, re_set_speed);
}

#pragma section all restore
