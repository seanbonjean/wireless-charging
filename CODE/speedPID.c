/*
 * 后轮电机速度控制
 * 采用增量式PID
 */
#include "headfile.h"
#include "user.h"

//0号电机
#define M0_DIR P02_4//电机方向引脚
#define M0_PWM ATOM0_CH5_P02_5//电机PWM引脚
#define M0_nFAULT P02_6//电机过流保护引脚

#define M0_ENC_LSB GPT12_T4INA_P02_8//编码器计数引脚
#define M0_ENC_DIR GPT12_T4EUDA_P00_9//编码器方向引脚

//1号电机
#define M1_DIR P21_2//电机方向引脚
#define M1_PWM ATOM0_CH1_P21_3//电机PWM引脚
#define M1_nFAULT P21_4//电机过流保护引脚

#define M1_ENC_LSB GPT12_T2INB_P33_7//编码器计数引脚
#define M1_ENC_DIR GPT12_T2EUDB_P33_6//编码器方向引脚

#pragma section all "cpu1_dsram"

//0号电机
float M0_Kp, M0_Ki, M0_Kd; //增量式PID参数
int16 M0_delta_u; //增量式PID输出增量
int16 M0_u; //输出量
int16 M0_real_speed; //实际速度
int16 M0_set_speed = 0; //期望速度
int16 M0_ek = 0, M0_ek1 = 0, M0_ek2 = 0; //前后三次误差
int32 M0_mot_duty; //电机占空比

//1号电机
float M1_Kp, M1_Ki, M1_Kd; //增量式PID参数
int16 M1_delta_u; //增量式PID输出增量
int16 M1_u; //输出量
int16 M1_real_speed; //实际速度
int16 M1_set_speed = 0; //期望速度
int16 M1_ek = 0, M1_ek1 = 0, M1_ek2 = 0; //前后三次误差
int32 M1_mot_duty; //电机占空比

#pragma section all restore

//电机速度控制初始化
void speedPID_init (void)
{
    pit_interrupt_ms(CCU6_1, PIT_CH0, 5);    //CCU6定时器初始化
    //电机初始化
    //0号电机
    gpio_init(M0_DIR, GPO, 0, PUSHPULL);    //电机方向引脚初始化
    gtm_pwm_init(M0_PWM, 17000, 0);  //电机PWM引脚初始化
    gpio_init(M0_nFAULT, GPI, 0, PULLUP);    //电机过流保护引脚初始化

    gpt12_init(GPT12_T4, M0_ENC_LSB, M0_ENC_DIR);    //编码器初始化

    //1号电机
    gpio_init(M1_DIR, GPO, 0, PUSHPULL);    //电机方向引脚初始化
    gtm_pwm_init(M1_PWM, 17000, 0);  //电机PWM引脚初始化
    gpio_init(M1_nFAULT, GPI, 0, PULLUP);    //电机过流保护引脚初始化

    gpt12_init(GPT12_T2, M1_ENC_LSB, M1_ENC_DIR);    //编码器初始化

    //设置PID系数
    //0号电机
    M0_Kp = M0_KP;    //proportional
    M0_Ki = M0_KI;    //integral
    M0_Kd = M0_KD;    //derivative

    //1号电机
    M1_Kp = M1_KP;    //proportional
    M1_Ki = M1_KI;    //integral
    M1_Kd = M1_KD;    //derivative
}

#pragma section all "cpu1_psram"

void speedPID (void)
{
    //0号电机
    M0_real_speed = gpt12_get(GPT12_T4);    //获取电机速度
    gpt12_clear(GPT12_T4);    //重置编码器

    //1号电机
    M1_real_speed = gpt12_get(GPT12_T2);    //获取电机速度
    gpt12_clear(GPT12_T2);    //重置编码器

    //0号电机
    M0_ek2 = M0_ek1;    //更新历史误差
    M0_ek1 = M0_ek;
    M0_ek = M0_set_speed - M0_real_speed;    //计算error term

    //1号电机
    M1_ek2 = M1_ek1;    //更新历史误差
    M1_ek1 = M1_ek;
    M1_ek = M1_set_speed - M1_real_speed;    //计算error term

    //增量式PID
    //0号电机
    M0_delta_u = (int16) (M0_Kp * (M0_ek - M0_ek1) + M0_Ki * M0_ek + M0_Kd * (M0_ek - 2 * M0_ek1 + M0_ek2));  //计算增量
    M0_u += M0_delta_u;    //累加入增量
    M0_u = (int16) limit(M0_u, GTM_ATOM0_PWM_DUTY_MAX);    //输出限幅，防止输出超过100%占空比

    M0_mot_duty = (int32) M0_u;    //输出给电机占空比

    //1号电机
    M1_delta_u = (int16) (M1_Kp * (M1_ek - M1_ek1) + M1_Ki * M1_ek + M1_Kd * (M1_ek - 2 * M1_ek1 + M1_ek2));  //计算增量
    M1_u += M1_delta_u;    //累加入增量
    M1_u = (int16) limit(M1_u, GTM_ATOM0_PWM_DUTY_MAX);    //输出限幅，防止输出超过100%占空比

    M1_mot_duty = (int32) M1_u;    //输出给电机占空比

    //将电机占空比转换为DRV8701E芯片逻辑
    //0号电机
    if (0 <= M0_mot_duty)    //若电机占空比>=0
    {
        gpio_set(M0_DIR, 0);    //低电平正转
        pwm_duty(M0_PWM, M0_mot_duty);    //设置电机占空比
    }
    else
    {
        gpio_set(M0_DIR, 1);    //高电平反转
        pwm_duty(M0_PWM, -M0_mot_duty);    //设置电机占空比
    }

    //1号电机
    if (0 <= M1_mot_duty)    //若电机占空比>=0
    {
        gpio_set(M1_DIR, 0);    //低电平正转
        pwm_duty(M1_PWM, M1_mot_duty);    //设置电机占空比
    }
    else
    {
        gpio_set(M1_DIR, 1);    //高电平反转
        pwm_duty(M1_PWM, -M1_mot_duty);    //设置电机占空比
    }

    //printf("$%d %d;", M0_real_speed, M0_set_speed);
}

#pragma section all restore
