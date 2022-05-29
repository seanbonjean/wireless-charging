/*
 * 舵机转向控制
 * 采用位置式PD
 */
#include "headfile.h"
#include "user.h"

#define SERVO_CENTER 1.5/20*GTM_ATOM0_PWM_DUTY_MAX//舵机居中占空比
#define SERVO_PIN ATOM1_CH1_P33_9//舵机引脚

#pragma section all "cpu1_dsram"

float se_Kp, se_Kd; //位置式PD参数
int16 se_u; //输出量
int16 se_ek = 0, se_ek1; //当次误差&历史误差
int16 se_real_pos; //实际位置偏差
int16 se_set_pos; //期望位置偏差
int32 se_servo_duty; //舵机占空比

#pragma section all restore

extern int16 pos; //摄像头位置偏差

void servoPID_init (void)
{
    gtm_pwm_init(SERVO_PIN, 50, SERVO_CENTER); //舵机初始化并居中
    pit_interrupt_ms(CCU6_1, PIT_CH1, 5);    //CCU6定时器初始化

    se_set_pos = 0; //期望位置偏差为0，即严格按照赛道中线行驶

    //设置PD系数
    se_Kp = SERVO_KP;    //proportional
    se_Kd = SERVO_KD;    //derivative
}

#pragma section all "cpu1_psram"

void servoPID (void)
{
    se_real_pos = pos;    //读取摄像头位置偏差

    se_ek1 = se_ek;    //更新历史误差
    se_ek = se_set_pos - se_real_pos;    //计算error term

    se_u = (int16) (se_Kp * se_ek + se_Kd * (se_ek - se_ek1));    //计算输出占空比
    se_u = (int16) limit(se_u, SERVO_LIMIT);    //输出限幅

    se_servo_duty = (int32) se_u;    //输出舵机占空比
    pwm_duty(SERVO_PIN, SERVO_CENTER + se_servo_duty);    //执行

    //printf("$%d %d;", se_real_pos, se_set_pos);
}

#pragma section all restore
