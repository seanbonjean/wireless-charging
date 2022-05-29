/*
 * ���ת�����
 * ����λ��ʽPD
 */
#include "headfile.h"
#include "user.h"

#define SERVO_CENTER 1.5/20*GTM_ATOM0_PWM_DUTY_MAX//�������ռ�ձ�
#define SERVO_PIN ATOM1_CH1_P33_9//�������

#pragma section all "cpu1_dsram"

float se_Kp, se_Kd; //λ��ʽPD����
int16 se_u; //�����
int16 se_ek = 0, se_ek1; //�������&��ʷ���
int16 se_real_pos; //ʵ��λ��ƫ��
int16 se_set_pos; //����λ��ƫ��
int32 se_servo_duty; //���ռ�ձ�

#pragma section all restore

extern int16 pos; //����ͷλ��ƫ��

void servoPID_init (void)
{
    gtm_pwm_init(SERVO_PIN, 50, SERVO_CENTER); //�����ʼ��������
    pit_interrupt_ms(CCU6_1, PIT_CH1, 5);    //CCU6��ʱ����ʼ��

    se_set_pos = 0; //����λ��ƫ��Ϊ0�����ϸ�������������ʻ

    //����PDϵ��
    se_Kp = SERVO_KP;    //proportional
    se_Kd = SERVO_KD;    //derivative
}

#pragma section all "cpu1_psram"

void servoPID (void)
{
    se_real_pos = pos;    //��ȡ����ͷλ��ƫ��

    se_ek1 = se_ek;    //������ʷ���
    se_ek = se_set_pos - se_real_pos;    //����error term

    se_u = (int16) (se_Kp * se_ek + se_Kd * (se_ek - se_ek1));    //�������ռ�ձ�
    se_u = (int16) limit(se_u, SERVO_LIMIT);    //����޷�

    se_servo_duty = (int32) se_u;    //������ռ�ձ�
    pwm_duty(SERVO_PIN, SERVO_CENTER + se_servo_duty);    //ִ��

    //printf("$%d %d;", se_real_pos, se_set_pos);
}

#pragma section all restore
