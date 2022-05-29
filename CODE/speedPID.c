/*
 * ���ֵ���ٶȿ���
 * ��������ʽPID
 */
#include "headfile.h"
#include "user.h"

#define REAR_MOT_DIR P02_4//�����������
#define REAR_MOT_PWM ATOM0_CH5_P02_5//���PWM����
#define REAR_MOT_nFAULT P02_6//���������������

#define REAR_ENC_LSB GPT12_T4INA_P02_8//��������������
#define REAR_ENC_DIR GPT12_T4EUDA_P00_9//��������������

#pragma section all "cpu1_dsram"

float re_Kp, re_Ki, re_Kd; //����ʽPID����
int16 re_delta_u; //����ʽPID�������
int16 re_u; //�����
int16 re_real_speed; //ʵ���ٶ�
int16 re_set_speed; //�����ٶ�
int16 re_ek = 0, re_ek1, re_ek2; //ǰ���������
int32 re_mot_duty; //���ռ�ձ�

#pragma section all restore

//����ٶȿ��Ƴ�ʼ��
void speedPID_init (void)
{
    //�����ʼ��
    gpio_init(REAR_MOT_DIR, GPO, 0, PUSHPULL);    //����������ų�ʼ��
    gtm_pwm_init(REAR_MOT_PWM, 17000, 0);  //���PWM���ų�ʼ��
    gpio_init(REAR_MOT_nFAULT, GPI, 0, PULLUP);    //��������������ų�ʼ��

    gpt12_init(GPT12_T4, REAR_ENC_LSB, REAR_ENC_DIR);    //��������ʼ��
    pit_interrupt_ms(CCU6_1, PIT_CH0, 5);    //CCU6��ʱ����ʼ��

    //����PIDϵ��
    re_Kp = REAR_KP;    //proportional
    re_Ki = REAR_KI;    //integral
    re_Kd = REAR_KD;    //derivative
}

#pragma section all "cpu1_psram"

void speedPID (void)
{
    re_real_speed = gpt12_get(GPT12_T4);    //��ȡ����ٶ�
    gpt12_clear(GPT12_T4);    //���ñ�����

    re_ek2 = re_ek1;    //������ʷ���
    re_ek1 = re_ek;
    re_ek = re_set_speed - re_real_speed;    //����error term

    //����ʽPID
    re_delta_u = (int16) (re_Kp * (re_ek - re_ek1) + re_Ki * re_ek + re_Kd * (re_ek - 2 * re_ek1 + re_ek2));  //��������
    re_u += re_delta_u;    //�ۼ�������
    re_u = (int16) limit(re_u, GTM_ATOM0_PWM_DUTY_MAX);    //����޷�����ֹ�������100%ռ�ձ�

    re_mot_duty = (int32) re_u;    //��������ռ�ձ�

    //�����ռ�ձ�ת��ΪDRV8701EоƬ�߼�
    if (0 <= re_mot_duty)    //�����ռ�ձ�>=0
    {
        gpio_set(REAR_MOT_DIR, 0);    //�͵�ƽ��ת
        pwm_duty(REAR_MOT_PWM, re_mot_duty);    //���õ��ռ�ձ�
    }
    else
    {
        gpio_set(REAR_MOT_DIR, 1);    //�ߵ�ƽ��ת
        pwm_duty(REAR_MOT_PWM, -re_mot_duty);    //���õ��ռ�ձ�
    }

    //printf("$%d %d;", re_real_speed, re_set_speed);
}

#pragma section all restore
