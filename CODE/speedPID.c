/*
 * ���ֵ���ٶȿ���
 * ��������ʽPID
 */
#include "headfile.h"
#include "user.h"

//0�ŵ��
#define M0_DIR P02_4//�����������
#define M0_PWM ATOM0_CH5_P02_5//���PWM����
#define M0_nFAULT P02_6//���������������

#define M0_ENC_LSB GPT12_T4INA_P02_8//��������������
#define M0_ENC_DIR GPT12_T4EUDA_P00_9//��������������

//1�ŵ��
#define M1_DIR P21_2//�����������
#define M1_PWM ATOM0_CH1_P21_3//���PWM����
#define M1_nFAULT P21_4//���������������

#define M1_ENC_LSB GPT12_T2INB_P33_7//��������������
#define M1_ENC_DIR GPT12_T2EUDB_P33_6//��������������

#pragma section all "cpu1_dsram"

//0�ŵ��
float M0_Kp, M0_Ki, M0_Kd; //����ʽPID����
int16 M0_delta_u; //����ʽPID�������
int16 M0_u; //�����
int16 M0_real_speed; //ʵ���ٶ�
int16 M0_set_speed = 0; //�����ٶ�
int16 M0_ek = 0, M0_ek1 = 0, M0_ek2 = 0; //ǰ���������
int32 M0_mot_duty; //���ռ�ձ�

//1�ŵ��
float M1_Kp, M1_Ki, M1_Kd; //����ʽPID����
int16 M1_delta_u; //����ʽPID�������
int16 M1_u; //�����
int16 M1_real_speed; //ʵ���ٶ�
int16 M1_set_speed = 0; //�����ٶ�
int16 M1_ek = 0, M1_ek1 = 0, M1_ek2 = 0; //ǰ���������
int32 M1_mot_duty; //���ռ�ձ�

#pragma section all restore

//����ٶȿ��Ƴ�ʼ��
void speedPID_init (void)
{
    pit_interrupt_ms(CCU6_1, PIT_CH0, 5);    //CCU6��ʱ����ʼ��
    //�����ʼ��
    //0�ŵ��
    gpio_init(M0_DIR, GPO, 0, PUSHPULL);    //����������ų�ʼ��
    gtm_pwm_init(M0_PWM, 17000, 0);  //���PWM���ų�ʼ��
    gpio_init(M0_nFAULT, GPI, 0, PULLUP);    //��������������ų�ʼ��

    gpt12_init(GPT12_T4, M0_ENC_LSB, M0_ENC_DIR);    //��������ʼ��

    //1�ŵ��
    gpio_init(M1_DIR, GPO, 0, PUSHPULL);    //����������ų�ʼ��
    gtm_pwm_init(M1_PWM, 17000, 0);  //���PWM���ų�ʼ��
    gpio_init(M1_nFAULT, GPI, 0, PULLUP);    //��������������ų�ʼ��

    gpt12_init(GPT12_T2, M1_ENC_LSB, M1_ENC_DIR);    //��������ʼ��

    //����PIDϵ��
    //0�ŵ��
    M0_Kp = M0_KP;    //proportional
    M0_Ki = M0_KI;    //integral
    M0_Kd = M0_KD;    //derivative

    //1�ŵ��
    M1_Kp = M1_KP;    //proportional
    M1_Ki = M1_KI;    //integral
    M1_Kd = M1_KD;    //derivative
}

#pragma section all "cpu1_psram"

void speedPID (void)
{
    //0�ŵ��
    M0_real_speed = gpt12_get(GPT12_T4);    //��ȡ����ٶ�
    gpt12_clear(GPT12_T4);    //���ñ�����

    //1�ŵ��
    M1_real_speed = gpt12_get(GPT12_T2);    //��ȡ����ٶ�
    gpt12_clear(GPT12_T2);    //���ñ�����

    //0�ŵ��
    M0_ek2 = M0_ek1;    //������ʷ���
    M0_ek1 = M0_ek;
    M0_ek = M0_set_speed - M0_real_speed;    //����error term

    //1�ŵ��
    M1_ek2 = M1_ek1;    //������ʷ���
    M1_ek1 = M1_ek;
    M1_ek = M1_set_speed - M1_real_speed;    //����error term

    //����ʽPID
    //0�ŵ��
    M0_delta_u = (int16) (M0_Kp * (M0_ek - M0_ek1) + M0_Ki * M0_ek + M0_Kd * (M0_ek - 2 * M0_ek1 + M0_ek2));  //��������
    M0_u += M0_delta_u;    //�ۼ�������
    M0_u = (int16) limit(M0_u, GTM_ATOM0_PWM_DUTY_MAX);    //����޷�����ֹ�������100%ռ�ձ�

    M0_mot_duty = (int32) M0_u;    //��������ռ�ձ�

    //1�ŵ��
    M1_delta_u = (int16) (M1_Kp * (M1_ek - M1_ek1) + M1_Ki * M1_ek + M1_Kd * (M1_ek - 2 * M1_ek1 + M1_ek2));  //��������
    M1_u += M1_delta_u;    //�ۼ�������
    M1_u = (int16) limit(M1_u, GTM_ATOM0_PWM_DUTY_MAX);    //����޷�����ֹ�������100%ռ�ձ�

    M1_mot_duty = (int32) M1_u;    //��������ռ�ձ�

    //�����ռ�ձ�ת��ΪDRV8701EоƬ�߼�
    //0�ŵ��
    if (0 <= M0_mot_duty)    //�����ռ�ձ�>=0
    {
        gpio_set(M0_DIR, 0);    //�͵�ƽ��ת
        pwm_duty(M0_PWM, M0_mot_duty);    //���õ��ռ�ձ�
    }
    else
    {
        gpio_set(M0_DIR, 1);    //�ߵ�ƽ��ת
        pwm_duty(M0_PWM, -M0_mot_duty);    //���õ��ռ�ձ�
    }

    //1�ŵ��
    if (0 <= M1_mot_duty)    //�����ռ�ձ�>=0
    {
        gpio_set(M1_DIR, 0);    //�͵�ƽ��ת
        pwm_duty(M1_PWM, M1_mot_duty);    //���õ��ռ�ձ�
    }
    else
    {
        gpio_set(M1_DIR, 1);    //�ߵ�ƽ��ת
        pwm_duty(M1_PWM, -M1_mot_duty);    //���õ��ռ�ձ�
    }

    //printf("$%d %d;", M0_real_speed, M0_set_speed);
}

#pragma section all restore
