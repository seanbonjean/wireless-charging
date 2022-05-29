#include "headfile.h"
#include "user.h"

#define MOMEN_MOT_DIR P21_2//�����������
#define MOMEN_MOT_PWM ATOM0_CH1_P21_3//���PWM����
#define MOMEN_MOT_nFAULT P21_4//���������������

#define MOMEN_ENC_LSB GPT12_T2INB_P33_7//��������������
#define MOMEN_ENC_DIR GPT12_T2EUDB_P33_6//��������������

extern boolean rest;

#pragma section all "cpu0_dsram"

int16 i = 0; //����Эͬ������

//TODO �ɹ�ֱ���������ܷ�ĳ��ýṹ�嶨����Щ����

//�������ٶȻ�
float moSpe_Kp, moSpe_Ki, moSpe_Kd; //PID����
int16 moSpe_u; //�����
int16 moSpe_last_u = 0; //��һ�������
int16 moSpe_ek = 0, moSpe_ek1; //�������&��ʷ���
int16 moSpe_set_speed; //�����ٶ�
int16 moSpe_real_speed; //ʵ���ٶ�
int32 moSpe_integ_accum = 0; //�����ۼ�
int16 moSpe_isolate; //���ַ���

//�����ֽǶȻ�
float moAng_Kp, moAng_Ki, moAng_Kd; //PID����
int16 moAng_u; //�����
int16 moAng_last_u = 0; //��һ�������
int16 moAng_ek = 0, moAng_ek1; //�������&��ʷ���
int16 moAng_set_angle; //�����Ƕ�
int16 moAng_real_angle; //ʵ�ʽǶ�
int32 moAng_integ_accum = 0; //�����ۼ�
int16 moAng_isolate; //���ַ���

//�����ֽ��ٶȻ�
float moPal_Kp, moPal_Ki, moPal_Kd; //PID����
int16 moPal_u; //�����
int16 moPal_last_u = 0; //��һ�������
int16 moPal_ek = 0, moPal_ek1; //�������&��ʷ���
int16 moPal_set_palst; //�������ٶ�
int16 moPal_real_palst; //ʵ�ʽ��ٶ�
int32 moPal_integ_accum = 0; //�����ۼ�
int16 moPal_isolate; //���ַ���

int32 mo_mot_duty; //���ռ�ձ�

#pragma section all restore

//������ƽ����Ƴ�ʼ��
void balancePID_init (void)
{
    //�����ʼ��
    gpio_init(MOMEN_MOT_DIR, GPO, 0, PUSHPULL);    //����������ų�ʼ��
#if(1 == WITCH_CAR)
    gtm_pwm_init(MOMEN_MOT_PWM, 1000, 0);  //���PWM���ų�ʼ��
#elif(0 == WITCH_CAR)
    gtm_pwm_init(MOMEN_MOT_PWM, 17000, 0);  //���PWM���ų�ʼ��
#endif
    gpio_init(MOMEN_MOT_nFAULT, GPI, 0, PULLUP);    //��������������ų�ʼ��

    gpt12_init(GPT12_T2, MOMEN_ENC_LSB, MOMEN_ENC_DIR);    //��������ʼ��
    pit_interrupt_ms(CCU6_0, PIT_CH1, 3);    //CCU6��ʱ����ʼ��

    moSpe_set_speed = 0;
    moAng_set_angle = 0;

    //����PIDϵ��
    moSpe_Kp = MO_SPE_KP;    //proportional
    moSpe_Ki = MO_SPE_KI;    //integral
    moSpe_Kd = MO_SPE_KD;    //derivative

    moAng_Kp = MO_ANG_KP;    //proportional
    moAng_Ki = MO_ANG_KI;    //integral
    moAng_Kd = MO_ANG_KD;    //derivative

    moPal_Kp = MO_PAL_KP;    //proportional
    moPal_Ki = MO_PAL_KI;    //integral
    moPal_Kd = MO_PAL_KD;    //derivative
}

#pragma section all "cpu0_psram"

void balancePID (float angle)
{
#if(1 == SERIES_OR_PARALLEL)
    i++;
    if (i > 4)
        i = 1;

    //�ٶȻ�
    if (i == 4)
    {
        moSpe_real_speed = gpt12_get(GPT12_T2);    //��ȡ����ٶ�
        gpt12_clear(GPT12_T2);    //���ñ�����

        moSpe_ek1 = moSpe_ek;    //������ʷ���
        moSpe_ek = moSpe_set_speed - moSpe_real_speed;    //����error term
                /*
                 //���ַ���
                 if (moSpe_ek > MO_SPE_ERR_ISO || moSpe_ek < -MO_SPE_ERR_ISO)    //��������
                 moSpe_isolate = 0;    //�رջ�������
                 else
                 {
                 moSpe_isolate = 1;    //������������

                 //���ֿ�����
                 if (moSpe_last_u > MO_SPE_LIMIT - 10)    //������ϱ���
                 {
                 if (moSpe_ek < 0)    //ֻ������ܼ�С���ֻ���ʱ
                 moSpe_integ_accum += moSpe_ek;    //�����ۼ�
                 }
                 else if (moSpe_last_u < -MO_SPE_LIMIT + 10)    //������±���
                 {
                 if (moSpe_ek > 0)    //ֻ������ܼ�С���ֻ���ʱ
                 moSpe_integ_accum += moSpe_ek;    //�����ۼ�
                 }
                 else
                 moSpe_integ_accum += moSpe_ek;    //�����ۼ�
                 }
                 */
        moSpe_integ_accum = limit((float) moSpe_integ_accum, MO_SPE_INT_LIM);    //�����޷�

        moSpe_u = (int16) (moSpe_Kp * 0.01 * moSpe_ek + moSpe_Ki * moSpe_integ_accum * MO_SPE_PERIOD
                + moSpe_Kd * (moSpe_ek - moSpe_ek1)); //�������
        moSpe_u = limit(moSpe_u, MO_SPE_LIMIT);    //����޷�
    }

    //�ǶȻ�
    if (i == 2 || i == 4)
    {
        moAng_set_angle = moSpe_u;    //��������Ƕ�
        moAng_real_angle = (int16) (angle);    //��ȡIMU��̬�ǣ�ȡ���Լӿ�����
        moAng_real_angle -= IMU_MEC_ZERO;    //��е���
        /*
         if (moAng_real_angle < 20 && moAng_real_angle > -20)
         moAng_real_angle = 0;
         */
        moAng_ek1 = moAng_ek;    //������ʷ���
        moAng_ek = moAng_set_angle - moAng_real_angle;    //����error term
                /*
                 //���ַ���
                 if (moAng_ek > MO_ANG_ERR_ISO || moAng_ek < -MO_ANG_ERR_ISO)    //��������
                 moAng_isolate = 0;    //�رջ�������
                 else
                 {
                 moAng_isolate = 1;    //������������

                 //���ֿ�����
                 if (moAng_last_u > MO_ANG_LIMIT - 10)    //������ϱ���
                 {
                 if (moAng_ek < 0)    //ֻ������ܼ�С���ֻ���ʱ
                 moAng_integ_accum += moAng_ek;    //�����ۼ�
                 }
                 else if (moAng_last_u < -MO_ANG_LIMIT + 10)    //������±���
                 {
                 if (moAng_ek > 0)    //ֻ������ܼ�С���ֻ���ʱ
                 moAng_integ_accum += moAng_ek;    //�����ۼ�
                 }
                 else
                 moAng_integ_accum += moAng_ek;    //�����ۼ�
                 }
                 */
        moAng_integ_accum = limit((float) moAng_integ_accum, MO_ANG_INT_LIM);    //�����޷�

        moAng_u = (int16) (moAng_Kp * moAng_ek + moAng_Ki * moAng_integ_accum * MO_ANG_PERIOD
                + moAng_Kd * (moAng_ek - moAng_ek1)); //�������
        //moAng_u = limit(moAng_u, MO_ANG_LIMIT);    //����޷�
    }

    //���ٶȻ�
    moPal_set_palst = moAng_u;    //����������ٶ�
#if(1 == WITCH_CAR)
    moPal_real_palst = -icm_gyro_x;    //��ȡIMUʵ�ʽ��ٶ�
#elif(0 == WITCH_CAR)
    moPal_real_palst = icm_gyro_y;    //��ȡIMUʵ�ʽ��ٶ�
#endif

    moPal_ek1 = moPal_ek;    //������ʷ���
    moPal_ek = moPal_set_palst - moPal_real_palst;    //����error term
            /*
             //���ַ���
             if (moPal_ek > MO_PAL_ERR_ISO || moPal_ek < -MO_PAL_ERR_ISO)    //��������
             moPal_isolate = 0;    //�رջ�������
             else
             {
             moPal_isolate = 1;    //������������

             //���ֿ�����
             if (moPal_last_u > MO_PAL_LIMIT - 10)    //������ϱ���
             {
             if (moPal_ek < 0)    //ֻ������ܼ�С���ֻ���ʱ
             moPal_integ_accum += moPal_ek;    //�����ۼ�
             }
             else if (moPal_last_u < -MO_PAL_LIMIT + 10)    //������±���
             {
             if (moPal_ek > 0)    //ֻ������ܼ�С���ֻ���ʱ
             moPal_integ_accum += moPal_ek;    //�����ۼ�
             }
             else
             moPal_integ_accum += moPal_ek;    //�����ۼ�
             }
             */
    moPal_integ_accum = limit((float) moPal_integ_accum, MO_PAL_INT_LIM);    //�����޷�

    moPal_u = (int16) (moPal_Kp * moPal_ek + moPal_Ki * moPal_integ_accum * MO_PAL_PERIOD
            + moPal_Kd * (moPal_ek - moPal_ek1)); //�������
    moPal_u = limit(moPal_u, MO_PAL_LIMIT);    //����޷�

    mo_mot_duty = (int32) moPal_u;    //������ռ�ձ�
#elif(0 == SERIES_OR_PARALLEL)
    //�ǶȻ�
    moAng_real_angle = (int16) (angle);    //��ȡIMU��̬�ǣ�ȡ���Լӿ�����
    moAng_real_angle -= IMU_MEC_ZERO;    //��е���

    moAng_ek = moAng_real_angle;    //����error term

    moAng_integ_accum += moAng_ek;    //�����ۼ�
    moAng_integ_accum = limit(moAng_integ_accum, 2000);    //�����޷�

    moAng_u = (int16) (moAng_Kp * moAng_ek + moAng_Ki * moAng_integ_accum + moAng_Kd * icm_gyro_y);

    //�ٶȻ�
    moSpe_real_speed = gpt12_get(GPT12_T2);    //��ȡ����ٶ�
    gpt12_clear(GPT12_T2);    //���ñ�����

    moSpe_ek1 = moSpe_ek;
    moSpe_ek = moSpe_real_speed;    //����error term
    moSpe_ek = moSpe_ek * 0.3 + moSpe_ek1 * 0.7;

    moSpe_integ_accum += moSpe_ek;    //�����ۼ�
    moSpe_integ_accum = limit(moSpe_integ_accum, 2000);    //�����޷�

    moSpe_u = (int16) (moSpe_Kp * moSpe_ek + moSpe_Ki * moSpe_integ_accum / 100);

    mo_mot_duty = (int32) limit(moAng_u - moSpe_u, GTM_ATOM0_PWM_DUTY_MAX);    //����޷�����ֹ�������100%ռ�ձ�
/*
    mo_mot_duty = (int32) (moAng_u - moSpe_u);

    if (mo_mot_duty > 8000)
        mo_mot_duty = 8000;        // �����ֵ���޷�
    else if (mo_mot_duty < -8000)
        mo_mot_duty = -8000; // �����ֵ���޷�
    else if (mo_mot_duty < -0)
        mo_mot_duty -= 800;    // ����
    else if (mo_mot_duty > 0)
        mo_mot_duty += 800;      // ����
    if ((mo_mot_duty < 1000) && (mo_mot_duty > -1000))
        mo_mot_duty = 0;
        */
#endif
    if (moAng_real_angle > FALL_ANGLE || moAng_real_angle < -FALL_ANGLE || rest)    //����Ѿ�ˤ��
    {
        //��������
        moSpe_integ_accum = 0;
        moAng_integ_accum = 0;
        moPal_integ_accum = 0;

        pwm_duty(MOMEN_MOT_PWM, 0);    //���õ��ռ�ձ�
    }
    else
    {
#if(1 == WITCH_CAR)
        //�����ռ�ձ�ת��Ϊ��ˢ��������߼�
        if (0 <= mo_mot_duty)    //�����ռ�ձ�>=0
        {
            pwm_duty(MOMEN_MOT_PWM, mo_mot_duty);    //���õ��ռ�ձ�
            gpio_set(MOMEN_MOT_DIR, 1);    //�ߵ�ƽ��ת
        }
        else
        {
            pwm_duty(MOMEN_MOT_PWM, -mo_mot_duty);    //���õ��ռ�ձ�
            gpio_set(MOMEN_MOT_DIR, 0);    //�͵�ƽ��ת
        }
#elif(0 == WITCH_CAR)
        mo_mot_duty = -mo_mot_duty;    //�߽ӷ��ˣ�������= =

        //�����ռ�ձ�ת��ΪDRV8701EоƬ�߼�
        if (0 <= mo_mot_duty)    //�����ռ�ձ�>=0
        {
            gpio_set(MOMEN_MOT_DIR, 0);    //�͵�ƽ��ת
            pwm_duty(MOMEN_MOT_PWM, mo_mot_duty);    //���õ��ռ�ձ�
        }
        else
        {
            gpio_set(MOMEN_MOT_DIR, 1);    //�ߵ�ƽ��ת
            pwm_duty(MOMEN_MOT_PWM, -mo_mot_duty);    //���õ��ռ�ձ�
        }
#endif
    }

    //printf("$%d %d;", moAng_set_angle, moAng_real_angle);

    //printf("$%d %d;", moPal_set_palst, moPal_real_palst);

    //printf("$%ld;", moPal_integ_accum);

    //printf("$%d ", moAng_real_angle);

    //printf("%ld;", mo_mot_duty);

    //printf("$%d %d;", moAng_real_angle, moPal_u);

    //printf("$%d;", icm_gyro_y);

    //printf("$%d %d;", moSpe_isolate, moPal_isolate);

    printf("$%d %d;", moAng_real_angle, moAng_u);
}

#pragma section all restore
