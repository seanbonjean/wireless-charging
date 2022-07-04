#include "headfile.h"
#include "user.h"

#pragma section all "cpu0_dsram"

float ramp_acc_ratio = RAMP_ACC_RATIO; //���ٶȼƱ���
float ramp_gyro_ratio = RAMP_GYRO_RATIO; //�����Ǳ���

float dt = IMU_PERIOD; //IMU��������

#pragma section all restore

extern boolean onRamp; //IMU����Ƿ����µ��ϣ����µ���ʱΪ1

#pragma section all "cpu0_psram"

//----------------------------------------------------------------
//  @brief      һ�׻����˲�
//  @param      angle_m     ���ٶȼ�����
//  @param      gyro_m      ����������
//  @return     float       �����ںϺ�ĽǶ�
//----------------------------------------------------------------

float ramp_angle_calc (float angle_m, float gyro_m)
{
    float temp_angle;
    float gyro_now;
    float error_angle;

    static float last_angle;
    static uint8 first_angle;

    if (!first_angle) //�ж��Ƿ�Ϊ��һ�����б�����
    {
        //����ǵ�һ�����У����ϴνǶ�ֵ����Ϊ����ٶ�ֵһ��
        first_angle = 1;
        last_angle = angle_m;
    }

    gyro_now = gyro_m * ramp_gyro_ratio;

    //���ݲ������ļ��ٶ�ֵת��Ϊ�Ƕ�֮�����ϴεĽǶ�ֵ��ƫ��
    error_angle = (angle_m - last_angle) * ramp_acc_ratio;

    //����ƫ���������ǲ����õ��ĽǶ�ֵ���㵱ǰ�Ƕ�ֵ
    temp_angle = last_angle + (error_angle + gyro_now) * dt;

    //���浱ǰ�Ƕ�ֵ
    last_angle = temp_angle;

    return temp_angle;
}

void getAngle (void)
{
    float ramp_angle;

    get_icm20602_accdata_spi();
    get_icm20602_gyro_spi();

    icm_gyro_x += 38; //����������
    icm_gyro_y -= 4;

    ramp_angle = ramp_angle_calc(icm_acc_x, icm_gyro_y);

    //printf("$%f ", ramp_angle);

    //�µ����
    if (!onRamp && ramp_angle > RAMP_THRESHOLD)
        onRamp = TRUE;
    if (onRamp && ramp_angle < -RAMP_THRESHOLD)
        onRamp = FALSE;
}

#pragma section all restore

void getAngle_init (void)
{
    icm20602_init_spi();
    pit_interrupt_ms(CCU6_0, PIT_CH1, 3);    //CCU6��ʱ����ʼ��
}
