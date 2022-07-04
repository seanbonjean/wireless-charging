#ifndef CODE_USER_CONFIG_H_
#define CODE_USER_CONFIG_H_

#define IN_RACE 0//�Ƿ��ڱ�����
/*
 * IN_RACE�궨�壺
 * 1.Ϊ0ʱ��IPS�Ҳ���ʾ�����PID������ͬʱ���ð�������
 * 2.Ϊ1ʱ��IPS�Ҳ���ʾ����ͷ�㷨���׶ν��ֵ�������ð�������
 * 3.����ʱ��1
 */
#define ENABLE_WIRELESS 1//ѡ���Ƿ�ʹ�����ߴ���������ݣ�0ʹ�����������ڣ�1ʹ�����ߴ���

#define SAMPLE_LINE 80//�ο��׵������
#define CAMERA_THRESHOLD 35//�ڰ����ز�Ⱥ���ֵ

#define SPEED_NORM 100//ƽʱ�ٶ�
#define SPEED_RAMP 50//�µ��ٶ�

#define RAMP_THRESHOLD 1500//�µ���ֵ�������ж��Ƿ����µ�

//IMU����
#define IMU_MEC_ZERO 115//IMU��е���

#define RAMP_ACC_RATIO 3.66//���ٶȼƱ���
#define RAMP_GYRO_RATIO 3.88//�����Ǳ���

#define IMU_PERIOD 0.003//IMU��������

#define POS_MAP 50//λ��ƫ��ӳ�䵽�����ٶȱ���

//0�ŵ��PID����
#define M0_KP 25
#define M0_KI 2
#define M0_KD 8

//1�ŵ��PID����
#define M1_KP 25
#define M1_KI 2
#define M1_KD 8

#endif
