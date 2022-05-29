#ifndef CODE_USER_CONFIG_H_
#define CODE_USER_CONFIG_H_

#define WITCH_CAR 1//ѡ��С����0��ˢ��1��ˢ
#define SERIES_OR_PARALLEL 1//ѡ����Ʋ��ԣ�0������1����

#define IN_RACE 0//�Ƿ��ڱ�����
/*
 * IN_RACE�궨�壺
 * 1.Ϊ0ʱ��IPS�Ҳ���ʾ�����PID������ͬʱ���ð�������
 * 2.Ϊ1ʱ��IPS�Ҳ���ʾ����ͷ�㷨���׶ν��ֵ�������ð�������
 * 3.����ʱ��1
 */
#define ENABLE_WIRELESS 1//ѡ���Ƿ�ʹ�����ߴ���������ݣ�0ʹ�����������ڣ�1ʹ�����ߴ���
#define SET_MEC_ZERO 0//ѡ���Ƿ�����е�����ԣ�1�������

#define SAMPLE_LINE 80//�ο��׵������
#define CAMERA_THRESHOLD 35//�ڰ����ز�Ⱥ���ֵ

#define SPEED_NORM 100//ƽʱ�ٶ�
#define SPEED_RAMP 50//�µ��ٶ�

#define FALL_ANGLE 800//ˤ���Ƕ���ֵ���ж��Ƿ�ˤ��
#define RAMP_THRESHOLD 1500//�µ���ֵ�������ж��Ƿ����µ�

#if(1 == WITCH_CAR)//��ˢ����

//IMU����
#define IMU_MEC_ZERO -200//IMU��е���
//ƽ���
#define BAL_ACC_RATIO 1.6//���ٶȼƱ���2.66
#define BAL_GYRO_RATIO 4.08//�����Ǳ���3.88
//�µ���
#define RAMP_ACC_RATIO 3.66//���ٶȼƱ���
#define RAMP_GYRO_RATIO 3.88//�����Ǳ���

#define IMU_PERIOD 0.003//IMU��������

//�����ֻ��ַ��������
#define MO_SPE_ERR_ISO 300//�ٶȻ�
#define MO_ANG_ERR_ISO 600//�ǶȻ�
#define MO_PAL_ERR_ISO 300//���ٶȻ�

//�����ֻ����޷�
#define MO_SPE_INT_LIM 4000//�ٶȻ�
#define MO_ANG_INT_LIM 4000//�ǶȻ�
#define MO_PAL_INT_LIM 4000//���ٶȻ�

//����������޷�
#define MO_SPE_LIMIT 4000//�ٶȻ�������ǶȻ��������Ƕȼ���Ϊ����-1400���µ�����3000����
#define MO_ANG_LIMIT 2000//�ǶȻ���������ٶȻ�
#define MO_PAL_LIMIT GTM_ATOM0_PWM_DUTY_MAX*0.5//���ٶȻ���������PWM�����ܳ���100%ռ�ձ�

//�����ֿ�������
#define MO_SPE_PERIOD 0.012f//�ٶȻ�
#define MO_ANG_PERIOD 0.006f//�ǶȻ�
#define MO_PAL_PERIOD 0.003f//���ٶȻ�

//�������ٶȻ�PID����
#define MO_SPE_KP 0//0.45//3.5 0.03
#define MO_SPE_KI 0//13//0.01
#define MO_SPE_KD 0

//�����ֽǶȻ�PID����
#define MO_ANG_KP 0//15//1.01//3.7 6.5
#define MO_ANG_KI 0//1//0.01
#define MO_ANG_KD 0//2.01//0.79

//�����ֽ��ٶȻ�PID����
#define MO_PAL_KP 4.5//3//15//3.5 7.1
#define MO_PAL_KI 0.08//0.1//4.49
#define MO_PAL_KD 0//0.1

//���PD����
#define SERVO_KP 5
#define SERVO_KD 2

#define SERVO_LIMIT 150//������Χ�޷�//todo

//���ֵ��PID����
#define REAR_KP 25
#define REAR_KI 2
#define REAR_KD 8

#elif(0 == WITCH_CAR)//��ˢ����

//IMU����
#define IMU_MEC_ZERO 115//IMU��е���
//ƽ���
#define BAL_ACC_RATIO 2.66//���ٶȼƱ���
#define BAL_GYRO_RATIO 3.88//�����Ǳ���
//�µ���
#define RAMP_ACC_RATIO 3.66//���ٶȼƱ���
#define RAMP_GYRO_RATIO 3.88//�����Ǳ���

#define IMU_PERIOD 0.003//IMU��������

//�����ֻ��ַ��������
#define MO_SPE_ERR_ISO 300//�ٶȻ�
#define MO_ANG_ERR_ISO 600//�ǶȻ�
#define MO_PAL_ERR_ISO 300//���ٶȻ�

//�����ֻ����޷�
#define MO_SPE_INT_LIM 100000//�ٶȻ�
#define MO_ANG_INT_LIM 100000//�ǶȻ�
#define MO_PAL_INT_LIM 100000//���ٶȻ�

//����������޷�
#define MO_SPE_LIMIT 1000//�ٶȻ�������ǶȻ��������Ƕȼ���Ϊ����-1400���µ�����3000����
#define MO_ANG_LIMIT 2000//�ǶȻ���������ٶȻ�
#define MO_PAL_LIMIT GTM_ATOM0_PWM_DUTY_MAX//���ٶȻ���������PWM�����ܳ���100%ռ�ձ�

//�����ֿ�������
#define MO_SPE_PERIOD 0.012f//�ٶȻ�
#define MO_ANG_PERIOD 0.006f//�ǶȻ�
#define MO_PAL_PERIOD 0.003f//���ٶȻ�

//�������ٶȻ�PID����
#define MO_SPE_KP 0//0.1 1.2
#define MO_SPE_KI 0//0 0.13
#define MO_SPE_KD 0//0 0

//�����ֽǶȻ�PID����
#define MO_ANG_KP 0//2 2.3
#define MO_ANG_KI 0//0 0
#define MO_ANG_KD 0//1 1

//�����ֽ��ٶȻ�PID����
#define MO_PAL_KP 0//10 200 20
#define MO_PAL_KI 0//20 20 10
#define MO_PAL_KD 0//0 0 0

//���PD����
#define SERVO_KP 5
#define SERVO_KD 2

#define SERVO_LIMIT 150//������Χ�޷�

//���ֵ��PID����
#define REAR_KP 25
#define REAR_KI 2
#define REAR_KD 8

#endif

#endif
