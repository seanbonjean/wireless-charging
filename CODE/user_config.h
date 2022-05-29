#ifndef CODE_USER_CONFIG_H_
#define CODE_USER_CONFIG_H_

#define WITCH_CAR 1//选择小车：0有刷，1无刷
#define SERIES_OR_PARALLEL 1//选择控制策略：0并级，1串级

#define IN_RACE 0//是否在比赛中
/*
 * IN_RACE宏定义：
 * 1.为0时，IPS右侧显示各组件PID参数，同时启用按键调参
 * 2.为1时，IPS右侧显示摄像头算法各阶段结果值，并禁用按键调参
 * 3.比赛时置1
 */
#define ENABLE_WIRELESS 1//选择是否使用无线串口输出数据，0使用下载器串口，1使用无线串口
#define SET_MEC_ZERO 0//选择是否进入机械零点调试，1进入调试

#define SAMPLE_LINE 80//参考白点计算线
#define CAMERA_THRESHOLD 35//黑白像素差比和阈值

#define SPEED_NORM 100//平时速度
#define SPEED_RAMP 50//坡道速度

#define FALL_ANGLE 800//摔倒角度阈值，判断是否摔倒
#define RAMP_THRESHOLD 1500//坡道阈值，用于判断是否上坡道

#if(1 == WITCH_CAR)//无刷参数

//IMU参数
#define IMU_MEC_ZERO -200//IMU机械零点
//平衡角
#define BAL_ACC_RATIO 1.6//加速度计比例2.66
#define BAL_GYRO_RATIO 4.08//陀螺仪比例3.88
//坡道角
#define RAMP_ACC_RATIO 3.66//加速度计比例
#define RAMP_GYRO_RATIO 3.88//陀螺仪比例

#define IMU_PERIOD 0.003//IMU采样周期

//动量轮积分分离误差限
#define MO_SPE_ERR_ISO 300//速度环
#define MO_ANG_ERR_ISO 600//角度环
#define MO_PAL_ERR_ISO 300//角速度环

//动量轮积分限幅
#define MO_SPE_INT_LIM 4000//速度环
#define MO_ANG_INT_LIM 4000//角度环
#define MO_PAL_INT_LIM 4000//角速度环

//动量轮输出限幅
#define MO_SPE_LIMIT 4000//速度环输出给角度环，单车角度极限为左倾-1400以下到右倾3000以上
#define MO_ANG_LIMIT 2000//角度环输出给角速度环
#define MO_PAL_LIMIT GTM_ATOM0_PWM_DUTY_MAX*0.5//角速度环输出给电机PWM，不能超过100%占空比

//动量轮控制周期
#define MO_SPE_PERIOD 0.012f//速度环
#define MO_ANG_PERIOD 0.006f//角度环
#define MO_PAL_PERIOD 0.003f//角速度环

//动量轮速度环PID参数
#define MO_SPE_KP 0//0.45//3.5 0.03
#define MO_SPE_KI 0//13//0.01
#define MO_SPE_KD 0

//动量轮角度环PID参数
#define MO_ANG_KP 0//15//1.01//3.7 6.5
#define MO_ANG_KI 0//1//0.01
#define MO_ANG_KD 0//2.01//0.79

//动量轮角速度环PID参数
#define MO_PAL_KP 4.5//3//15//3.5 7.1
#define MO_PAL_KI 0.08//0.1//4.49
#define MO_PAL_KD 0//0.1

//舵机PD参数
#define SERVO_KP 5
#define SERVO_KD 2

#define SERVO_LIMIT 150//舵机活动范围限幅//todo

//后轮电机PID参数
#define REAR_KP 25
#define REAR_KI 2
#define REAR_KD 8

#elif(0 == WITCH_CAR)//有刷参数

//IMU参数
#define IMU_MEC_ZERO 115//IMU机械零点
//平衡角
#define BAL_ACC_RATIO 2.66//加速度计比例
#define BAL_GYRO_RATIO 3.88//陀螺仪比例
//坡道角
#define RAMP_ACC_RATIO 3.66//加速度计比例
#define RAMP_GYRO_RATIO 3.88//陀螺仪比例

#define IMU_PERIOD 0.003//IMU采样周期

//动量轮积分分离误差限
#define MO_SPE_ERR_ISO 300//速度环
#define MO_ANG_ERR_ISO 600//角度环
#define MO_PAL_ERR_ISO 300//角速度环

//动量轮积分限幅
#define MO_SPE_INT_LIM 100000//速度环
#define MO_ANG_INT_LIM 100000//角度环
#define MO_PAL_INT_LIM 100000//角速度环

//动量轮输出限幅
#define MO_SPE_LIMIT 1000//速度环输出给角度环，单车角度极限为左倾-1400以下到右倾3000以上
#define MO_ANG_LIMIT 2000//角度环输出给角速度环
#define MO_PAL_LIMIT GTM_ATOM0_PWM_DUTY_MAX//角速度环输出给电机PWM，不能超过100%占空比

//动量轮控制周期
#define MO_SPE_PERIOD 0.012f//速度环
#define MO_ANG_PERIOD 0.006f//角度环
#define MO_PAL_PERIOD 0.003f//角速度环

//动量轮速度环PID参数
#define MO_SPE_KP 0//0.1 1.2
#define MO_SPE_KI 0//0 0.13
#define MO_SPE_KD 0//0 0

//动量轮角度环PID参数
#define MO_ANG_KP 0//2 2.3
#define MO_ANG_KI 0//0 0
#define MO_ANG_KD 0//1 1

//动量轮角速度环PID参数
#define MO_PAL_KP 0//10 200 20
#define MO_PAL_KI 0//20 20 10
#define MO_PAL_KD 0//0 0 0

//舵机PD参数
#define SERVO_KP 5
#define SERVO_KD 2

#define SERVO_LIMIT 150//舵机活动范围限幅

//后轮电机PID参数
#define REAR_KP 25
#define REAR_KI 2
#define REAR_KD 8

#endif

#endif
