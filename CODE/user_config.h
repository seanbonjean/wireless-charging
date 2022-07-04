#ifndef CODE_USER_CONFIG_H_
#define CODE_USER_CONFIG_H_

#define IN_RACE 0//是否在比赛中
/*
 * IN_RACE宏定义：
 * 1.为0时，IPS右侧显示各组件PID参数，同时启用按键调参
 * 2.为1时，IPS右侧显示摄像头算法各阶段结果值，并禁用按键调参
 * 3.比赛时置1
 */
#define ENABLE_WIRELESS 1//选择是否使用无线串口输出数据，0使用下载器串口，1使用无线串口

#define SAMPLE_LINE 80//参考白点计算线
#define CAMERA_THRESHOLD 35//黑白像素差比和阈值

#define SPEED_NORM 100//平时速度
#define SPEED_RAMP 50//坡道速度

#define RAMP_THRESHOLD 1500//坡道阈值，用于判断是否上坡道

//IMU参数
#define IMU_MEC_ZERO 115//IMU机械零点

#define RAMP_ACC_RATIO 3.66//加速度计比例
#define RAMP_GYRO_RATIO 3.88//陀螺仪比例

#define IMU_PERIOD 0.003//IMU采样周期

#define POS_MAP 50//位置偏差映射到差速速度比例

//0号电机PID参数
#define M0_KP 25
#define M0_KI 2
#define M0_KD 8

//1号电机PID参数
#define M1_KP 25
#define M1_KI 2
#define M1_KD 8

#endif
