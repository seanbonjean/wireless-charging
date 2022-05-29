#include "headfile.h"
#include "user.h"

#pragma section all "cpu0_dsram"

//平衡角
float bal_acc_ratio = BAL_ACC_RATIO; //加速度计比例
float bal_gyro_ratio = BAL_GYRO_RATIO; //陀螺仪比例

//坡道角
float ramp_acc_ratio = RAMP_ACC_RATIO; //加速度计比例
float ramp_gyro_ratio = RAMP_GYRO_RATIO; //陀螺仪比例

float dt = IMU_PERIOD; //IMU采样周期

#pragma section all restore

extern boolean onRamp; //IMU检测是否在坡道上，在坡道上时为1

#pragma section all "cpu0_psram"

//----------------------------------------------------------------
//  @brief      一阶互补滤波
//  @param      angle_m     加速度计数据
//  @param      gyro_m      陀螺仪数据
//  @return     float       数据融合后的角度
//----------------------------------------------------------------

float bal_angle_calc (float angle_m, float gyro_m)
{
    float temp_angle;
    float gyro_now;
    float error_angle;

    static float last_angle;
    static uint8 first_angle;

    if (!first_angle) //判断是否为第一次运行本函数
    {
        //如果是第一次运行，则将上次角度值设置为与加速度值一致
        first_angle = 1;
        last_angle = angle_m;
    }

    gyro_now = gyro_m * bal_gyro_ratio;

    //根据测量到的加速度值转换为角度之后与上次的角度值求偏差
    error_angle = (angle_m - last_angle) * bal_acc_ratio;

    //根据偏差与陀螺仪测量得到的角度值计算当前角度值
    temp_angle = last_angle + (error_angle + gyro_now) * dt;

    //保存当前角度值
    last_angle = temp_angle;

    return temp_angle;
}

float ramp_angle_calc (float angle_m, float gyro_m)
{
    float temp_angle;
    float gyro_now;
    float error_angle;

    static float last_angle;
    static uint8 first_angle;

    if (!first_angle) //判断是否为第一次运行本函数
    {
        //如果是第一次运行，则将上次角度值设置为与加速度值一致
        first_angle = 1;
        last_angle = angle_m;
    }

    gyro_now = gyro_m * ramp_gyro_ratio;

    //根据测量到的加速度值转换为角度之后与上次的角度值求偏差
    error_angle = (angle_m - last_angle) * ramp_acc_ratio;

    //根据偏差与陀螺仪测量得到的角度值计算当前角度值
    temp_angle = last_angle + (error_angle + gyro_now) * dt;

    //保存当前角度值
    last_angle = temp_angle;

    return temp_angle;
}

float getAngle (void)
{
    float bal_angle;
    float ramp_angle;

    get_icm20602_accdata_spi();
    get_icm20602_gyro_spi();

#if(1 == WITCH_CAR)

    icm_gyro_x += 38; //矫正陀螺仪
    icm_gyro_y -= 4;

    bal_angle = bal_angle_calc(icm_acc_y, -icm_gyro_x);
    ramp_angle = ramp_angle_calc(icm_acc_x, icm_gyro_y);

#elif(0 == WITCH_CAR)
    bal_angle = bal_angle_calc(icm_acc_z, icm_gyro_y);
    ramp_angle = ramp_angle_calc(icm_acc_y, -icm_gyro_z);
#endif

    //printf("$%f ", ramp_angle);

    //坡道检测
    if (!onRamp && ramp_angle > RAMP_THRESHOLD)
        onRamp = TRUE;
    if (onRamp && ramp_angle < -RAMP_THRESHOLD)
        onRamp = FALSE;

    return bal_angle;
}

#pragma section all restore

void getAngle_init (void)
{
    icm20602_init_spi();
}
