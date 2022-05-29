#include "headfile.h"
#include "user.h"

#define MOMEN_MOT_DIR P21_2//电机方向引脚
#define MOMEN_MOT_PWM ATOM0_CH1_P21_3//电机PWM引脚
#define MOMEN_MOT_nFAULT P21_4//电机过流保护引脚

#define MOMEN_ENC_LSB GPT12_T2INB_P33_7//编码器计数引脚
#define MOMEN_ENC_DIR GPT12_T2EUDB_P33_6//编码器方向引脚

extern boolean rest;

#pragma section all "cpu0_dsram"

int16 i = 0; //三环协同控制量

//TODO 成功直立后试试能否改成用结构体定义这些变量

//动量轮速度环
float moSpe_Kp, moSpe_Ki, moSpe_Kd; //PID参数
int16 moSpe_u; //输出量
int16 moSpe_last_u = 0; //上一次输出量
int16 moSpe_ek = 0, moSpe_ek1; //当次误差&历史误差
int16 moSpe_set_speed; //期望速度
int16 moSpe_real_speed; //实际速度
int32 moSpe_integ_accum = 0; //积分累加
int16 moSpe_isolate; //积分分离

//动量轮角度环
float moAng_Kp, moAng_Ki, moAng_Kd; //PID参数
int16 moAng_u; //输出量
int16 moAng_last_u = 0; //上一次输出量
int16 moAng_ek = 0, moAng_ek1; //当次误差&历史误差
int16 moAng_set_angle; //期望角度
int16 moAng_real_angle; //实际角度
int32 moAng_integ_accum = 0; //积分累加
int16 moAng_isolate; //积分分离

//动量轮角速度环
float moPal_Kp, moPal_Ki, moPal_Kd; //PID参数
int16 moPal_u; //输出量
int16 moPal_last_u = 0; //上一次输出量
int16 moPal_ek = 0, moPal_ek1; //当次误差&历史误差
int16 moPal_set_palst; //期望角速度
int16 moPal_real_palst; //实际角速度
int32 moPal_integ_accum = 0; //积分累加
int16 moPal_isolate; //积分分离

int32 mo_mot_duty; //电机占空比

#pragma section all restore

//动量轮平衡控制初始化
void balancePID_init (void)
{
    //电机初始化
    gpio_init(MOMEN_MOT_DIR, GPO, 0, PUSHPULL);    //电机方向引脚初始化
#if(1 == WITCH_CAR)
    gtm_pwm_init(MOMEN_MOT_PWM, 1000, 0);  //电机PWM引脚初始化
#elif(0 == WITCH_CAR)
    gtm_pwm_init(MOMEN_MOT_PWM, 17000, 0);  //电机PWM引脚初始化
#endif
    gpio_init(MOMEN_MOT_nFAULT, GPI, 0, PULLUP);    //电机过流保护引脚初始化

    gpt12_init(GPT12_T2, MOMEN_ENC_LSB, MOMEN_ENC_DIR);    //编码器初始化
    pit_interrupt_ms(CCU6_0, PIT_CH1, 3);    //CCU6定时器初始化

    moSpe_set_speed = 0;
    moAng_set_angle = 0;

    //设置PID系数
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

    //速度环
    if (i == 4)
    {
        moSpe_real_speed = gpt12_get(GPT12_T2);    //获取电机速度
        gpt12_clear(GPT12_T2);    //重置编码器

        moSpe_ek1 = moSpe_ek;    //更新历史误差
        moSpe_ek = moSpe_set_speed - moSpe_real_speed;    //计算error term
                /*
                 //积分分离
                 if (moSpe_ek > MO_SPE_ERR_ISO || moSpe_ek < -MO_SPE_ERR_ISO)    //若误差过大
                 moSpe_isolate = 0;    //关闭积分输入
                 else
                 {
                 moSpe_isolate = 1;    //开启积分输入

                 //积分抗饱和
                 if (moSpe_last_u > MO_SPE_LIMIT - 10)    //若输出上饱和
                 {
                 if (moSpe_ek < 0)    //只在误差能减小积分积累时
                 moSpe_integ_accum += moSpe_ek;    //积分累加
                 }
                 else if (moSpe_last_u < -MO_SPE_LIMIT + 10)    //若输出下饱和
                 {
                 if (moSpe_ek > 0)    //只在误差能减小积分积累时
                 moSpe_integ_accum += moSpe_ek;    //积分累加
                 }
                 else
                 moSpe_integ_accum += moSpe_ek;    //积分累加
                 }
                 */
        moSpe_integ_accum = limit((float) moSpe_integ_accum, MO_SPE_INT_LIM);    //积分限幅

        moSpe_u = (int16) (moSpe_Kp * 0.01 * moSpe_ek + moSpe_Ki * moSpe_integ_accum * MO_SPE_PERIOD
                + moSpe_Kd * (moSpe_ek - moSpe_ek1)); //计算输出
        moSpe_u = limit(moSpe_u, MO_SPE_LIMIT);    //输出限幅
    }

    //角度环
    if (i == 2 || i == 4)
    {
        moAng_set_angle = moSpe_u;    //输出期望角度
        moAng_real_angle = (int16) (angle);    //读取IMU姿态角，取整以加快运算
        moAng_real_angle -= IMU_MEC_ZERO;    //机械零点
        /*
         if (moAng_real_angle < 20 && moAng_real_angle > -20)
         moAng_real_angle = 0;
         */
        moAng_ek1 = moAng_ek;    //更新历史误差
        moAng_ek = moAng_set_angle - moAng_real_angle;    //计算error term
                /*
                 //积分分离
                 if (moAng_ek > MO_ANG_ERR_ISO || moAng_ek < -MO_ANG_ERR_ISO)    //若误差过大
                 moAng_isolate = 0;    //关闭积分输入
                 else
                 {
                 moAng_isolate = 1;    //开启积分输入

                 //积分抗饱和
                 if (moAng_last_u > MO_ANG_LIMIT - 10)    //若输出上饱和
                 {
                 if (moAng_ek < 0)    //只在误差能减小积分积累时
                 moAng_integ_accum += moAng_ek;    //积分累加
                 }
                 else if (moAng_last_u < -MO_ANG_LIMIT + 10)    //若输出下饱和
                 {
                 if (moAng_ek > 0)    //只在误差能减小积分积累时
                 moAng_integ_accum += moAng_ek;    //积分累加
                 }
                 else
                 moAng_integ_accum += moAng_ek;    //积分累加
                 }
                 */
        moAng_integ_accum = limit((float) moAng_integ_accum, MO_ANG_INT_LIM);    //积分限幅

        moAng_u = (int16) (moAng_Kp * moAng_ek + moAng_Ki * moAng_integ_accum * MO_ANG_PERIOD
                + moAng_Kd * (moAng_ek - moAng_ek1)); //计算输出
        //moAng_u = limit(moAng_u, MO_ANG_LIMIT);    //输出限幅
    }

    //角速度环
    moPal_set_palst = moAng_u;    //输出期望角速度
#if(1 == WITCH_CAR)
    moPal_real_palst = -icm_gyro_x;    //读取IMU实际角速度
#elif(0 == WITCH_CAR)
    moPal_real_palst = icm_gyro_y;    //读取IMU实际角速度
#endif

    moPal_ek1 = moPal_ek;    //更新历史误差
    moPal_ek = moPal_set_palst - moPal_real_palst;    //计算error term
            /*
             //积分分离
             if (moPal_ek > MO_PAL_ERR_ISO || moPal_ek < -MO_PAL_ERR_ISO)    //若误差过大
             moPal_isolate = 0;    //关闭积分输入
             else
             {
             moPal_isolate = 1;    //开启积分输入

             //积分抗饱和
             if (moPal_last_u > MO_PAL_LIMIT - 10)    //若输出上饱和
             {
             if (moPal_ek < 0)    //只在误差能减小积分积累时
             moPal_integ_accum += moPal_ek;    //积分累加
             }
             else if (moPal_last_u < -MO_PAL_LIMIT + 10)    //若输出下饱和
             {
             if (moPal_ek > 0)    //只在误差能减小积分积累时
             moPal_integ_accum += moPal_ek;    //积分累加
             }
             else
             moPal_integ_accum += moPal_ek;    //积分累加
             }
             */
    moPal_integ_accum = limit((float) moPal_integ_accum, MO_PAL_INT_LIM);    //积分限幅

    moPal_u = (int16) (moPal_Kp * moPal_ek + moPal_Ki * moPal_integ_accum * MO_PAL_PERIOD
            + moPal_Kd * (moPal_ek - moPal_ek1)); //计算输出
    moPal_u = limit(moPal_u, MO_PAL_LIMIT);    //输出限幅

    mo_mot_duty = (int32) moPal_u;    //输出电机占空比
#elif(0 == SERIES_OR_PARALLEL)
    //角度环
    moAng_real_angle = (int16) (angle);    //读取IMU姿态角，取整以加快运算
    moAng_real_angle -= IMU_MEC_ZERO;    //机械零点

    moAng_ek = moAng_real_angle;    //计算error term

    moAng_integ_accum += moAng_ek;    //积分累加
    moAng_integ_accum = limit(moAng_integ_accum, 2000);    //积分限幅

    moAng_u = (int16) (moAng_Kp * moAng_ek + moAng_Ki * moAng_integ_accum + moAng_Kd * icm_gyro_y);

    //速度环
    moSpe_real_speed = gpt12_get(GPT12_T2);    //获取电机速度
    gpt12_clear(GPT12_T2);    //重置编码器

    moSpe_ek1 = moSpe_ek;
    moSpe_ek = moSpe_real_speed;    //计算error term
    moSpe_ek = moSpe_ek * 0.3 + moSpe_ek1 * 0.7;

    moSpe_integ_accum += moSpe_ek;    //积分累加
    moSpe_integ_accum = limit(moSpe_integ_accum, 2000);    //积分限幅

    moSpe_u = (int16) (moSpe_Kp * moSpe_ek + moSpe_Ki * moSpe_integ_accum / 100);

    mo_mot_duty = (int32) limit(moAng_u - moSpe_u, GTM_ATOM0_PWM_DUTY_MAX);    //输出限幅，防止输出超过100%占空比
/*
    mo_mot_duty = (int32) (moAng_u - moSpe_u);

    if (mo_mot_duty > 8000)
        mo_mot_duty = 8000;        // 动量轮电机限幅
    else if (mo_mot_duty < -8000)
        mo_mot_duty = -8000; // 动量轮电机限幅
    else if (mo_mot_duty < -0)
        mo_mot_duty -= 800;    // 死区
    else if (mo_mot_duty > 0)
        mo_mot_duty += 800;      // 死区
    if ((mo_mot_duty < 1000) && (mo_mot_duty > -1000))
        mo_mot_duty = 0;
        */
#endif
    if (moAng_real_angle > FALL_ANGLE || moAng_real_angle < -FALL_ANGLE || rest)    //如果已经摔倒
    {
        //积分清零
        moSpe_integ_accum = 0;
        moAng_integ_accum = 0;
        moPal_integ_accum = 0;

        pwm_duty(MOMEN_MOT_PWM, 0);    //设置电机占空比
    }
    else
    {
#if(1 == WITCH_CAR)
        //将电机占空比转换为无刷电机驱动逻辑
        if (0 <= mo_mot_duty)    //若电机占空比>=0
        {
            pwm_duty(MOMEN_MOT_PWM, mo_mot_duty);    //设置电机占空比
            gpio_set(MOMEN_MOT_DIR, 1);    //高电平正转
        }
        else
        {
            pwm_duty(MOMEN_MOT_PWM, -mo_mot_duty);    //设置电机占空比
            gpio_set(MOMEN_MOT_DIR, 0);    //低电平反转
        }
#elif(0 == WITCH_CAR)
        mo_mot_duty = -mo_mot_duty;    //线接反了，反个向= =

        //将电机占空比转换为DRV8701E芯片逻辑
        if (0 <= mo_mot_duty)    //若电机占空比>=0
        {
            gpio_set(MOMEN_MOT_DIR, 0);    //低电平正转
            pwm_duty(MOMEN_MOT_PWM, mo_mot_duty);    //设置电机占空比
        }
        else
        {
            gpio_set(MOMEN_MOT_DIR, 1);    //高电平反转
            pwm_duty(MOMEN_MOT_PWM, -mo_mot_duty);    //设置电机占空比
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
