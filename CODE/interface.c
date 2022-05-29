#include "headfile.h"
#include "user.h"

#define BEEP_PIN P33_10//蜂鸣器引脚

//按键引脚
#define KEY1 P33_12
#define KEY2 P33_13
#define KEY3 P22_0
#define KEY4 P22_1
#define KEY5 P22_2
#define KEY6 P22_3

#pragma section all "cpu0_dsram"

boolean beeping = FALSE; //蜂鸣器发声指示，TRUE正在发声，FALSE未发声

boolean rest = FALSE; //电机休息，保护参数

//当前按键状态
boolean key1_state = TRUE;
boolean key2_state = TRUE;
boolean key3_state = TRUE;
boolean key4_state = TRUE;
boolean key5_state = TRUE;
boolean key6_state = TRUE;

//历史按键状态
boolean key1_preState = TRUE;
boolean key2_preState = TRUE;
boolean key3_preState = TRUE;
boolean key4_preState = TRUE;
boolean key5_preState = TRUE;
boolean key6_preState = TRUE;

enum
{
    moSpeed, moAngle, moPalstance, servo, rearWheel
} nowShowing = 0; //指示当前调参目标

enum
{
    Kp, Ki, Kd
} nowChanging = 0; //指示当前调参参数

enum
{
    verylow, low, medium, high, veryhigh
} changeLevel = medium; //调参幅度等级

#define CHANGE_LEVEL_VERYLOW 0.01
#define CHANGE_LEVEL_LOW 0.1
#define CHANGE_LEVEL_MEDIUM 1
#define CHANGE_LEVEL_HIGH 10
#define CHANGE_LEVEL_VERYHIGH 100

float changeSize = CHANGE_LEVEL_MEDIUM; //调参幅度

#pragma section all restore

extern boolean beep; //蜂鸣器发声命令，置1发声，并由CPU0控制发声300ms后置0停止发声

#if(0 == IN_RACE)
extern float moSpe_Kp, moSpe_Ki, moSpe_Kd;
extern float moAng_Kp, moAng_Ki, moAng_Kd;
extern float moPal_Kp, moPal_Ki, moPal_Kd;
extern float se_Kp, se_Kd;
extern float re_Kp, re_Ki, re_Kd;
#endif

#if(1 == IN_RACE)
extern float display_e2;
extern uint8 display_startLine;
extern uint8 display_startLine_length;
extern uint8 leftLine[MT9V03X_H]; //左边界
extern uint8 rightLine[MT9V03X_H]; //右边界
extern uint8 centerLine[MT9V03X_H]; //赛道中线
extern int16 pos; //记录位置偏差
#endif

//extern int16 re_set_speed; //电机期望速度

void interface_init (void)
{
    ips114_init(); //初始化IPS屏幕
    //显示初始化信息
    ips114_showstr(0, 0, "CQUPT violet evergarden team");
    ips114_showstr(0, 1, "Initializing, please wait. ");

    gpio_init(BEEP_PIN, GPO, 1, PUSHPULL); //初始化蜂鸣器引脚

    //初始化按键引脚
    gpio_init(KEY1, GPI, 0, PULLUP);
    gpio_init(KEY2, GPI, 0, PULLUP);
    gpio_init(KEY3, GPI, 0, PULLUP);
    gpio_init(KEY4, GPI, 0, PULLUP);
}

void interface_start (void)
{
    ips114_clear(WHITE); //清屏
    gpio_set(BEEP_PIN, 0); //关闭蜂鸣器

#if(0 == IN_RACE)
    //显示不会改变的屏幕元素
    ips114_showstr(190, 2, "Kp:");
    ips114_showstr(190, 4, "Ki:");
    ips114_showstr(190, 6, "Kd:");
#endif
}

#pragma section all "cpu0_psram"

void interface (void)
{
    if (!beeping && beep) //若未发声且请求发声
    {
        gpio_set(BEEP_PIN, 1); //打开蜂鸣器
        beeping = TRUE; //指示蜂鸣器正在发声
        systick_start(STM0); //开始发声计时
    }
    else if (beeping && 300 < systick_getval_ms(STM0)) //若正在发声且计时时间到
    {
        gpio_set(BEEP_PIN, 0); //关闭蜂鸣器
        beeping = FALSE; //指示蜂鸣器未发声
        beep = FALSE; //发声命令置0，等待接受下一次发声命令
    }

    //更新历史按键状态
    key1_preState = key1_state;
    key2_preState = key2_state;
    key3_preState = key3_state;
    key4_preState = key4_state;
    key5_preState = key5_state;
    key6_preState = key6_state;

    //读取当前按键状态
    key1_state = gpio_get(KEY1);
    key2_state = gpio_get(KEY2);
    key3_state = gpio_get(KEY3);
    key4_state = gpio_get(KEY4);
    key5_state = gpio_get(KEY5);
    key6_state = gpio_get(KEY6);

    //当按键6按下
    if (!key6_state && key6_preState)
        rest = !rest;
    //re_set_speed = SPEED_NORM; //走起！

#if(0 == IN_RACE)
    //当按键1按下
    if (!key1_state && key1_preState)
    {
        nowShowing++; //更改当前调参目标
        if (nowShowing > rearWheel) //防止超出范围
            nowShowing = moSpeed;
    }

    //当按键2按下
    if (!key2_state && key2_preState)
    {
        nowChanging++; //更改当前调参参数
        if (nowChanging > Kd) //防止超出范围
            nowChanging = Kp;
    }

    //当按键3按下
    if (!key3_state && key3_preState)
    {
        changeLevel++;
        if (changeLevel > veryhigh)
            changeLevel = verylow;

        switch (changeLevel)
        {
            case verylow :
                changeSize = CHANGE_LEVEL_VERYLOW;
                break;
            case low :
                changeSize = CHANGE_LEVEL_LOW;
                break;
            case medium :
                changeSize = CHANGE_LEVEL_MEDIUM;
                break;
            case high :
                changeSize = CHANGE_LEVEL_HIGH;
                break;
            case veryhigh :
                changeSize = CHANGE_LEVEL_VERYHIGH;
                break;
        }
    }

    //当按键4按下
    if (!key4_state && key4_preState)
    {
        switch (nowShowing)
        {
            case moSpeed :
                switch (nowChanging)
                {
                    case Kp :
                        moSpe_Kp -= changeSize;
                        break;

                    case Ki :
                        moSpe_Ki -= changeSize;
                        break;

                    case Kd :
                        moSpe_Kd -= changeSize;
                        break;
                }
                break;

            case moAngle :
                switch (nowChanging)
                {
                    case Kp :
                        moAng_Kp -= changeSize;
                        break;

                    case Ki :
                        moAng_Ki -= changeSize;
                        break;

                    case Kd :
                        moAng_Kd -= changeSize;
                        break;
                }
                break;

            case moPalstance :
                switch (nowChanging)
                {
                    case Kp :
                        moPal_Kp -= changeSize;
                        break;

                    case Ki :
                        moPal_Ki -= changeSize;
                        break;

                    case Kd :
                        moPal_Kd -= changeSize;
                        break;
                }
                break;

            case servo :
                switch (nowChanging)
                {
                    case Kp :
                        se_Kp -= changeSize;
                        break;

                    case Ki :
                        break;

                    case Kd :
                        se_Kd -= changeSize;
                        break;
                }
                break;

            case rearWheel :
                switch (nowChanging)
                {
                    case Kp :
                        re_Kp -= changeSize;
                        break;

                    case Ki :
                        re_Ki -= changeSize;
                        break;

                    case Kd :
                        re_Kd -= changeSize;
                        break;
                }
                break;
        }
    }

    //当按键5按下
    if (!key5_state && key5_preState)
    {
        switch (nowShowing)
        {
            case moSpeed :
                switch (nowChanging)
                {
                    case Kp :
                        moSpe_Kp += changeSize;
                        break;

                    case Ki :
                        moSpe_Ki += changeSize;
                        break;

                    case Kd :
                        moSpe_Kd += changeSize;
                        break;
                }
                break;

            case moAngle :
                switch (nowChanging)
                {
                    case Kp :
                        moAng_Kp += changeSize;
                        break;

                    case Ki :
                        moAng_Ki += changeSize;
                        break;

                    case Kd :
                        moAng_Kd += changeSize;
                        break;
                }
                break;

            case moPalstance :
                switch (nowChanging)
                {
                    case Kp :
                        moPal_Kp += changeSize;
                        break;

                    case Ki :
                        moPal_Ki += changeSize;
                        break;

                    case Kd :
                        moPal_Kd += changeSize;
                        break;
                }
                break;

            case servo :
                switch (nowChanging)
                {
                    case Kp :
                        se_Kp += changeSize;
                        break;

                    case Ki :
                        break;

                    case Kd :
                        se_Kd += changeSize;
                        break;
                }
                break;

            case rearWheel :
                switch (nowChanging)
                {
                    case Kp :
                        re_Kp += changeSize;
                        break;

                    case Ki :
                        re_Ki += changeSize;
                        break;

                    case Kd :
                        re_Kd += changeSize;
                        break;
                }
                break;
        }
    }

    //显示PID参数
    switch (nowShowing)
    {
        case moSpeed :
            ips114_showstr(190, 0, "moSpe");
            ips114_showfloat(190, 3, moSpe_Kp, 3, 2);
            ips114_showfloat(190, 5, moSpe_Ki, 3, 2);
            ips114_showfloat(190, 7, moSpe_Kd, 3, 2);
            break;

        case moAngle :
            ips114_showstr(190, 0, "moAng");
            ips114_showfloat(190, 3, moAng_Kp, 3, 2);
            ips114_showfloat(190, 5, moAng_Ki, 3, 2);
            ips114_showfloat(190, 7, moAng_Kd, 3, 2);
            break;

        case moPalstance :
            ips114_showstr(190, 0, "moPal");
            ips114_showfloat(190, 3, moPal_Kp, 3, 2);
            ips114_showfloat(190, 5, moPal_Ki, 3, 2);
            ips114_showfloat(190, 7, moPal_Kd, 3, 2);
            break;

        case servo :
            ips114_showstr(190, 0, "servo");
            ips114_showfloat(190, 3, se_Kp, 3, 2);
            ips114_showstr(190, 5, "No Ki");
            ips114_showfloat(190, 7, se_Kd, 3, 2);
            break;

        case rearWheel :
            ips114_showstr(190, 0, "rear ");
            ips114_showfloat(190, 3, re_Kp, 3, 2);
            ips114_showfloat(190, 5, re_Ki, 3, 2);
            ips114_showfloat(190, 7, re_Kd, 3, 2);
            break;
    }

    //显示当前选中参数
    switch (nowChanging)
    {
        case Kp :
            ips114_showstr(190, 1, "P");
            break;

        case Ki :
            ips114_showstr(190, 1, "I");
            break;

        case Kd :
            ips114_showstr(190, 1, "D");
            break;
    }

    //显示当前调参幅度
    ips114_showfloat(200, 1, changeSize, 3, 2);
#endif

#if(1 == IN_RACE)
    ips114_showfloat(190, 0, display_e2, 3, 2);
    ips114_showuint8(190, 1, display_startLine);
    ips114_showuint8(190, 2, display_startLine_length);
    ips114_showint16(190, 3, pos);

    for (int i = 0; i < MT9V03X_W; i++) //清除图像底部显示
    {
        for (int j = MT9V03X_H; j < IPS114_W; j++)
            ips114_drawpoint((uint16) i, (uint16) j, WHITE);
    }
    for (int i = MT9V03X_H; i < IPS114_W; i++) //显示边界搜索起始线
        ips114_drawpoint((uint16) display_startLine, (uint16) i, RED);

    for (int i = display_startLine_length; i < MT9V03X_H; i++) //显示赛道信息
    {
        ips114_drawpoint((uint16) leftLine[i], (uint16) i, RED);
        ips114_drawpoint((uint16) rightLine[i], (uint16) i, RED);
        ips114_drawpoint((uint16) centerLine[i], (uint16) i, GREEN);
    }
#endif

    ips114_displayimage032(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
}

#pragma section all restore
