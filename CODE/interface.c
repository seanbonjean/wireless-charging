#include "headfile.h"
#include "user.h"

#define BEEP_PIN P33_10//����������

//��������
#define KEY1 P33_12
#define KEY2 P33_13
#define KEY3 P22_0
#define KEY4 P22_1
#define KEY5 P22_2
#define KEY6 P22_3

#pragma section all "cpu0_dsram"

boolean beeping = FALSE; //����������ָʾ��TRUE���ڷ�����FALSEδ����

//��ǰ����״̬
boolean key1_state = TRUE;
boolean key2_state = TRUE;
boolean key3_state = TRUE;
boolean key4_state = TRUE;
boolean key5_state = TRUE;
boolean key6_state = TRUE;

//��ʷ����״̬
boolean key1_preState = TRUE;
boolean key2_preState = TRUE;
boolean key3_preState = TRUE;
boolean key4_preState = TRUE;
boolean key5_preState = TRUE;
boolean key6_preState = TRUE;

enum
{
    M0, M1
} nowShowing = 0; //ָʾ��ǰ����Ŀ��

enum
{
    Kp, Ki, Kd
} nowChanging = 0; //ָʾ��ǰ���β���

enum
{
    verylow, low, medium, high, veryhigh
} changeLevel = medium; //���η��ȵȼ�

#define CHANGE_LEVEL_VERYLOW 0.01
#define CHANGE_LEVEL_LOW 0.1
#define CHANGE_LEVEL_MEDIUM 1
#define CHANGE_LEVEL_HIGH 10
#define CHANGE_LEVEL_VERYHIGH 100

float changeSize = CHANGE_LEVEL_MEDIUM; //���η���

#pragma section all restore

extern boolean beep; //���������������1����������CPU0���Ʒ���300ms����0ֹͣ����

#if(0 == IN_RACE)
extern float M0_Kp, M0_Ki, M0_Kd;
extern float M1_Kp, M1_Ki, M1_Kd;
#endif

#if(1 == IN_RACE)
extern float display_e2;
extern uint8 display_startLine;
extern uint8 display_startLine_length;
extern uint8 leftLine[MT9V03X_H]; //��߽�
extern uint8 rightLine[MT9V03X_H]; //�ұ߽�
extern uint8 centerLine[MT9V03X_H]; //��������
extern int16 pos; //��¼λ��ƫ��
#endif

/*
extern int16 M0_set_speed; //��������ٶ�
extern int16 M1_set_speed;
*/

void interface_init (void)
{
    ips114_init(); //��ʼ��IPS��Ļ
    //��ʾ��ʼ����Ϣ
    ips114_showstr(0, 0, "CQUPT violet evergarden team");
    ips114_showstr(0, 1, "Initializing, please wait. ");

    gpio_init(BEEP_PIN, GPO, 1, PUSHPULL); //��ʼ������������

    //��ʼ����������
    gpio_init(KEY1, GPI, 0, PULLUP);
    gpio_init(KEY2, GPI, 0, PULLUP);
    gpio_init(KEY3, GPI, 0, PULLUP);
    gpio_init(KEY4, GPI, 0, PULLUP);
}

void interface_start (void)
{
    ips114_clear(WHITE); //����
    gpio_set(BEEP_PIN, 0); //�رշ�����

#if(0 == IN_RACE)
    //��ʾ����ı����ĻԪ��
    ips114_showstr(190, 2, "Kp:");
    ips114_showstr(190, 4, "Ki:");
    ips114_showstr(190, 6, "Kd:");
#endif
}

#pragma section all "cpu0_psram"

void interface (void)
{
    if (!beeping && beep) //��δ������������
    {
        gpio_set(BEEP_PIN, 1); //�򿪷�����
        beeping = TRUE; //ָʾ���������ڷ���
        systick_start(STM0); //��ʼ������ʱ
    }
    else if (beeping && 300 < systick_getval_ms(STM0)) //�����ڷ����Ҽ�ʱʱ�䵽
    {
        gpio_set(BEEP_PIN, 0); //�رշ�����
        beeping = FALSE; //ָʾ������δ����
        beep = FALSE; //����������0���ȴ�������һ�η�������
    }

    //������ʷ����״̬
    key1_preState = key1_state;
    key2_preState = key2_state;
    key3_preState = key3_state;
    key4_preState = key4_state;
    key5_preState = key5_state;
    key6_preState = key6_state;

    //��ȡ��ǰ����״̬
    key1_state = gpio_get(KEY1);
    key2_state = gpio_get(KEY2);
    key3_state = gpio_get(KEY3);
    key4_state = gpio_get(KEY4);
    key5_state = gpio_get(KEY5);
    key6_state = gpio_get(KEY6);

    //������6����
    if (!key6_state && key6_preState)
    {
        /*
        M0_set_speed = SPEED_NORM; //����
        M1_set_speed = SPEED_NORM;
        */
    }

#if(0 == IN_RACE)
    //������1����
    if (!key1_state && key1_preState)
    {
        nowShowing++; //���ĵ�ǰ����Ŀ��
        if (nowShowing > M1) //��ֹ������Χ
            nowShowing = M0;
    }

    //������2����
    if (!key2_state && key2_preState)
    {
        nowChanging++; //���ĵ�ǰ���β���
        if (nowChanging > Kd) //��ֹ������Χ
            nowChanging = Kp;
    }

    //������3����
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

    //������4����
    if (!key4_state && key4_preState)
    {
        switch (nowShowing)
        {
            case M0 :
                switch (nowChanging)
                {
                    case Kp :
                        M0_Kp -= changeSize;
                        break;

                    case Ki :
                        M0_Ki -= changeSize;
                        break;

                    case Kd :
                        M0_Kd -= changeSize;
                        break;
                }
                break;

            case M1 :
                switch (nowChanging)
                {
                    case Kp :
                        M1_Kp -= changeSize;
                        break;

                    case Ki :
                        M1_Ki -= changeSize;
                        break;

                    case Kd :
                        M1_Kd -= changeSize;
                        break;
                }
                break;
        }
    }

    //������5����
    if (!key5_state && key5_preState)
    {
        switch (nowShowing)
        {
            case M0 :
                switch (nowChanging)
                {
                    case Kp :
                        M0_Kp += changeSize;
                        break;

                    case Ki :
                        M0_Ki += changeSize;
                        break;

                    case Kd :
                        M0_Kd += changeSize;
                        break;
                }
                break;

            case M1 :
                switch (nowChanging)
                {
                    case Kp :
                        M1_Kp += changeSize;
                        break;

                    case Ki :
                        M1_Ki += changeSize;
                        break;

                    case Kd :
                        M1_Kd += changeSize;
                        break;
                }
                break;
        }
    }

    //��ʾPID����
    switch (nowShowing)
    {
        case M0 :
            ips114_showstr(190, 0, "M0");
            ips114_showfloat(190, 3, M0_Kp, 3, 2);
            ips114_showfloat(190, 5, M0_Ki, 3, 2);
            ips114_showfloat(190, 7, M0_Kd, 3, 2);
            break;

        case M1 :
            ips114_showstr(190, 0, "M1");
            ips114_showfloat(190, 3, M1_Kp, 3, 2);
            ips114_showfloat(190, 5, M1_Ki, 3, 2);
            ips114_showfloat(190, 7, M1_Kd, 3, 2);
            break;
    }

    //��ʾ��ǰѡ�в���
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

    //��ʾ��ǰ���η���
    ips114_showfloat(200, 1, changeSize, 3, 2);
#endif

#if(1 == IN_RACE)
    ips114_showfloat(190, 0, display_e2, 3, 2);
    ips114_showuint8(190, 1, display_startLine);
    ips114_showuint8(190, 2, display_startLine_length);
    ips114_showint16(190, 3, pos);

    for (int i = 0; i < MT9V03X_W; i++) //���ͼ��ײ���ʾ
    {
        for (int j = MT9V03X_H; j < IPS114_W; j++)
            ips114_drawpoint((uint16) i, (uint16) j, WHITE);
    }
    for (int i = MT9V03X_H; i < IPS114_W; i++) //��ʾ�߽�������ʼ��
        ips114_drawpoint((uint16) display_startLine, (uint16) i, RED);

    for (int i = display_startLine_length; i < MT9V03X_H; i++) //��ʾ������Ϣ
    {
        ips114_drawpoint((uint16) leftLine[i], (uint16) i, RED);
        ips114_drawpoint((uint16) rightLine[i], (uint16) i, RED);
        ips114_drawpoint((uint16) centerLine[i], (uint16) i, GREEN);
    }
#endif

    ips114_displayimage032(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
}

#pragma section all restore
