#include "headfile.h"
#include "user.h"

#define GRAY_SCALE 256//�Ҷ�ֵ��Χ

#pragma section all "cpu1_dsram"

int16 pos = 0; //��¼λ��ƫ��
boolean beep = FALSE; //���������������1����������CPU0���Ʒ���300ms����0ֹͣ����
boolean onRamp = FALSE; //IMU����Ƿ����µ��ϣ����µ���ʱΪTRUE

//�ο��׵����
uint8 grayLevel; //�ݴ浱ǰ�Ҷ�ֵ�������ظ�Ѱַ
uint8 minGray, maxGray; //��С/���Ҷ�ֵ
uint8 n1, n2; //��/�����ظ���
uint16 g1, g2; //��/�׻Ҷ��ۼ�ֵ
float p1, p2; //��/�����ظ���
float e1, e2; //��/�����ؾ�ֵ
float e12; //e1 - e2
float sigma; //����ֵ��Ӧsigmaֵ
float max_sigma; //���sigmaֵ
float maxSig_e2; //���sigma��Ӧe2��ֵ�����ο��׵�

//�߽�������ʼ�߼���
uint8 startLine; //�߽�������ʼ��
uint8 startLine_length; //�߽�������ʼ�߷����򳤶�

//������Ϣ��¼����
uint8 leftLine[MT9V03X_H]; //��߽�
uint8 rightLine[MT9V03X_H]; //�ұ߽�
uint8 centerLine[MT9V03X_H]; //��������

//λ��ƫ�����
uint8 center; //�ݴ浱ǰ���е㣬�����ظ�Ѱַ
int16 centerError; //���������

//���ٿ���
int16 overall_speed = 0; //ȫ���ٶ�
extern int16 M0_set_speed; //�����ٶ�
extern int16 M1_set_speed;

//��Ļ��ʾ
float display_e2;
uint8 display_startLine;
uint8 display_startLine_length;

enum
{
    out, junction1, ramp, junction2, roundabout1, cross1, roundabout2, junction3, junction4, cross2, in
} carState; //��������ָʾ

#pragma section all restore

void getPosition_init (void)
{
    mt9v03x_init(); //����ͷ��ʼ��
}

#pragma section all "cpu1_psram"

void getPosition (void)
{
    if (mt9v03x_finish_flag) //���ɼ����ʱ
    {
        //seekfree_sendimg_03x(WIRELESS_UART, mt9v03x_image[0], MT9V03X_W, MT9V03X_H); //ͼ������λ��

        //ȡ���е�һ�����ػҶ���Ϊ�Ҷȷ�Χ�ĳ�ֵ
        minGray = mt9v03x_image[SAMPLE_LINE][0];
        maxGray = minGray;

        for (int i = 1; i < MT9V03X_W; i++) //���������������ػҶȣ��õ��Ҷȷ�Χ
        {
            grayLevel = mt9v03x_image[SAMPLE_LINE][i];

            if (minGray > grayLevel) //������С�Ҷ�
                minGray = grayLevel;
            else if (maxGray < grayLevel) //�������Ҷ�
                maxGray = grayLevel;
        }

        max_sigma = 0; //��ʼ�����sigmaֵ

        for (int i = minGray; i <= maxGray; i++) //�ڻҶȷ�Χ�ڱ����Ҷ�
        {
            //��ʼ��ͳ��ֵ
            n1 = 0;
            g1 = 0;
            g2 = 0;

            for (int j = 0; j < MT9V03X_W; j++) //��ÿһ���ٶ�����ֵ����������sigma
            {
                grayLevel = mt9v03x_image[SAMPLE_LINE][j]; //�ݴ�Ҷ�ֵ�������ظ�Ѱַ

                //����㷨����sigma
                if (grayLevel < i)
                {
                    n1++;
                    g1 += grayLevel;
                }
                else
                    g2 += grayLevel;
            }
            n2 = MT9V03X_W - n1;
            p1 = (float) (n1) / MT9V03X_W;
            p2 = 1 - p1;
            e1 = (float) (g1) / n1;
            e2 = (float) (g2) / n2;
            e12 = e1 - e2;
            sigma = p1 * p2 * e12 * e12;

            if (max_sigma < sigma) //ȡsigmaֵ���ĻҶ���Ϊ��ֵ
            {
                max_sigma = sigma;
                maxSig_e2 = e2; //��¼��ʱ��e2��Ϊ�ο��׵�
            }
        }
        display_e2 = maxSig_e2; //��ʾ�ο��׵���ֵ

        startLine_length = MT9V03X_H; //��ʼ����ʼ�߳���

        for (int i = 0; i < MT9V03X_W; i++) //������ʼ��
        {
            int j;
            for (j = MT9V03X_H - 1; j >= 0; j--) //������������
            {
                if (CAMERA_THRESHOLD < diff_div_sum((uint8) maxSig_e2, mt9v03x_image[j][i])) //��������ɫ���أ�ȡ��ʱjֵ
                    break;
            }
            if (startLine_length > (uint8) j) //��jԽ����ͼ�񶥲�
            {
                startLine_length = (uint8) j; //��¼��ʱ����
                startLine = (uint8) i; //��¼��ʱ����
            }
        }
        display_startLine = startLine; //��ʾ��ʼ��
        display_startLine_length = startLine_length; //��ʾ��ʼ�߳���

        for (int i = startLine_length; i < MT9V03X_H; i++) //��������ʼ�ߣ������������߽�
        {
            int j;
            for (j = startLine; j >= 0; j--) //���������߽�
            {
                if (CAMERA_THRESHOLD < diff_div_sum(mt9v03x_image[i][j], mt9v03x_image[i][j - 5])) //��������ɫ���أ�ȡ��ʱjֵ
                    break;
            }
            leftLine[i] = (uint8) j;

            for (j = startLine; j < MT9V03X_W; j++) //���������߽�
            {
                if (CAMERA_THRESHOLD < diff_div_sum(mt9v03x_image[i][j], mt9v03x_image[i][j + 5])) //��������ɫ���أ�ȡ��ʱjֵ
                    break;
            }
            rightLine[i] = (uint8) j;
        }

        mt9v03x_finish_flag = 0; //���ͼ��ɼ���־λ

        centerError = 0; //�������������

        for (int i = startLine_length; i < MT9V03X_H; i++)
        {
            center = (leftLine[i] + rightLine[i]) / 2; //������������
            centerLine[i] = center; //�洢��������
            centerError += center; //�ۼ���������λ��
        }
        centerError -= MT9V03X_W * (MT9V03X_H - startLine_length) / 2; //��ȥ����������λ��
        centerError /= MT9V03X_H - startLine_length; //��ƽ��

        pos = centerError; //���λ��ƫ��

        //���ٿ���
        M0_set_speed = overall_speed * (1 + (float) pos / POS_MAP);
        M1_set_speed = overall_speed * (1 - (float) pos / POS_MAP);
    }
}

//��Ⱥ��㷨
int16 diff_div_sum (uint8 a, uint8 b)
{
    int16 diff;
    int16 sum;
    int16 ans;

    diff = a - b;
    sum = a + b;
    ans = (int16) (diff << 7 / sum);

    return ans;
}

#pragma section all restore
