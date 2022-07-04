#include "headfile.h"
#include "user.h"

#define GRAY_SCALE 256//灰度值范围

#pragma section all "cpu1_dsram"

int16 pos = 0; //记录位置偏差
boolean beep = FALSE; //蜂鸣器发声命令，置1发声，并由CPU0控制发声300ms后置0停止发声
boolean onRamp = FALSE; //IMU检测是否在坡道上，在坡道上时为TRUE

//参考白点计算
uint8 grayLevel; //暂存当前灰度值，避免重复寻址
uint8 minGray, maxGray; //最小/最大灰度值
uint8 n1, n2; //黑/白像素个数
uint16 g1, g2; //黑/白灰度累加值
float p1, p2; //黑/白像素概率
float e1, e2; //黑/白像素均值
float e12; //e1 - e2
float sigma; //该阈值对应sigma值
float max_sigma; //最大sigma值
float maxSig_e2; //最大sigma对应e2的值，即参考白点

//边界搜索起始线计算
uint8 startLine; //边界搜索起始线
uint8 startLine_length; //边界搜索起始线反方向长度

//赛道信息记录数组
uint8 leftLine[MT9V03X_H]; //左边界
uint8 rightLine[MT9V03X_H]; //右边界
uint8 centerLine[MT9V03X_H]; //赛道中线

//位置偏差计算
uint8 center; //暂存当前行中点，避免重复寻址
int16 centerError; //中心线误差

//差速控制
int16 overall_speed = 0; //全局速度
extern int16 M0_set_speed; //期望速度
extern int16 M1_set_speed;

//屏幕显示
float display_e2;
uint8 display_startLine;
uint8 display_startLine_length;

enum
{
    out, junction1, ramp, junction2, roundabout1, cross1, roundabout2, junction3, junction4, cross2, in
} carState; //赛道进程指示

#pragma section all restore

void getPosition_init (void)
{
    mt9v03x_init(); //摄像头初始化
}

#pragma section all "cpu1_psram"

void getPosition (void)
{
    if (mt9v03x_finish_flag) //当采集完成时
    {
        //seekfree_sendimg_03x(WIRELESS_UART, mt9v03x_image[0], MT9V03X_W, MT9V03X_H); //图像发送上位机

        //取该行第一个像素灰度作为灰度范围的初值
        minGray = mt9v03x_image[SAMPLE_LINE][0];
        maxGray = minGray;

        for (int i = 1; i < MT9V03X_W; i++) //遍历该行所有像素灰度，得到灰度范围
        {
            grayLevel = mt9v03x_image[SAMPLE_LINE][i];

            if (minGray > grayLevel) //更新最小灰度
                minGray = grayLevel;
            else if (maxGray < grayLevel) //更新最大灰度
                maxGray = grayLevel;
        }

        max_sigma = 0; //初始化最大sigma值

        for (int i = minGray; i <= maxGray; i++) //在灰度范围内遍历灰度
        {
            //初始化统计值
            n1 = 0;
            g1 = 0;
            g2 = 0;

            for (int j = 0; j < MT9V03X_W; j++) //对每一个假定的阈值，计算它的sigma
            {
                grayLevel = mt9v03x_image[SAMPLE_LINE][j]; //暂存灰度值，避免重复寻址

                //大津算法计算sigma
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

            if (max_sigma < sigma) //取sigma值最大的灰度作为阈值
            {
                max_sigma = sigma;
                maxSig_e2 = e2; //记录此时的e2作为参考白点
            }
        }
        display_e2 = maxSig_e2; //显示参考白点数值

        startLine_length = MT9V03X_H; //初始化起始线长度

        for (int i = 0; i < MT9V03X_W; i++) //搜索起始线
        {
            int j;
            for (j = MT9V03X_H - 1; j >= 0; j--) //从下往上搜索
            {
                if (CAMERA_THRESHOLD < diff_div_sum((uint8) maxSig_e2, mt9v03x_image[j][i])) //若遇到黑色像素，取此时j值
                    break;
            }
            if (startLine_length > (uint8) j) //若j越靠近图像顶部
            {
                startLine_length = (uint8) j; //记录此时行数
                startLine = (uint8) i; //记录此时列数
            }
        }
        display_startLine = startLine; //显示起始线
        display_startLine_length = startLine_length; //显示起始线长度

        for (int i = startLine_length; i < MT9V03X_H; i++) //沿搜索起始线，向左右搜索边界
        {
            int j;
            for (j = startLine; j >= 0; j--) //向左搜索边界
            {
                if (CAMERA_THRESHOLD < diff_div_sum(mt9v03x_image[i][j], mt9v03x_image[i][j - 5])) //若遇到黑色像素，取此时j值
                    break;
            }
            leftLine[i] = (uint8) j;

            for (j = startLine; j < MT9V03X_W; j++) //向右搜索边界
            {
                if (CAMERA_THRESHOLD < diff_div_sum(mt9v03x_image[i][j], mt9v03x_image[i][j + 5])) //若遇到黑色像素，取此时j值
                    break;
            }
            rightLine[i] = (uint8) j;
        }

        mt9v03x_finish_flag = 0; //清除图像采集标志位

        centerError = 0; //中心线误差置零

        for (int i = startLine_length; i < MT9V03X_H; i++)
        {
            center = (leftLine[i] + rightLine[i]) / 2; //计算赛道中线
            centerLine[i] = center; //存储赛道中线
            centerError += center; //累加赛道中线位置
        }
        centerError -= MT9V03X_W * (MT9V03X_H - startLine_length) / 2; //减去赛道中心线位置
        centerError /= MT9V03X_H - startLine_length; //求平均

        pos = centerError; //输出位置偏差

        //差速控制
        M0_set_speed = overall_speed * (1 + (float) pos / POS_MAP);
        M1_set_speed = overall_speed * (1 - (float) pos / POS_MAP);
    }
}

//差比和算法
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
