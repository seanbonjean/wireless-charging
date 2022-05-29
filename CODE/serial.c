/*
 ע�����
 1.��д�淶���£�
 �����ַ����ݣ���������ʶ��������ֵ��
 �籾ʾ��Ӧ��������ac��paraA��Ϊ'c'������a��ΪparaA�Ĳ�����ʶ�������ַ����ݲ��ӷֺ�

 ���������������ݣ���������ʶ��������ֵ��;
 �籾ʾ��Ӧ��������b1234;��paraB��Ϊ'1234'������b��ΪparaB�Ĳ�����ʶ��ĩβ�ӱ�ʾ�����ķֺ�
 Ӧ��������c56.78;��paraC��Ϊ'56.78'������c��ΪparaC�Ĳ�����ʶ��ĩβ�ӱ�ʾ�����ķֺ�

 2.����Ҫ�޸ĵĲ������Ͳ�ƥ�䣬����serialInt/serialFlo�������β�data�ͱ���ans��serialFlo��Ҫ�ı���factor�������ͼ���
 3.ִ��serial_ioǰ�������һ��ʱ�䣨������ڶ�ʱ���ж�����50ms������ʹ���ڳ�ʱ����ֹ����һֱִ��ifȴ������switch��
 4.����ϣ����printf�������ô��ڣ���printfĬ����DEBUG_UARTʹ�ã�������Ҫ��zf_uart.c���fputc������uart_putchar������uartn����
 5.�ڱ��ļ���extern����Ҫʹ�ô����޸ĵĲ�����������serial_io������ʵ��Ŀ��
 */

#include "headfile.h"
#include "user.h"

#pragma section all "cpu0_dsram"
uint8 paraChoice = '\0';
#pragma section all restore

extern int16 re_set_speed; //��������ٶ�

void serial_io_init (void)
{
#if(0 == ENABLE_WIRELESS)
    //uart_init(DEBUG_UART, DEBUG_UART_BAUD, DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN); //��ʼ������������
    //���ﲻ��ʼ������Ϊget_clk�����ں���DEBUG_UART�ĳ�ʼ���������ظ���ʼ��
#elif(1 == ENABLE_WIRELESS)
    //seekfree_wireless_init();
    //��ɿ�Դ�������������������������ʼ�������������£�
    //һ���������������ͣ���ἤ������Լҵ����߶����ݺ��������ݶ����Ǹ��������������Լ��ĺ����Ͷ�������
    //���ǳ�ʼ�����������ţ������������û��Ҫ�ã����������ͷͼ����λ���ǻ����Լ�����һ��
    uart_init(WIRELESS_UART, WIRELESS_UART_BAUD, WIRELESS_UART_TX, WIRELESS_UART_RX);    //��ʼ�����ߴ���
#endif

    pit_interrupt_ms(CCU6_0, PIT_CH0, 50);    //CCU6��ʱ����ʼ��
}

#pragma section all "cpu0_psram"

//brief ���ڸ���ĳһ����
//param uartn: ���մ���
//return void
void serial_io (UARTN_enum uartn)
{
    if (uart_query(uartn, &paraChoice)) //��ȡ������ʶ
    {
        switch (paraChoice)
        //���ݲ�����ʶѡ���޸��ĸ�����
        {
            case 'a' :
                serialInt(uartn, (uint16*)&re_set_speed);
                break;
        }
    }
}

//brief �����ַ�������ת����
//param uartn: ���մ���
//param *data: �������ݵĵ�ַ
//return uint8: 1���ճɹ� 0���ݷǷ�
uint8 serialInt (UARTN_enum uartn, uint16 *data)
{
    uint8 buffer; //�ݴ洮�������ַ�
    uint16 ans = 0; //�ݴ����ͽ��
    uint8 flag = 0; //���ű�ǣ�0δ������1������

    while (uart_query(uartn, &buffer)) //һֱ��ȡ��������
    {
        if (buffer == ';') //������ݽ������
        {
            while (uart_query(uartn, &buffer))
                ; //����ʣ�����ݣ���ֹӰ������

            if (flag) //���������
                ans = -ans;

            *data = ans; //�ش�ans��ֵ
            return 1;
        }
        else if (buffer == '-') //�����������
        {
            flag = 1;
            continue; //'-'���ܲμ����㣬�����˴�ѭ����ȡ��һ�������ַ�
        }
        else if (buffer < '0' || buffer > '9') //������ݷǷ�
        {
            while (uart_query(uartn, &buffer))
                ; //����ʣ�����ݣ���ֹӰ������
            return 0;
        }

        ans *= 10; //��λ
        ans += buffer - '0'; //�ַ�ת����
    }

    return 0; //����δ�ӽ�����';'
}

//brief �����ַ�������ת������
//param uartn: ���մ���
//param *data: �������ݵĵ�ַ
//return uint8: 1���ճɹ� 0���ݷǷ�
uint8 serialFlo (UARTN_enum uartn, float32 *data)
{
    uint8 buffer; //�ݴ洮�������ַ�
    float32 ans = 0; //�ݴ渡���ͽ��
    float32 factor = 1; //С������
    uint8 flag = 0; //С�����ǣ�0δ������1������

    while (uart_query(uartn, &buffer)) //һֱ��ȡ��������
    {
        if (buffer == ';') //������ݽ������
        {
            while (uart_query(uartn, &buffer))
                ; //����ʣ�����ݣ���ֹӰ������
            *data = ans; //�ش�ans��ֵ
            return 1;
        }
        else if (buffer == '.') //�������С����
        {
            flag = 1;
            continue; //'.'���ܲμ����㣬�����˴�ѭ����ȡ��һ�������ַ�
        }
        else if (buffer < '0' || buffer > '9') //������ݷǷ�
        {
            while (uart_query(uartn, &buffer))
                ; //����ʣ�����ݣ���ֹӰ������
            return 0;
        }

        if (flag) //������С����
        {
            factor *= 0.1; //С�����ӽ�����һλ
            ans += factor * (buffer - '0'); //�ַ�ת������
        }
        else //δ����С����
        {
            ans *= 10; //��λ
            ans += buffer - '0'; //�ַ�ת����
        }
    }

    return 0; //����δ�ӽ�����';'
}

#pragma section all restore
