/*
 注意事项：
 1.读写规范如下：
 读入字符数据：（参数标识）（更改值）
 如本示例应串口输入ac将paraA改为'c'，其中a即为paraA的参数标识。读入字符数据不加分号

 读入其它类型数据：（参数标识）（更改值）;
 如本示例应串口输入b1234;将paraB改为'1234'，其中b即为paraB的参数标识，末尾加表示结束的分号
 应串口输入c56.78;将paraC改为'56.78'，其中c即为paraC的参数标识，末尾加表示结束的分号

 2.若需要修改的参数类型不匹配，更改serialInt/serialFlo函数中形参data和变量ans（serialFlo还要改变量factor）的类型即可
 3.执行serial_io前后必须间隔一段时间（建议放在定时器中断里，间隔50ms），以使串口超时，防止代码一直执行if却不进入switch内
 4.由于希望和printf函数共用串口，而printf默认在DEBUG_UART使用，所以需要在zf_uart.c里改fputc函数中uart_putchar函数的uartn参数
 5.在本文件中extern入想要使用串口修改的参数，并配置serial_io函数以实现目的
 */

#include "headfile.h"
#include "user.h"

#pragma section all "cpu0_dsram"
uint8 paraChoice = '\0';
#pragma section all restore

extern int16 re_set_speed; //电机期望速度

void serial_io_init (void)
{
#if(0 == ENABLE_WIRELESS)
    //uart_init(DEBUG_UART, DEBUG_UART_BAUD, DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN); //初始化下载器串口
    //这里不初始化是因为get_clk函数内含有DEBUG_UART的初始化，不用重复初始化
#elif(1 == ENABLE_WIRELESS)
    //seekfree_wireless_init();
    //逐飞开源库的这个函数比下面这个单独初始化多做了两件事：
    //一是设置了无线类型，这会激活逐飞自家的无线读数据函数，数据都被那个函数读了我们自己的函数就读不到了
    //二是初始化了流控引脚，如果数据量少没必要用，如果传摄像头图像到上位机那还是自己配置一下
    uart_init(WIRELESS_UART, WIRELESS_UART_BAUD, WIRELESS_UART_TX, WIRELESS_UART_RX);    //初始化无线串口
#endif

    pit_interrupt_ms(CCU6_0, PIT_CH0, 50);    //CCU6定时器初始化
}

#pragma section all "cpu0_psram"

//brief 串口更改某一参数
//param uartn: 接收串口
//return void
void serial_io (UARTN_enum uartn)
{
    if (uart_query(uartn, &paraChoice)) //获取参数标识
    {
        switch (paraChoice)
        //根据参数标识选择修改哪个参数
        {
            case 'a' :
                serialInt(uartn, (uint16*)&re_set_speed);
                break;
        }
    }
}

//brief 串口字符串输入转整型
//param uartn: 接收串口
//param *data: 接收数据的地址
//return uint8: 1接收成功 0数据非法
uint8 serialInt (UARTN_enum uartn, uint16 *data)
{
    uint8 buffer; //暂存串口输入字符
    uint16 ans = 0; //暂存整型结果
    uint8 flag = 0; //负号标记，0未遇到，1已遇到

    while (uart_query(uartn, &buffer)) //一直读取串口数据
    {
        if (buffer == ';') //如果数据接受完毕
        {
            while (uart_query(uartn, &buffer))
                ; //读完剩余数据，防止影响下文

            if (flag) //如果含负号
                ans = -ans;

            *data = ans; //回传ans的值
            return 1;
        }
        else if (buffer == '-') //如果遇到负号
        {
            flag = 1;
            continue; //'-'不能参加运算，跳过此次循环读取下一个数字字符
        }
        else if (buffer < '0' || buffer > '9') //如果数据非法
        {
            while (uart_query(uartn, &buffer))
                ; //读完剩余数据，防止影响下文
            return 0;
        }

        ans *= 10; //进位
        ans += buffer - '0'; //字符转整型
    }

    return 0; //数据未加结束符';'
}

//brief 串口字符串输入转浮点型
//param uartn: 接收串口
//param *data: 接收数据的地址
//return uint8: 1接收成功 0数据非法
uint8 serialFlo (UARTN_enum uartn, float32 *data)
{
    uint8 buffer; //暂存串口输入字符
    float32 ans = 0; //暂存浮点型结果
    float32 factor = 1; //小数因子
    uint8 flag = 0; //小数点标记，0未遇到，1已遇到

    while (uart_query(uartn, &buffer)) //一直读取串口数据
    {
        if (buffer == ';') //如果数据接受完毕
        {
            while (uart_query(uartn, &buffer))
                ; //读完剩余数据，防止影响下文
            *data = ans; //回传ans的值
            return 1;
        }
        else if (buffer == '.') //如果遇到小数点
        {
            flag = 1;
            continue; //'.'不能参加运算，跳过此次循环读取下一个数字字符
        }
        else if (buffer < '0' || buffer > '9') //如果数据非法
        {
            while (uart_query(uartn, &buffer))
                ; //读完剩余数据，防止影响下文
            return 0;
        }

        if (flag) //已遇到小数点
        {
            factor *= 0.1; //小数因子进入下一位
            ans += factor * (buffer - '0'); //字符转浮点型
        }
        else //未遇到小数点
        {
            ans *= 10; //进位
            ans += buffer - '0'; //字符转整型
        }
    }

    return 0; //数据未加结束符';'
}

#pragma section all restore
