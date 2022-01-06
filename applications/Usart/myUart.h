#ifndef __MYUART_H__
#define __MYUART_H__
#include <rtthread.h>
#include <stdio.h>

#define MY_USE_UART         "uart2"
#define PRINTF_TXT_SIZE     64

struct Uart_Msg
{
	rt_device_t dev;
	rt_size_t size;
};

typedef struct
{
    rt_uint16_t head;   //数据头
    rt_uint8_t cmd;     //命令
    float  data1;       //数据1
    float  data2;       //数据2
    rt_uint16_t tail;   //数据尾

} Ratio_motor_Cmd_t;

//数据包共用体，方便操作
union uart_cmd
{
	Ratio_motor_Cmd_t cmd;
	rt_uint8_t buff[sizeof(Ratio_motor_Cmd_t)];
};

typedef void (*Func_Uart_Recv)(const Ratio_motor_Cmd_t *data);

rt_err_t MyUart_Init(void);
void MyUart_Send(const void *buffer,rt_size_t size);
void Register_MyUart_Recv_Callback(Func_Uart_Recv func);
char *Get_PrintfTxt(void);
void MyUart_Send_PrintfTxt(void);
void MyUart_Send_PrintfString(const char *s);

#endif
