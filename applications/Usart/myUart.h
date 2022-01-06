#ifndef __MYUART_H__
#define __MYUART_H__
#include <rtthread.h>
#include <stdio.h>

#define MY_USE_UART "uart2"


/* 串口驱动信息 */
struct Uart_Msg
{
	rt_device_t dev;
	rt_size_t size;
};

typedef struct
{
    rt_uint16_t head;
    rt_uint8_t cmd;
    float  data1;
    float  data2;
    rt_uint16_t tail;

} Canister_Cmd_t;

union uart_cmd
{
	Canister_Cmd_t cmd;
	rt_uint8_t buff[sizeof(Canister_Cmd_t)];
};

typedef void (*Func_Uart_Recv)(const Canister_Cmd_t *data);

rt_err_t MyUart_Init(void);
void MyUart_Send(const void *buffer,rt_size_t size);
void Register_MyUart_Recv_Callback(Func_Uart_Recv func);
char *Get_PrintfTxt(void);
void MyUart_Send_PrintfTxt(void);
void MyUart_Send_PrintfString(const char *s);

#endif
