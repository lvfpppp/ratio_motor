#ifndef __MYUART_H__
#define __MYUART_H__
#include <rtthread.h>

#define MY_USE_UART "uart2"

/* 串口驱动信息 */
struct Uart_Msg
{
	rt_device_t dev;
	rt_size_t size;
};

rt_err_t MyUart_Init(void);
void MyUart_Send(const void *buffer,rt_size_t size);

#endif
