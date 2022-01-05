#include "myUart.h"

struct rt_messagequeue my_uart_mq;
rt_uint8_t msg_pool[256];
rt_device_t MyUart_Serial;
//TODO:全部加上static

const rt_uint8_t cmd_length = sizeof(Canister_Cmd_t);
Func_Uart_Recv recv_process_p = RT_NULL;


static rt_err_t MyUart_Callback(rt_device_t dev, rt_size_t size)
{
	struct Uart_Msg msg;
	rt_err_t result;
	msg.dev = dev;
	msg.size = size;

	result = rt_mq_send(&my_uart_mq, &msg, sizeof(msg));
	if (result == -RT_EFULL)
	{
		rt_kprintf("my_uart: message queue full\n");
	}
	return result;
}


static void MyUart_Process_thread(void *parameter)
{
	struct Uart_Msg msg;
	rt_err_t result;
    union uart_cmd canister_u;

	while (1)
	{
		rt_memset(&msg, 0, sizeof(msg));
		result = rt_mq_recv(&my_uart_mq, &msg, sizeof(msg), RT_WAITING_FOREVER); //等待回调函数的消息队列
				
        if (result == RT_EOK)
        {
            rt_device_read(msg.dev, 0, canister_u.buff, msg.size);
            if (recv_process_p != RT_NULL)
                (*recv_process_p)(&canister_u.cmd);
        }
    }
}


rt_err_t MyUart_Init(void)
{
    rt_err_t res = RT_EOK;

    /* 使用默认的串口配置，配置为波特率 115200,8位数据位,1位停止位,无校验位 */
    MyUart_Serial = rt_device_find(MY_USE_UART);
	if (!MyUart_Serial)
    {
        rt_kprintf("rt_device_find my UART failed !\n");
		return RT_ERROR;
    }

	/* 初始化消息队列 */
    res = rt_mq_init(&my_uart_mq, "my_uart_mq",
               msg_pool,                
               sizeof(struct Uart_Msg),  
               sizeof(msg_pool),      
               RT_IPC_FLAG_FIFO);  
	if ( res != RT_EOK)
		return res;	

	/* DMA 接收模式 */	      
    res = rt_device_open(MyUart_Serial, RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX);
	if ( res != RT_EOK)
		return res;	

    res = rt_device_set_rx_indicate(MyUart_Serial, MyUart_Callback);
	if ( res != RT_EOK)
		return res;

    rt_thread_t MyUart_Thread = rt_thread_create("MyUart_Thread", MyUart_Process_thread, 
												RT_NULL, 1024, 7, 10);
    if (MyUart_Thread != RT_NULL)
    {
        rt_thread_startup(MyUart_Thread);
    }
    else
    {
        return RT_ERROR;
    }

    return RT_EOK;
}


void MyUart_Send(const void *buffer,rt_size_t size)
{
    rt_device_write(MyUart_Serial,0,buffer,size);
}

void Register_MyUart_Recv_Callback(Func_Uart_Recv func)
{
    recv_process_p = func;
}
