#include "myUart.h"

static struct rt_messagequeue my_uart_mq;
static rt_uint8_t msg_pool[64];
static rt_device_t MyUart_Serial;

static Func_Uart_Recv recv_process_p = RT_NULL; //串口接收一帧数据后产生的处理回调
static union uart_cmd my_agree;

static char printf_txt[PRINTF_TXT_SIZE]; //全局的打印缓存，需要注意互斥保护
static struct rt_mutex printf_txt_mutex;

/**
 * @brief   串口中断产生的回调函数
 */
static rt_err_t MyUart_Callback(rt_device_t dev, rt_size_t size)
{
	struct Uart_Msg msg;
	rt_err_t result;
	msg.dev = dev;
	msg.size = size;

	result = rt_mq_send(&my_uart_mq, &msg, sizeof(msg));
	if (result == -RT_EFULL)
	{
		MyUart_Send_PrintfString("[my_uart]: message queue full\n");
	}
	return result;
}

/**
 * @brief   串口处理线程
 */
static void MyUart_Process_thread(void *parameter)
{
	struct Uart_Msg msg;
	rt_err_t result;

	while (1)
	{
		rt_memset(&msg, 0, sizeof(msg));
        //等待回调函数的消息队列
		result = rt_mq_recv(&my_uart_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            rt_device_read(msg.dev, 0, my_agree.buff, msg.size);
            if (recv_process_p != RT_NULL)
                (*recv_process_p)(&my_agree.cmd);
        }
    }
}

/**
 * @brief   串口初始化
 * @return  错误码
 */
rt_err_t MyUart_Init(void)
{
    rt_err_t res = RT_EOK;

    /* 初始化打印的互斥保护 */
    res = rt_mutex_init(&printf_txt_mutex,"print_txt",RT_IPC_FLAG_PRIO);
	if ( res != RT_EOK)
		return res;	

    /* 使用默认的串口配置，配置为波特率 115200,8位数据位,1位停止位,无校验位 */
    MyUart_Serial = rt_device_find(MY_USE_UART);
	if (!MyUart_Serial)
		return RT_ERROR;

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
												RT_NULL, 1024, 15, 10);
    if (MyUart_Thread != RT_NULL)
        rt_thread_startup(MyUart_Thread);
    else
        return RT_ERROR;

    return RT_EOK;
}

/**
 * @brief   注册串口帧数据处理回调
 * @param   func 待注册函数
 */
void Register_MyUart_Recv_Callback(Func_Uart_Recv func)
{
    recv_process_p = func;
}

/**
 * @brief   串口发送
 * @param   buffer 缓存数据
 * @param   size 数据大小
 */
void MyUart_Send(const void *buffer,rt_size_t size)
{
    rt_device_write(MyUart_Serial,0,buffer,size);
}

/**
 * @brief   获得全局的打印缓冲数组地址
 * @note    一般在调用该函数后，紧接着调用MyUart_Send_PrintfTxt函数(涉及到锁的释放)
 * @return  数组地址
 */
char *Get_PrintfTxt(void)
{
    if (rt_mutex_take(&printf_txt_mutex,RT_WAITING_FOREVER) == RT_EOK)
    {
        rt_memset(printf_txt,'\0',PRINTF_TXT_SIZE); //用之前先清空
        return printf_txt;
    }
    else
    {
        RT_ASSERT(0);//一般不会失败,便不做特殊处理
        return RT_NULL;
    }
}

/**
 * @brief   串口发送PrintfTxt缓冲数组数据
 */
void MyUart_Send_PrintfTxt(void)
{
    rt_device_write(MyUart_Serial,0,printf_txt,rt_strlen(printf_txt));
    rt_thread_mdelay(10);//不使用延时会有bug(后发送的字符会覆盖前面的),目前不清楚原因

    rt_mutex_release(&printf_txt_mutex);
}

/**
 * @brief   串口发送字符串常量
 * @param   s 待发送字符串
 */
void MyUart_Send_PrintfString(const char *s)
{
    const rt_uint8_t size = rt_strlen(s);
    rt_strncpy(Get_PrintfTxt(),s,size);

    MyUart_Send_PrintfTxt();
}
