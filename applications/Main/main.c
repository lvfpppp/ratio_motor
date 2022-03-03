#include "drv_canthread.h"
#include <board.h>
#include "ratio_motor.h"
#include "myUart.h"

// #define EN_ADJUST_TEST      //adjust_sample.c
// #define EN_PID_DEBUG        //PID_sample.c
// #define EN_UART_CMD         //cmd_sample.c
#define EN_ALL_FUN          //all_fun_sample.c

extern int adjust_main(void);
extern int pid_main(void);
extern int cmd_main(void);
extern int all_fun_main(void);


#define CAN_RS_PIN     GET_PIN(A,15)


void Board_Base_Init(void)
{
    rt_err_t res = RT_EOK;
    
    /* 串口2初始化 */
    res = MyUart_Init();
    RT_ASSERT(res == RT_EOK);

    /* can硬件电路要求,can_rs引脚拉低 */
    rt_pin_mode(CAN_RS_PIN, PIN_MODE_OUTPUT_OD);
    rt_pin_write(CAN_RS_PIN, PIN_LOW);
		
    /* RS485硬件电路,默认将EN引脚拉低，处于接收状态 */
	rt_pin_mode(RS485_EN_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(RS485_EN_PIN, PIN_LOW);
		
    /* can初始化 */
	res = Can1_Init();
    RT_ASSERT(res == RT_EOK);

    /* 电机初始化 */
    Ratio_Motor_Init();
//		while(1)//串口测试
//		{
//			MyUart_Send_PrintfString("[CMD 1]: Hello World.\n");
//            
//			rt_thread_mdelay(1000);
//		}
}

int main(void)
{
#if defined(EN_ADJUST_TEST)
    adjust_main();
#elif defined(EN_PID_DEBUG)
    pid_main();
#elif defined(EN_UART_CMD)
    cmd_main();
#elif defined(EN_ALL_FUN)
    all_fun_main();
#endif
}
