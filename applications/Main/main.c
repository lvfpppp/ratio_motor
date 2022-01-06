#include "drv_canthread.h"
#include <board.h>
#include "ratio_motor.h"

// #define EN_CALLBACK_TEST
// #define EN_ADJUST_TEST
// #define EN_PID_DEBUG
// #define EN_UART_CMD
#define EN_ALL_FUN

extern int callback_main(void);
extern int adjust_main(void);
extern int pid_main(void);
extern int cmd_main(void);
extern int all_fun_main(void);


#define CAN_RS_PIN     GET_PIN(A,15)

void Board_Base_Init(void)
{
    rt_err_t res = RT_EOK;

    /* can硬件电路要求,can_rs引脚拉低 */
    rt_pin_mode(CAN_RS_PIN, PIN_MODE_OUTPUT_OD);
    rt_pin_write(CAN_RS_PIN, PIN_LOW);

    /* can初始化 */
	res = Can1_Init();
    RT_ASSERT(res == RT_EOK);

    /* 电机初始化 */
    Ratio_Motor_Init();
}

//TODO:分析所有线程的优先级,校准时间过长,打印和停止
int main(void)
{
#if defined(EN_CALLBACK_TEST)
    callback_main();
#elif defined(EN_ADJUST_TEST)
    adjust_main();
#elif defined(EN_PID_DEBUG)
    pid_main();
#elif defined(EN_UART_CMD)
    cmd_main();
#elif defined(EN_ALL_FUN)
    all_fun_main();
#endif
}
