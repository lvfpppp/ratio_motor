#include "canister.h"
#include "myUart.h"
#include "patrol.h"
#include "uart_agree.h"

extern void Board_Base_Init(void);

/* 在同一个回调函数中,尽量用一个MyUart_Send函数。不然会因为保护锁把原来非阻塞式发送的串口变为阻塞。*/
static void MyUart_Send_String(const char *s)
{
    const rt_uint8_t size = rt_strlen(s);

    rt_strncpy(printf_txt,s,size);
    MyUart_Send(printf_txt,size);
}


/* 电机pid的角度pid达到设定值附近时,反馈数据 */
static void PID_Arrive_Callback(Motor_t const *motor,Target_e kind)
{
    /* 输出到达设定值的当前角度值 */
    sprintf(printf_txt,"PID steady state: %3.3f (angle)\n",Canister_Read_NowPos());
    MyUart_Send(printf_txt,rt_strlen(printf_txt));
}

static void Feedback_Part_Init(void)
{
    Patrol_Init();

    Target_Set_Precision(1,MOTOR_SET);//修改辨识到达目标精度为 1度
    Register_Target_Callback(PID_Arrive_Callback,MOTOR_SET);
}
////////////////////////////////////////////////////////////////////
static void Adjust_Complete_Callback(float range)
{
    MyUart_Send_String("motor adjust result:\n");
    MyUart_Send_String("min angle: 0\n");

    sprintf(printf_txt,"max angle: %3.3f\n",range);
    MyUart_Send(printf_txt,rt_strlen(printf_txt));
}

/* 上电默认校准,直至校准完毕 */
static void Adjust_Part_Init(void)
{
    Register_Adjust_Callback(Adjust_Complete_Callback);

    Canister_Adjust_Init();
    Canister_Adjust_Start();

    //等待校准完毕
    while(Canister_Get_Adjust_State() != 0){
        rt_thread_mdelay(1);
    }

    // Canister_Set_Position(0);//停在0位
}
////////////////////////////////////////////////////////////////////

/* 所有功能的主函数 */
int all_fun_main(void)
{
    /* 串口2初始化 */
    rt_err_t res = MyUart_Init();
    RT_ASSERT(res == RT_EOK);

    Board_Base_Init();

    Adjust_Part_Init();

    Feedback_Part_Init();

    MyUart_Agree_Init();

    return 0;
}
