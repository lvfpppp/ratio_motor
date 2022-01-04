#include "drv_canthread.h"
#include <board.h>
#include "canister.h"
#include <stdio.h>


#define CAN_RS_PIN     GET_PIN(A,15)

#define P_START_POS    0    //单位度
#define P_END_POS      45   //单位度

Target_e now_pos;

/* 到达设定值的回调函数 */
void Patrol_Callback(Motor_t const *motor,Target_e kind)
{
    /* 输出到达设定值的当前角度值 */
    char txt[10];
    sprintf(txt,"%3.3f",Canister_Read_NowPos());
    
    if (kind == PATROL_START)
        rt_kprintf("Patrol Start: %s\n",txt);
    else if (kind == PATROL_END)
        rt_kprintf("Patrol End  : %s\n",txt);

    now_pos = kind;
}

/* 电机pid的角度pid达到设定值附近时,反馈数据 */
void PID_Arrive_Callback(Motor_t const *motor,Target_e kind)
{
    /* 输出到达设定值的当前角度值 */
    char txt[10];
    sprintf(txt,"%3.3f",Canister_Read_NowPos());
    rt_kprintf("PID: %s\n",txt);
}

/* 能实现电机的巡逻功能,并在到达目标点时,反馈数据 */
void Test_Canister(void)
{
    Target_Set_Pos(P_START_POS,PATROL_START);
    Register_Target_Callback(Patrol_Callback,PATROL_START);

    Target_Set_Pos(P_END_POS,PATROL_END);
    Register_Target_Callback(Patrol_Callback,PATROL_END);

    Target_Set_Precision(1,MOTOR_SET);
    Register_Target_Callback(PID_Arrive_Callback,MOTOR_SET);

    while(1)
    {
        if (now_pos == PATROL_START)
        {
            Canister_Set_Position(P_END_POS);
            rt_thread_mdelay(2000);
        }
        else if(now_pos == PATROL_END)
        {
            Canister_Set_Position(P_START_POS);
            rt_thread_mdelay(2000);
        }

        rt_thread_mdelay(1);
    }
}

/* 根据 P_START_POS 和 P_END_POS 设定的起点和终点角度值，来回巡逻。
    在到达起点和终点后，会进入相应的回调函数*/
int callback_main(void)
{
	rt_err_t res = RT_EOK;

    /* can硬件电路要求,can_rs引脚拉低 */
    rt_pin_mode(CAN_RS_PIN, PIN_MODE_OUTPUT_OD);
    rt_pin_write(CAN_RS_PIN, PIN_LOW);

	res = Can1_Init();
    RT_ASSERT(res == RT_EOK);

    Canister_Init();

	Test_Canister();

    return 0;
}
