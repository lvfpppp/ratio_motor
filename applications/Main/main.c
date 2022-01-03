#include "drv_canthread.h"
#include "canister.h"
#include <board.h>

#define CAN_RS_PIN     GET_PIN(A,15)
#include <stdio.h>
#include <math.h>

#define P_START_POS    30
#define P_END_POS      120

Target_e now_pos;

/* 到达设定值的回调函数 */
void Patrol_Callback(Motor_t *motor,Target_e kind)
{
    /* 输出到达设定值的当前角度值 */
    char txt[10];
    sprintf(txt,"%3.3f",Motor_Read_NowAngle(motor));
    
    if (kind == PATROL_START)
        rt_kprintf("Patrol Start: %s\n",txt);
    else if (kind == PATROL_END)
        rt_kprintf("Patrol End  : %s\n",txt);

    now_pos = kind;
}

void PID_Arrive_Callback(Motor_t *motor,Target_e kind)
{
    /* 输出到达设定值的当前角度值 */
    char txt[10];
    sprintf(txt,"%3.3f",Motor_Read_NowAngle(motor));
    rt_kprintf("PID: %s\n",txt);
}

void Test_Canister(void)
{
    Canister_Init();

    Target_Set_Pos(P_START_POS,PATROL_START);
    Target_Register_Callback(Patrol_Callback,PATROL_START);

    Target_Set_Pos(P_END_POS,PATROL_END);
    Target_Register_Callback(Patrol_Callback,PATROL_END);

    Target_Set_Precision(1,MOTOR_SET);
    Target_Register_Callback(PID_Arrive_Callback,MOTOR_SET);

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


int main(void)
{
	rt_err_t res = RT_EOK;

    /* can硬件电路要求,can_rs引脚拉低 */
    rt_pin_mode(CAN_RS_PIN, PIN_MODE_OUTPUT_OD);
    rt_pin_write(CAN_RS_PIN, PIN_LOW);

	res = Can1_Init();
    RT_ASSERT(res == RT_EOK);

	Test_Canister();
}
