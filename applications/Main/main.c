#include "drv_canthread.h"
#include "canister.h"
#include <board.h>

#define CAN_RS_PIN      GET_PIN(A,15)
#include <stdio.h>
#include <math.h>

#define PATROL_START    0
#define PATROL_END      180

Patrol_e now_pos;

/* 到达设定值的回调函数 */
void Canister_Set_Callback(Motor_t *motor,Patrol_e kind)
{
    /* 输出到达设定值的当前角度值 */
    char txt[20];
    sprintf(txt,"now angle: %3.3f",Motor_Read_NowAngle(motor));
    rt_kprintf("%s\n",txt);

    now_pos = kind;
}

void Test_Canister(void)
{
    Patrol_Set_StartPos(PATROL_START);
    Patrol_Set_EndPos(PATROL_END);

    Register_StartSet_Callback(Canister_Set_Callback);
    Register_EndSet_Callback(Canister_Set_Callback);

    Canister_Init();

    static rt_uint8_t _cnt = 0;

    while(1)
    {
        if (now_pos == P_START)
        {
            Canister_Set_Position(PATROL_END);
            rt_thread_mdelay(2000);
        }
        else if(now_pos == P_END)
        {
            Canister_Set_Position(PATROL_START);
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
