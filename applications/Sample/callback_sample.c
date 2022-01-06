#include "ratio_motor.h"
#include <stdio.h>

extern void Board_Base_Init(void);

#define P_START_POS    0    //单位度
#define P_END_POS      45   //单位度

static Target_e now_pos;

/* 到达设定值的回调函数 */
static void Patrol_Callback(Motor_t const *motor,Target_e kind)
{
    /* 输出到达设定值的当前角度值 */
    char txt[10];
    sprintf(txt,"%03.3f",Ratio_Motor_Read_NowPos());
    
    if (kind == PATROL_POS_START)
        rt_kprintf("Patrol Start: %s\n",txt);
    else if (kind == PATROL_POS_END)
        rt_kprintf("Patrol End  : %s\n",txt);

    now_pos = kind;
}

/* 电机pid的角度pid达到设定值附近时,反馈数据 */
static void PID_Arrive_Callback(Motor_t const *motor,Target_e kind)
{
    /* 输出到达设定值的当前角度值 */
    char txt[10];
    sprintf(txt,"%03.3f",Ratio_Motor_Read_NowPos());
    rt_kprintf("PID: %s\n",txt);
}

/* 能实现电机的巡逻功能,并在到达目标点时,反馈数据 */
void Test_Ratio_motor(void)
{
    Target_Set_Pos(P_START_POS,PATROL_POS_START);
    Register_Target_Callback(Patrol_Callback,PATROL_POS_START);

    Target_Set_Pos(P_END_POS,PATROL_POS_END);
    Register_Target_Callback(Patrol_Callback,PATROL_POS_END);

    Target_Set_Precision(1,MOTOR_SET);//修改辨识到达目标精度为 1度
    Register_Target_Callback(PID_Arrive_Callback,MOTOR_SET);

    while(1)
    {
        if (now_pos == PATROL_POS_START)
        {
            Ratio_Motor_Set_Position(P_END_POS);
            rt_thread_mdelay(2000);
        }
        else if(now_pos == PATROL_POS_END)
        {
            Ratio_Motor_Set_Position(P_START_POS);
            rt_thread_mdelay(2000);
        }

        rt_thread_mdelay(1);
    }
}

/* 根据 P_START_POS 和 P_END_POS 设定的起点和终点角度值，来回巡逻。
    在到达起点和终点后，会进入相应的回调函数
    (rt_kprintf函数需要设置串口2为控制台串口)
*/
int callback_main(void)
{
    Board_Base_Init();

	Test_Ratio_motor();

    return 0;
}
