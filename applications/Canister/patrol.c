#include "patrol.h"
#include "myUart.h"

static patrol_t patrol = {
    .now_state = PATROL_IDLE,
    .now_endpoint = PATROL_POS_START,
    .start_pos = DEFAULT_PATROL_START_POS,
    .end_pos = DEFAULT_PATROL_END_POS,
    .cnt = 0,
};

static void Patrol_State_Transfer(enum patrol_state_e next)
{
    patrol.now_state = next;
}

/* 到达设定值的回调函数 */
static void Patrol_Arrive_Callback(Motor_t const *motor,Target_e kind)
{
    if (patrol.now_state != PATROL_IDLE && patrol.now_state != PATROL_CLOSE)
    {
        /* 输出到达设定值的当前角度值 */
        if (kind == PATROL_POS_START)
            sprintf(printf_txt,"patrol start: %3.3f (angle)\n",Canister_Read_NowPos());
        else if (kind == PATROL_POS_END)
            sprintf(printf_txt,"patrol end  : %3.3f (angle)\n",Canister_Read_NowPos());

        MyUart_Send(printf_txt,rt_strlen(printf_txt));

        /* 到达目标点后迁入暂停态 */
        patrol.now_endpoint = kind;
        Patrol_State_Transfer(PATROL_PAUSE);
    }
}

static void Patrol_Move_Opposite(void)
{
    if (patrol.now_endpoint == PATROL_POS_START) {
        //如果当前在起点位置,则往终点位置移动
        Patrol_State_Transfer(MOVE_TO_END);
    }
    else if (patrol.now_endpoint == PATROL_POS_END) {
        //如果当前在终点位置,则往起点位置移动
        Patrol_State_Transfer(MOVE_TO_START);
    }
    else
        RT_ASSERT(0);//不允许赋值其他状态
}

static void Patrol_Thread(void *parameter)
{
    while(1)
    {
        switch (patrol.now_state)
        {
        case PATROL_IDLE:
            break;

        case PATROL_CLOSE:
            Canister_Set_Position(Canister_Read_NowPos());//停在原地
            Patrol_State_Transfer(PATROL_IDLE);//马上转入空闲态,以防一直卡在原地
            break;

        case PATROL_PAUSE:
            patrol.cnt ++;
            if (patrol.cnt >= PATROL_PAUSE_TIME)
            {
                patrol.cnt = 0;
                Patrol_Move_Opposite();
            }
            break;

        case MOVE_TO_START:
            Canister_Set_Position(patrol.start_pos);
            break;

        case MOVE_TO_END:
            Canister_Set_Position(patrol.end_pos);
            break;

        default:
            break;
        }

        rt_thread_mdelay(1);
    }
}

rt_err_t Patrol_Init(void)
{
    Target_Set_Pos(patrol.start_pos,PATROL_POS_START);
    Register_Target_Callback(Patrol_Arrive_Callback,PATROL_POS_START);

    Target_Set_Pos(patrol.end_pos,PATROL_POS_END);
    Register_Target_Callback(Patrol_Arrive_Callback,PATROL_POS_END);

    //初始化线程
    rt_thread_t thread = rt_thread_create("Patrol_Thread",Patrol_Thread,RT_NULL,1024,12,10);
    if(thread == RT_NULL)
        return RT_ERROR;

    if(rt_thread_startup(thread) != RT_EOK)
        return RT_ERROR;
    
	return RT_EOK;
}

//巡逻功能关闭
void Patrol_Fun_Close(void)
{
    Patrol_State_Transfer(PATROL_CLOSE);
}

//巡逻功能打开
void Patrol_Fun_Open(void)
{
    Patrol_Move_Opposite();
}

void Patrol_Set_Pos(float start,float end)
{
    patrol.start_pos = start;
    patrol.end_pos = end;

    Target_Set_Pos(patrol.start_pos,PATROL_POS_START);
    Target_Set_Pos(patrol.end_pos,PATROL_POS_END);
}
