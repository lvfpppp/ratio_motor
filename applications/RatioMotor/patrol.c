#include "patrol.h"
#include "myUart.h"

static patrol_t patrol = {
    .now_state = PATROL_IDLE,
    .now_endpoint = PATROL_POS_START,
    .start_pos = DEFAULT_PATROL_START_POS,
    .end_pos = DEFAULT_PATROL_END_POS,
    .cnt = 0,
};

/**
 * @brief   巡逻模块的状态转移函数
 * @param   next 下一个状态
 */
static void Patrol_State_Transfer(enum patrol_state_e next)
{
    patrol.now_state = next;
}

/**
 * @brief   到达端点值的回调函数
 * @note    利用达到两个端点的角度值时,触发的回调函数,设置另一个端点角度值为角度闭环设定值。
 * @param   kind 当前所在的端点类型
 */
static void Patrol_Arrive_Callback(Motor_t const *motor,Target_e kind)
{
    RT_ASSERT(motor != RT_NULL);

    /* 只有在非空闲和关闭态下,才进入暂停态 */
    if (patrol.now_state != PATROL_IDLE && patrol.now_state != PATROL_CLOSE)
    {
        if (kind == PATROL_POS_START)
        {
            MyUart_Send_PrintfString("}}}}}}}=================>>>\n");
            sprintf(Get_PrintfTxt(),"[patrol]: now in start (angle %06.3f)\n",Ratio_Motor_Read_NowPos());
            MyUart_Send_PrintfTxt();
        }
        else if (kind == PATROL_POS_END)
        {
            MyUart_Send_PrintfString("<<<================={{{{{{{\n");
            sprintf(Get_PrintfTxt(),"[patrol]: now in end   (angle %06.3f)\n",Ratio_Motor_Read_NowPos());
            MyUart_Send_PrintfTxt();
        }

        /* 到达目标点后迁入暂停态 */
        patrol.now_endpoint = kind;
        Patrol_State_Transfer(PATROL_PAUSE);
    }
    else
    {
        patrol.now_endpoint = kind;//没在工作时,记录位置
    }
}

/**
 * @brief   巡逻状态迁移到另一个端点的状态
 */
static void Patrol_Move_Opposite(void)
{
    if (patrol.now_endpoint == PATROL_POS_START)
    {
        //如果当前在起点位置,则往终点位置移动
        Patrol_State_Transfer(MOVE_TO_END);
    }
    else if (patrol.now_endpoint == PATROL_POS_END)
    {
        //如果当前在终点位置,则往起点位置移动
        Patrol_State_Transfer(MOVE_TO_START);
    }
    else
        RT_ASSERT(0);//不允许赋值其他状态
}

/**
 * @brief   巡逻线程
 */
static void Patrol_Thread(void *parameter)
{
    while(1)
    {
        switch (patrol.now_state)
        {
        case PATROL_IDLE:
            break;

        case PATROL_CLOSE:
            Ratio_Motor_Set_Position(Ratio_Motor_Read_NowPos());//停在原地
            Patrol_State_Transfer(PATROL_IDLE);//马上转入空闲态,以防一直卡在原地
            break;

        case PATROL_PAUSE:
            //当电机达到端点时，会先停在原地不动 PATROL_PAUSE_TIME ms
            patrol.cnt ++;
            if (patrol.cnt >= PATROL_PAUSE_TIME)
            {
                patrol.cnt = 0;
                Patrol_Move_Opposite();
            }
            break;

        case MOVE_TO_START:
            Ratio_Motor_Set_Position(patrol.start_pos);
            break;

        case MOVE_TO_END:
            Ratio_Motor_Set_Position(patrol.end_pos);
            break;

        default:
            break;
        }

        rt_thread_mdelay(1);
    }
}

/**
 * @brief   巡逻功能初始化
 * @return  错误码
 * @author  lfp 
 */
rt_err_t Patrol_Init(void)
{
    Target_Set_Pos(patrol.start_pos,PATROL_POS_START);
    Register_Target_Callback(Patrol_Arrive_Callback,PATROL_POS_START);

    Target_Set_Pos(patrol.end_pos,PATROL_POS_END);
    Register_Target_Callback(Patrol_Arrive_Callback,PATROL_POS_END);

    rt_thread_t thread = rt_thread_create("Patrol_Thread",Patrol_Thread,RT_NULL,1024,18,10);
    if(thread == RT_NULL)
        return RT_ERROR;

    if(rt_thread_startup(thread) != RT_EOK)
        return RT_ERROR;
    
	return RT_EOK;
}

/**
 * @brief   巡逻功能关闭
 */
void Patrol_Fun_Close(void)
{
    /* 只有在不是关闭和空闲态才能关闭,不能反复关闭 */
    if (patrol.now_state != PATROL_CLOSE && patrol.now_state != PATROL_IDLE)
        Patrol_State_Transfer(PATROL_CLOSE);
    else
        MyUart_Send_PrintfString("[patrol]: Patrol has been shut down.\n");
}

/**
 * @brief   巡逻功能打开
 */
void Patrol_Fun_Open(void)
{
    //只有当校准完毕,才能开始巡逻
    if (RatioM_Adjust_If_Finsh() == RT_TRUE)
    {
        //检查,巡逻范围是否在校准的范围之内
        if (Judge_In_Adjust_Range(patrol.start_pos) && Judge_In_Adjust_Range(patrol.end_pos)) 
        {
            MyUart_Send_PrintfString("[patrol]: ready ... ...\n");
            Patrol_Move_Opposite();
        }
        else
            MyUart_Send_PrintfString("[patrol]: range is not within the range of calibration.\n");
        
        sprintf(Get_PrintfTxt(),"[patrol]: start angle %03.3f, end angle %03.3f\n",patrol.start_pos,patrol.end_pos);
        MyUart_Send_PrintfTxt();
    }
    else
        MyUart_Send_PrintfString("[patrol]: The motor is not properly calibrated successfully.\n");
}

/**
 * @brief   配置巡逻的端点角度值
 * @note    输入的角度值必须在电机校准的角度范围之内
 * @param   start 起点角度值,单位度
 * @param   end   终点角度值,单位度
 */
void Patrol_Set_Pos(float start,float end)
{
    /* 输入限幅到电机校准后的角度值范围之内 */
    float ratio_motor_max = Ratio_Motor_Read_Pos_Range();
    VALUE_CLAMP(start,0,ratio_motor_max);
    VALUE_CLAMP(end,0,ratio_motor_max);

    patrol.start_pos = start;
    patrol.end_pos = end;

    /* 重新设置角度闭环靶点 */
    Target_Set_Pos(patrol.start_pos,PATROL_POS_START);
    Target_Set_Pos(patrol.end_pos,PATROL_POS_END);
}

/**
 * @brief   获得巡逻是否完成
 * @return  RT_TRUE为巡逻完成,RT_FALSE为正在巡逻
 */
rt_bool_t Patrol_If_Finsh(void)
{
    if (patrol.now_state == PATROL_IDLE)
        return RT_TRUE;
    else
        return RT_FALSE;
}
