#include "canister.h"
#include "HCanID_data.h"
#include "drv_canthread.h"
#include <math.h>

Motor_t M3508;

patrol_t start = {
    .pos = 0,
    .flag = 0,
    .err_precision = ANGLE_ERR_PRECISION,
    .arrive_cb = RT_NULL,
};

patrol_t end = {
    .pos = 0,
    .flag = 0,
    .err_precision = ANGLE_ERR_PRECISION,
    .arrive_cb = RT_NULL,
};

static void _Callback_Process(void)//TODO:把start和end合并
{
    /* 如果设定值到达 起始目标 附近，start_flag 用作回调的单次触发 */
    if (fabs(Motor_Read_NowAngle(&M3508) - start.pos) < start.err_precision && start.flag == 0)
    {
        if (start.arrive_cb != RT_NULL)
            (*start.arrive_cb)(&M3508,P_START);
        start.flag = 1;
    }
    else if(fabs(Motor_Read_NowAngle(&M3508) - start.pos) >= start.err_precision && start.flag == 1)
    {
        start.flag = 0;
    }

    /* 如果设定值到达 起始目标 附近，end_flag 用作回调的单次触发 */
    if (fabs(Motor_Read_NowAngle(&M3508) - end.pos) < end.err_precision && end.flag == 0)
    {
        if (end.arrive_cb != RT_NULL)
            (*end.arrive_cb)(&M3508,P_END);
        end.flag = 1;
    }
    else if(fabs(Motor_Read_NowAngle(&M3508) - end.pos) >= end.err_precision && end.flag == 1)
    {
        end.flag = 0;
    }
}

void Patrol_Set_StartPos(float pos)
{
    start.pos = pos;
}

void Patrol_Set_EndPos(float pos)
{
    end.pos = pos;
}

void Register_StartSet_Callback(Func_Arrive func)
{
    start.arrive_cb = func;
}

void Register_EndSet_Callback(Func_Arrive func)
{
    end.arrive_cb = func;
}


static struct rt_semaphore 	canister_sem;	
static void Canister_IRQHandler(void *parameter)
{
    rt_sem_release(&canister_sem);
}
static void Canister_Thread(void *parameter)
{
    while(1)
    {
        rt_sem_take(&canister_sem, RT_WAITING_FOREVER);

        _Callback_Process();

        Motor_AnglePIDCalculate(&M3508,Motor_Read_NowAngle(&M3508));

        Motor_Write_SetSpeed_ABS(&M3508,M3508.ang.out);
		
        Motor_SpeedPIDCalculate(&M3508,Motor_Read_NowSpeed(&M3508));

        motor_current_send(can1_dev,STDID_launch,Motor_Read_OutSpeed(&M3508),0,0,0);
    }
    
}

/* angle: 单位度 */
void Canister_Set_Position(float angle)
{
    /* 输入限幅 */
    if (angle >= POSITION_MAX)
        angle = POSITION_MAX;
    else if (angle <= POSITION_MIN)
        angle = POSITION_MIN;

    Motor_Write_SetAngle_ABS(&M3508,angle);
}

void Motor_Set_MaxSpeed(float out_limit)
{
    M3508.ang.out_limit_up = out_limit;
    M3508.ang.out_limit_down = - out_limit;
}

void Motor_Set_MaxCurrent(float out_limit)
{
    M3508.spe.out_limit_up = out_limit;
    M3508.spe.out_limit_down = - out_limit;
}


static void Motor_Init(void)
{
    motor_init(&M3508,MOTOR_ID_1,MOTOR_3508_RATIO,ANGLE_CTRL_FULL,8192,180,-180);//不选择ANGLE_CTRL_EXTRA,取消角度闭环的就近原则
    pid_init(&M3508.ang,30,0,0,1000,MAX_3508_RPM*MOTOR_3508_RATIO,-MAX_3508_RPM*MOTOR_3508_RATIO);
    pid_init(&M3508.spe,10,0.1,0,1000,10000,-10000);

    /*等待电机第一次通信完毕*/
	while(M3508.dji.Data_Valid == 0) 
    {rt_thread_mdelay(50);}
}

rt_err_t Canister_Init(void)
{
    rt_err_t res;
    rt_thread_t thread = RT_NULL;
    
    Motor_Init();

    //初始化信号量
    res = rt_sem_init(&canister_sem, "canister_sem", 0, RT_IPC_FLAG_FIFO);
    if ( res != RT_EOK)
		return res;

    //初始化底盘线程
    thread = rt_thread_create("Canister_Thread",Canister_Thread,RT_NULL,1024,10,10);
    if(thread == RT_NULL)
        return RT_ERROR;

    if(rt_thread_startup(thread) != RT_EOK)
        return RT_ERROR;

    //创建线程定时器
    rt_timer_t canister_timer = rt_timer_create("canister_timer",Canister_IRQHandler,
                                                RT_NULL,CANISTER_PERIOD,
                                                RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    res = rt_timer_start(canister_timer);
    if ( res != RT_EOK)
		return res;
    
	return RT_EOK;
}

void Refresh_Motor(struct rt_can_msg *msg)
{
    motor_readmsg(msg,&M3508.dji);
}
