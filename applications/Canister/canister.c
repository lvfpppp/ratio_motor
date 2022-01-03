#include "canister.h"
#include "HCanID_data.h"
#include "drv_canthread.h"
#include <math.h>

Motor_t M3508;
Target_t  target_point[TARGET_NUM];


static void Target_Callback_Process(Target_t *aim)
{
    /* 如果设定值到达 起始目标 附近, aim->flag 用作回调的单次触发 */
    if (fabs(Motor_Read_NowAngle(&M3508) - aim->pos) < aim->err_precision && aim->flag == 0)
    {
        if (aim->arrive_cb != RT_NULL)
            (*aim->arrive_cb)(&M3508,aim->kind);
        aim->flag = 1;
    }
    else if(fabs(Motor_Read_NowAngle(&M3508) - aim->pos) >= aim->err_precision && aim->flag == 1)
    {
        aim->flag = 0;
    }
}

static void Target_Init(Target_t *point, Target_e kind)
{
    point->kind = kind;
    point->pos = 0;
    point->flag = 0;
    point->err_precision = ANGLE_ERR_PRECISION;
    point->arrive_cb = RT_NULL;
}

void Target_Set_Pos(float pos, Target_e kind)
{
    target_point[kind].pos = pos;
}

void Target_Register_Callback(Func_Arrive func, Target_e kind)
{
    target_point[kind].arrive_cb = func;
}

void Target_Set_Precision(float precision, Target_e kind)
{
    target_point[kind].err_precision = precision;
}

static struct rt_semaphore canister_sem;	
static void Canister_IRQHandler(void *parameter)
{
    rt_sem_release(&canister_sem);
}
static void Canister_Thread(void *parameter)
{
    while(1)
    {
        rt_sem_take(&canister_sem, RT_WAITING_FOREVER);

        for (int i = 0;i<TARGET_NUM;i++)
            Target_Callback_Process(&target_point[i]);

        Motor_AnglePIDCalculate(&M3508,Motor_Read_NowAngle(&M3508));

        Motor_Write_SetSpeed_ABS(&M3508,M3508.ang.out);
		
        Motor_SpeedPIDCalculate(&M3508,Motor_Read_NowSpeed(&M3508));

        motor_current_send(can1_dev,STDID_launch,Motor_Read_OutSpeed(&M3508),0,0,0);
    }
}

static void Canister_Motor_Init(void)
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
    
    Target_Init(&target_point[PATROL_START],PATROL_START);
    Target_Init(&target_point[PATROL_END],PATROL_END);
    Target_Init(&target_point[MOTOR_SET],MOTOR_SET);
    Canister_Motor_Init();

    //初始化信号量
    res = rt_sem_init(&canister_sem, "canister_sem", 0, RT_IPC_FLAG_FIFO);
    if ( res != RT_EOK)
		return res;

    //初始化线程
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

void Canister_Refresh_Motor(struct rt_can_msg *msg)
{
    motor_readmsg(msg,&M3508.dji);
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

    Target_Set_Pos(angle,MOTOR_SET);
}

void Canister_Set_MaxSpeed(float out_limit)
{
    M3508.ang.out_limit_up = out_limit;
    M3508.ang.out_limit_down = - out_limit;
}

void Canister_Set_MaxCurrent(float out_limit)
{
    M3508.spe.out_limit_up = out_limit;
    M3508.spe.out_limit_down = - out_limit;
}
