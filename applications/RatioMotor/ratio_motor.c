#include "ratio_motor.h"
#include "HCanID_data.h"
#include "drv_canthread.h"
#include <math.h>
#include "myUart.h"

static Motor_t M3508;
static Target_t target_point[TARGET_NUM] = {
    [0].kind = PATROL_POS_START , [0].err_precision = DEFAULT_ERR_PRECISION, [0].arrive_cb = RT_NULL,
    [1].kind = PATROL_POS_END   , [1].err_precision = DEFAULT_ERR_PRECISION, [1].arrive_cb = RT_NULL,
    [2].kind = MOTOR_SET        , [2].err_precision = DEFAULT_ERR_PRECISION, [2].arrive_cb = RT_NULL,
};

static rt_uint8_t en_angle_loop = 1;   //置1为开启角度闭环

static Adjust_t RatioM_adjust = {
    .state = ADJ_IDLE_ERROR,
    .cnt = 0,
    .timeout_cnt = 0,
    .pos_max = DEFAULT_POS_MAX,
    .pos_min = DEFAULT_POS_MIN,
    .complete = RT_NULL,
};

static void Target_Callback_Process(Target_t *aim)
{
    RT_ASSERT(aim != RT_NULL);

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

void Target_Set_Pos(float pos, Target_e kind)
{
    /* 检查边界合理性 */
    RT_ASSERT(RatioM_adjust.pos_max > RatioM_adjust.pos_min);

    /* 角度补偿设定值 */
    pos += RatioM_adjust.pos_min;

    /* 输入限幅 */
    VALUE_CLAMP(pos,RatioM_adjust.pos_min,RatioM_adjust.pos_max);

    target_point[kind].pos = pos;
}

void Register_Target_Callback(Func_Arrive func, Target_e kind)
{
    RT_ASSERT(func != RT_NULL);
    target_point[kind].arrive_cb = func;
}

void Target_Set_Precision(float precision, Target_e kind)
{
    target_point[kind].err_precision = precision;
}

static struct rt_semaphore ratio_motor_sem;	
static void Ratio_Motor_IRQHandler(void *parameter)
{
    rt_sem_release(&ratio_motor_sem);
}
static void Ratio_Motor_Thread(void *parameter)
{
    while(1)
    {
        rt_sem_take(&ratio_motor_sem, RT_WAITING_FOREVER);

        if (en_angle_loop == 1)
        {
            Motor_AnglePIDCalculate(&M3508,Motor_Read_NowAngle(&M3508));
            Motor_Write_SetSpeed_ABS(&M3508,M3508.ang.out);
        }

        Motor_SpeedPIDCalculate(&M3508,Motor_Read_NowSpeed(&M3508));

        motor_current_send(can1_dev,STDID_launch,Motor_Read_OutSpeed(&M3508),0,0,0);
    }
}

/* 单独开一个线程,提高电机闭环的实时性 */
static void Ratio_Motor_Callback_Thread(void *parameter)
{
    while(1)
    {
        if (en_angle_loop == 1)
        {
            for (int i = 0; i < TARGET_NUM; i++)
                Target_Callback_Process(&target_point[i]);
        }
        rt_thread_mdelay(1);
    }
}

static void Motor3508_Init(void)
{
    motor_init(&M3508,MOTOR_ID_1,RATIO_MOTOR_MOTOR_RATIO,ANGLE_CTRL_FULL,8192,180,-180);//不选择ANGLE_CTRL_EXTRA,取消角度闭环的就近原则
    pid_init(&M3508.ang,30,0,0,1000,RATIO_MOTOR_MAX_SPEED, -RATIO_MOTOR_MAX_SPEED);
    pid_init(&M3508.spe,15,0.05,10,1000,10000,-10000);

    /*等待电机第一次通信完毕*/
	while(M3508.dji.Data_Valid == 0) 
    {
        rt_thread_mdelay(50);
    }
}

rt_err_t Ratio_Motor_Init(void)
{
    rt_err_t res;
    rt_thread_t thread = RT_NULL;

    Motor3508_Init();

    //初始化信号量
    res = rt_sem_init(&ratio_motor_sem, "RatioM_sem", 0, RT_IPC_FLAG_FIFO);
    if ( res != RT_EOK)
		return res;

    //初始化线程
    thread = rt_thread_create("RatioM_Thread",Ratio_Motor_Thread,RT_NULL,1024,11,10);
    if(thread == RT_NULL)
        return RT_ERROR;

    if(rt_thread_startup(thread) != RT_EOK)
        return RT_ERROR;

    //创建线程定时器
    rt_timer_t ratio_motor_timer = rt_timer_create("RatioM_timer",Ratio_Motor_IRQHandler,
                                                RT_NULL,RATIO_MOTOR_PERIOD,
                                                RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    res = rt_timer_start(ratio_motor_timer);
    if ( res != RT_EOK)
		return res;
        
    //初始化线程
    thread = rt_thread_create("Canis_cb",Ratio_Motor_Callback_Thread,RT_NULL,1024,13,10);
    if(thread == RT_NULL)
        return RT_ERROR;

    if(rt_thread_startup(thread) != RT_EOK)
        return RT_ERROR;
	
    return RT_EOK;
}

void Ratio_Motor_Refresh_Motor(struct rt_can_msg *msg)
{
    RT_ASSERT(msg != RT_NULL);
    motor_readmsg(msg,&M3508.dji);
}

/* angle: 单位度 */
/* 正方向为电机输出轴的逆时针方向 */
void Ratio_Motor_Set_Position(float angle)
{
    //电机校准完毕,才允许外部控制电机
    if (RatioM_Adjust_If_Finsh() == RT_TRUE)
    {
        /* 检查边界合理性 */
        RT_ASSERT(RatioM_adjust.pos_max > RatioM_adjust.pos_min);

        Target_Set_Pos(angle,MOTOR_SET);

        /* 角度补偿设定值 */
        angle += RatioM_adjust.pos_min;

        /* 输入限幅 */
        VALUE_CLAMP(angle,RatioM_adjust.pos_min,RatioM_adjust.pos_max);

        Motor_Write_SetAngle_ABS(&M3508,angle);
    }
    else
        MyUart_Send_PrintfString("[Ratio Motor]: The motor is not properly calibrated.\n");
}

float Ratio_Motor_Read_NowPos(void)
{
    float angle = Motor_Read_NowAngle(&M3508);

    /* 角度补偿设定值 */
    angle -= RatioM_adjust.pos_min;

    return angle;
}

float Ratio_Motor_Read_Pos_Range(void)
{
    /* 检查边界合理性 */
    RT_ASSERT(RatioM_adjust.pos_max > RatioM_adjust.pos_min);

    /* 角度补偿设定值 */
    return RatioM_adjust.pos_max - RatioM_adjust.pos_min;
}

void Ratio_Motor_Set_MaxSpeed(float out_limit)
{
    out_limit = fabs(out_limit);
    VALUE_CLAMP(out_limit,0,RATIO_MOTOR_MAX_SPEED);

    M3508.ang.out_limit_up = out_limit;
    M3508.ang.out_limit_down = - out_limit;
}

void Ratio_Motor_Set_MaxCurrent(float out_limit)
{
    out_limit = fabs(out_limit);
    VALUE_CLAMP(out_limit,0,RATIO_MOTOR_MAX_CURRENT);

    M3508.spe.out_limit_up = out_limit;
    M3508.spe.out_limit_down = - out_limit;
}

/* 调试pid用 */
const Motor_t* Ratio_Motor_Read_MotorData(void)
{
    return &M3508;
}

static void Adjust_Running_Normal(float offset)
{
    /* 速度回归正常,开始补偿 */
    RatioM_adjust.cnt -= 10;
    if (RatioM_adjust.cnt < 0)
    RatioM_adjust.cnt = 0;

    /* 未出现掉速一段时间 */
    RatioM_adjust.timeout_cnt ++;
    if (RatioM_adjust.timeout_cnt > ADJUST_TIMEOUT)
    {
        en_angle_loop = 1;  //开启角度闭环
        Motor_Write_SetAngle_ABS(&M3508,Motor_Read_NowAngle(&M3508) + offset);//停在原地
        MyUart_Send_PrintfString("[adjust]: Motor calibration timeout!\n");
        RatioM_adjust.state = ADJ_ERROR;
    }
}

//speed_run校准速度,带方向,> 0 逆时针
static void Adjust_Start_Process(float speed_run)
{
    RatioM_adjust.cnt = 0;
    RatioM_adjust.timeout_cnt = 0;

    en_angle_loop = 0;//开启速度闭环
    Motor_Write_SetSpeed_ABS(&M3508, speed_run);
}

static void Adjust_Clockwise_Process(void)
{
    if (M3508.spe.err < - ADJUST_SPEED_RUN/2) 
    {
        RatioM_adjust.cnt ++;
        //顺时针堵转
        if (RatioM_adjust.cnt > ADJUST_TIME)
        {
            en_angle_loop = 1;  //开启角度闭环
            RatioM_adjust.pos_min = Motor_Read_NowAngle(&M3508) + ADJUST_POS_MARGIN;

            //停在最小值附近,预留一段距离
            Motor_Write_SetAngle_ABS(&M3508,RatioM_adjust.pos_min);
            RatioM_adjust.state = ADJ_COUNTER_CLOCKWISE;
        }
    }
    else
        Adjust_Running_Normal(ADJUST_POS_MARGIN);
}

static void Adjust_CounterClockwise_Process(void)
{
    if (M3508.spe.err > ADJUST_SPEED_RUN/2)
    {
        RatioM_adjust.cnt ++;
        //逆时针堵转
        if (RatioM_adjust.cnt > ADJUST_TIME)
        {
            en_angle_loop = 1;  //开启角度闭环
            RatioM_adjust.pos_max = Motor_Read_NowAngle(&M3508) - ADJUST_POS_MARGIN;

            //停在最大值附近,预留一段距离
            Motor_Write_SetAngle_ABS(&M3508,RatioM_adjust.pos_max);
            RatioM_adjust.state = ADJ_SUCCESS;
        }
    }
    else
        Adjust_Running_Normal(- ADJUST_POS_MARGIN);
}

static void Adjust_Success_Process(void)
{
    //检查正负性
    if (RatioM_adjust.pos_max < RatioM_adjust.pos_min)
    {
        MyUart_Send_PrintfString("[adjust]: The motor calibration result is incorrect!\n");
        RatioM_adjust.state = ADJ_ERROR;
        return;
    }

    MyUart_Send_PrintfString("[adjust]: Calibrate the motor successfully!\n");
    if (RatioM_adjust.complete != RT_NULL)
        (*RatioM_adjust.complete)(RatioM_adjust.pos_max - RatioM_adjust.pos_min);
    
    RatioM_adjust.state = ADJ_IDLE;
}

static void RatioM_Adjust_Thread(void *parameter)
{
    while(1)
    {
        switch (RatioM_adjust.state)
        {
        case ADJ_IDLE:
        case ADJ_IDLE_ERROR:
            rt_thread_mdelay(1);
            break;

        case ADJ_CLOCKWISE:
            Adjust_Start_Process(- ADJUST_SPEED_RUN);
            rt_thread_mdelay(400);//延时一段时间,以便达到设定转速
            RatioM_adjust.state = ADJ_CLOCKWISE_RUNNING;
            break;
        
        case ADJ_CLOCKWISE_RUNNING:
            Adjust_Clockwise_Process();
            rt_thread_mdelay(1);
            break;

        case ADJ_COUNTER_CLOCKWISE:
            Adjust_Start_Process(ADJUST_SPEED_RUN);
            rt_thread_mdelay(400);//延时一段时间,以便达到设定转速
            RatioM_adjust.state = ADJ_COUNTER_CLOCKWISE_RUNNING;
            break;
        
        case ADJ_COUNTER_CLOCKWISE_RUNNING:
            Adjust_CounterClockwise_Process();
            rt_thread_mdelay(1);
            break;
        
        case ADJ_ERROR:
            RatioM_adjust.pos_max = DEFAULT_POS_MAX;
            RatioM_adjust.pos_min = DEFAULT_POS_MIN;
            RatioM_adjust.state = ADJ_IDLE_ERROR;
            break;
        
        case ADJ_SUCCESS:
            Adjust_Success_Process();
            break;
        }
    }
}

rt_err_t RatioM_Adjust_Init(void)
{
    //初始化线程
    rt_thread_t thread = rt_thread_create("Adjust_thread",RatioM_Adjust_Thread,RT_NULL,1024,13,10);
    if(thread == RT_NULL)
        return RT_ERROR;

    if(rt_thread_startup(thread) != RT_EOK)
        return RT_ERROR;
    
	return RT_EOK;
}

void RatioM_Adjust_Start(void)
{
    if (RatioM_adjust.state == ADJ_IDLE || RatioM_adjust.state == ADJ_IDLE_ERROR)
    {
        MyUart_Send_PrintfString("[adjust]: Now start calibrating the motor.\n");
        RatioM_adjust.state = ADJ_CLOCKWISE;
    }
    else
        MyUart_Send_PrintfString("[adjust]: The calibration function is enabled.\n");
}

//RT_TRUE为校准完成,RT_FALSE为正在校准,校准失败,未开始校准
rt_bool_t RatioM_Adjust_If_Finsh(void)
{
    if (RatioM_adjust.state == ADJ_IDLE)
        return RT_TRUE;
    else
        return RT_FALSE;
}

void Register_Adjust_Callback(void (*func)(float))
{
    RT_ASSERT(func != RT_NULL);
    RatioM_adjust.complete = func;
}

/* 检查val是否在校准后的数据范围之内,RT_TRUE为在其之内 */
rt_bool_t Judge_In_Adjust_Range(float val)
{
    /* 检查边界合理性 */
    RT_ASSERT(RatioM_adjust.pos_max > RatioM_adjust.pos_min);

    /* 角度补偿设定值 */
    val += RatioM_adjust.pos_min;

    if (val > RatioM_adjust.pos_max)
        return RT_FALSE;
    else if (val < RatioM_adjust.pos_min)
        return RT_FALSE;

    return RT_TRUE;
}
