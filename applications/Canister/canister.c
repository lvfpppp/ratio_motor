#include "canister.h"
#include "HCanID_data.h"
#include "drv_canthread.h"
#include <math.h>
#include "myUart.h"

static Motor_t M3508;
static Target_t target_point[TARGET_NUM];//改变设置初值方式,TODO:
static rt_uint8_t en_angle_loop = 1;   //置1为开启角度闭环
static rt_uint8_t en_pos_adjust = 0;   //置1为开启位置的校准,TODO:改名

static Adjust_t canister_adjust = {
    .pos_max = DEFAULT_POS_MAX,
    .pos_min = DEFAULT_POS_MIN,
    .range = 0,//TODO:
    .adjust_complete = RT_NULL,
};

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
    point->err_precision = DEFAULT_ERR_PRECISION;
    point->arrive_cb = RT_NULL;
}

void Target_Set_Pos(float pos, Target_e kind)
{
    /* 角度补偿设定值 */
    pos += canister_adjust.pos_min;

    /* 输入限幅 */
    if (pos >= canister_adjust.pos_max)
        pos = canister_adjust.pos_max;
    else if (pos <= canister_adjust.pos_min)
        pos = canister_adjust.pos_min;

    target_point[kind].pos = pos;
}

void Register_Target_Callback(Func_Arrive func, Target_e kind)
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

        if (en_angle_loop == 1)
        {
            for (int i = 0;i<TARGET_NUM;i++)
                Target_Callback_Process(&target_point[i]);

            Motor_AnglePIDCalculate(&M3508,Motor_Read_NowAngle(&M3508));
            Motor_Write_SetSpeed_ABS(&M3508,M3508.ang.out);
        }

        Motor_SpeedPIDCalculate(&M3508,Motor_Read_NowSpeed(&M3508));

        motor_current_send(can1_dev,STDID_launch,Motor_Read_OutSpeed(&M3508),0,0,0);
    }
}

static void Canister_Motor_Init(void)
{
    motor_init(&M3508,MOTOR_ID_1,CANISTER_MOTOR_RATIO,ANGLE_CTRL_FULL,8192,180,-180);//不选择ANGLE_CTRL_EXTRA,取消角度闭环的就近原则
    pid_init(&M3508.ang,30,0,0,1000,CANISTER_MAX_3508RPM*CANISTER_MOTOR_RATIO,-CANISTER_MAX_3508RPM*CANISTER_MOTOR_RATIO);
    pid_init(&M3508.spe,15,0.05,10,1000,10000,-10000);

    /*等待电机第一次通信完毕*/
	while(M3508.dji.Data_Valid == 0) 
    {
        rt_thread_mdelay(50);
    }
}

rt_err_t Canister_Init(void)
{
    rt_err_t res;
    rt_thread_t thread = RT_NULL;
    
    Target_Init(&target_point[PATROL_POS_START],PATROL_POS_START);
    Target_Init(&target_point[PATROL_POS_END],PATROL_POS_END);
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
/* 正方向为电机输出轴的逆时针方向 */
void Canister_Set_Position(float angle)
{
    Target_Set_Pos(angle,MOTOR_SET);
    
    /* 角度补偿设定值 */
    angle += canister_adjust.pos_min;

    /* 输入限幅 */
    if (angle >= canister_adjust.pos_max)
        angle = canister_adjust.pos_max;
    else if (angle <= canister_adjust.pos_min)
        angle = canister_adjust.pos_min;

    Motor_Write_SetAngle_ABS(&M3508,angle);
}

float Canister_Read_NowPos(void)
{
    float angle = Motor_Read_NowAngle(&M3508);

    /* 角度补偿设定值 */
    angle -= canister_adjust.pos_min;

    return angle;
}

void Canister_Set_MaxSpeed(float out_limit)
{
    out_limit = fabs(out_limit);
    M3508.ang.out_limit_up = out_limit;
    M3508.ang.out_limit_down = - out_limit;
}

void Canister_Set_MaxCurrent(float out_limit)
{
    out_limit = fabs(out_limit);
    M3508.spe.out_limit_up = out_limit;
    M3508.spe.out_limit_down = - out_limit;
}

/* 调试pid用 */
const Motor_t* Canister_Read_MotorData(void)
{
    return &M3508;
}

static float Cansiter_Adjust_Pos(float speed_run)
{
    static rt_int32_t _cnt = 0;
    static rt_int32_t timeout_cnt = 0;

    en_angle_loop = 0;//开启速度闭环

    Motor_Write_SetSpeed_ABS(&M3508,speed_run);
    rt_thread_mdelay(400);//延时一段时间,以便达到设定转速

    while(1)
    {
        // if (fabs(M3508.dji.speed) < fabs(ADJUST_SPEED_STOP))
        if (fabs(M3508.spe.err) > fabs(speed_run/2))
        {
            _cnt ++;
            /* 堵转 */
            if (_cnt > ADJUST_TIME)
            {
                _cnt = 0;
                timeout_cnt = 0;

                en_angle_loop = 1;  //开启角度闭环
                Motor_Write_SetAngle_ABS(&M3508,Motor_Read_NowAngle(&M3508));
                // Canister_Set_Position(Canister_Read_NowPos());//停在当前的位置,TODO:待整理重复代码

                return Motor_Read_NowAngle(&M3508);
            }
        }
        else
        {
            /* 速度回归正常,开始补偿 */
            _cnt -= 10;
            if (_cnt < 0)
                _cnt = 0;

            /* 未出现掉速一段时间 */
            timeout_cnt ++;
            if (timeout_cnt > ADJUST_TIMEOUT)
            {
                _cnt = 0;
                timeout_cnt = 0;
                
                en_angle_loop = 1;  //开启角度闭环
                Motor_Write_SetAngle_ABS(&M3508,Motor_Read_NowAngle(&M3508));
                // Canister_Set_Position(Canister_Read_NowPos());//停在当前的位置,TODO:待整理重复代码

                en_pos_adjust = 2;//异常
                rt_strncpy(printf_txt,"Motor calibration timeout!\n",29);//TODO:检查29
                MyUart_Send(printf_txt,rt_strlen(printf_txt));
                return 0;
            }
        }
        rt_thread_mdelay(1);
    }
}

static void Adjust_Thread(void *parameter)
{
    float temp_pos; //保护pos_min

    while(1)
    {
        if (en_pos_adjust == 1)
        {
            temp_pos = Cansiter_Adjust_Pos(-ADJUST_SPEED_RUN);
            if (en_pos_adjust == 2)
                goto _adjust_error;
            canister_adjust.pos_min = temp_pos;

            canister_adjust.pos_max = Cansiter_Adjust_Pos(ADJUST_SPEED_RUN);
            if (en_pos_adjust == 2)
            {
                canister_adjust.pos_max = DEFAULT_POS_MAX;
                canister_adjust.pos_min = DEFAULT_POS_MIN;
                goto _adjust_error;
            }

            canister_adjust.range = canister_adjust.pos_max - canister_adjust.pos_min;
            
            en_pos_adjust = 0;  //校准完毕

            if (canister_adjust.adjust_complete != RT_NULL)
                (*canister_adjust.adjust_complete)(canister_adjust.range);
        }

    _adjust_error:
        rt_thread_mdelay(1);
    }
}

rt_err_t Canister_Adjust_Init(void)
{
    //初始化线程
    rt_thread_t thread = rt_thread_create("Adjust_Thread",Adjust_Thread,RT_NULL,1024,15,10);
    if(thread == RT_NULL)
        return RT_ERROR;

    if(rt_thread_startup(thread) != RT_EOK)
        return RT_ERROR;
    
	return RT_EOK;
}

void Canister_Adjust_Start(void)
{
    en_pos_adjust = 1;
}

//1为开启位置的校准,0为校准结束或者未开始校准,2是校准失败
rt_uint8_t Canister_Get_Adjust_State(void)
{
    return en_pos_adjust;
}

void Register_Adjust_Callback(void (*func)(float))
{
    canister_adjust.adjust_complete = func;
}
