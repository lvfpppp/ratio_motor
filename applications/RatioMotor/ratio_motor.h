#ifndef __RATIO_MOTOR_H__
#define __RATIO_MOTOR_H__
#include "drv_motor.h"

#define RATIO_MOTOR_MAX_3508RPM    (469.0f)
#define RATIO_MOTOR_MOTOR_RATIO    (3591.0f/187.0f)   //机械结构传动比
#define RATIO_MOTOR_PERIOD         (1)                //定时器周期，单位ms
#define RATIO_MOTOR_MAX_SPEED      (RATIO_MOTOR_MAX_3508RPM*RATIO_MOTOR_MOTOR_RATIO)
#define RATIO_MOTOR_MAX_CURRENT    (16384)

#define DEFAULT_POS_MAX         (180)              //最大角度限位，单位度
#define DEFAULT_POS_MIN         (0)

#define DEFAULT_ERR_PRECISION   (2.5)              //默认辨识精度,单位度

#define ADJUST_SPEED_RUN        (500)              //校准时的速度,单位小转子的rpm
#define ADJUST_TIME             (300)              //校准判断时长,单位ms
#define ADJUST_TIMEOUT          (((RATIO_MOTOR_MOTOR_RATIO/ADJUST_SPEED_RUN)*60*1000)/2)    //校准超时判断时长,单位ms
#define ADJUST_POS_MARGIN       (2)                //堵转校准后预留的位置余量,单位度

/* 限制数值范围 */
#define VALUE_CLAMP(val,min,max)    do{\
                                        if ((val) >= (max)) {\
                                            (val) = (max);\
                                        }\
                                        else if ((val) <= (min)) {\
                                            (val) = (min);\
                                        }\
                                    }while(0);

typedef enum
{
    PATROL_POS_START = 0,
    PATROL_POS_END = 1,
    MOTOR_SET = 2,
    
    TARGET_NUM,
} Target_e;

typedef void (*Func_Arrive)(Motor_t const *,Target_e) ;
typedef struct
{
    Target_e kind;
    float pos;
    rt_uint8_t flag;    //用于触发一次回调

    float err_precision;      //角度误差精度
    Func_Arrive arrive_cb;

} Target_t;

typedef enum
{
    ADJ_IDLE,

    ADJ_CLOCKWISE,
    ADJ_CLOCKWISE_RUNNING,
    ADJ_COUNTER_CLOCKWISE,
    ADJ_COUNTER_CLOCKWISE_RUNNING,

    ADJ_ERROR,
    ADJ_SUCCESS,

} Adjust_e;

typedef struct
{
    Adjust_e state;
    
    rt_int32_t  cnt;
    rt_int32_t  timeout_cnt;

    float pos_max;
    float pos_min;
    void (*complete)(float);

} Adjust_t;


rt_err_t Ratio_Motor_Init(void);
void Ratio_Motor_Refresh_Motor(struct rt_can_msg *msg);
void Ratio_Motor_Set_Position(float angle);
void Ratio_Motor_Set_MaxSpeed(float out_limit);
void Ratio_Motor_Set_MaxCurrent(float out_limit);
float Ratio_Motor_Read_NowPos(void);
float Ratio_Motor_Read_Pos_Range(void);
const Motor_t* Ratio_Motor_Read_MotorData(void);

void Target_Set_Pos(float pos, Target_e kind);
void Register_Target_Callback(Func_Arrive func, Target_e kind);
void Target_Set_Precision(float precision, Target_e kind);

rt_err_t RatioM_Adjust_Init(void);
void RatioM_Adjust_Start(void);
rt_bool_t RatioM_Adjust_If_Finsh(void);
void Register_Adjust_Callback(void (*func)(float));

#endif

