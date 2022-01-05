#ifndef __CANISTER_H__
#define __CANISTER_H__
#include "drv_motor.h"

#define CANISTER_MAX_3508RPM    (469.0f)
#define CANISTER_MOTOR_RATIO    (3591.0f/187.0f)   //机械结构传动比
#define CANISTER_PERIOD         (1)                //定时器周期，单位ms

#define DEFAULT_POS_MAX         (180)              //最大角度限位，单位度
#define DEFAULT_POS_MIN         (0)

#define DEFAULT_ERR_PRECISION   (2.5)              //默认辨识精度,单位度

#define ADJUST_SPEED_RUN        (500)              //校准时的速度,单位小转子的rpm
#define ADJUST_SPEED_STOP       (200)              //校准时的速度,单位小转子的rpm
#define ADJUST_TIME             (300)              //校准判断时长,单位ms

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

typedef struct
{
    float pos_min;
    float pos_max;
    float range;
    void (*adjust_complete)(float);

} Adjust_t;


rt_err_t Canister_Init(void);
void Canister_Refresh_Motor(struct rt_can_msg *msg);
void Canister_Set_Position(float angle);
void Canister_Set_MaxSpeed(float out_limit);
void Canister_Set_MaxCurrent(float out_limit);
float Canister_Read_NowPos(void);
const Motor_t* Canister_Read_MotorData(void);

void Target_Set_Pos(float pos, Target_e kind);
void Register_Target_Callback(Func_Arrive func, Target_e kind);
void Target_Set_Precision(float precision, Target_e kind);

void Canister_Adjust_Start(void);
rt_err_t Canister_Adjust_Init(void);
rt_uint8_t Canister_Get_Adjust_State(void);
void Register_Adjust_Callback(void (*func)(float));

#endif

