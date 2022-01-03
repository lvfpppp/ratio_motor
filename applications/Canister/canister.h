#ifndef __CANISTER_H__
#define __CANISTER_H__
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_motor.h"

#define MAX_3508_RPM        (469.0f)
#define MOTOR_3508_RATIO    (3591.0f/187.0f)    //机械结构传动比
#define CANISTER_PERIOD     (1)                 //定时器周期，单位ms

#define POSITION_MAX        (180)               //最大角度限位，单位度
#define POSITION_MIN        (0)

#define ANGLE_ERR_PRECISION (2.5)                //默认辨识精度,单位度

typedef enum
{
    PATROL_START = 0,
    PATROL_END = 1,
    MOTOR_SET = 2,
    
    TARGET_NUM,
} Target_e;

typedef struct
{
    Target_e kind;
    float pos;
    rt_uint8_t flag;    //用于触发一次回调

    float err_precision;      //角度误差精度
    void (*arrive_cb)(Motor_t *,Target_e);

} Target_t;

typedef void (*Func_Arrive)(Motor_t *,Target_e) ;

rt_err_t Canister_Init(void);
void Canister_Refresh_Motor(struct rt_can_msg *msg);
void Canister_Set_Position(float angle);

void Target_Set_Pos(float pos, Target_e kind);
void Target_Register_Callback(Func_Arrive func, Target_e kind);
void Target_Set_Precision(float precision, Target_e kind);



#endif

