#ifndef __PATROL_H__
#define __PATROL_H__
#include "ratio_motor.h"

#define DEFAULT_PATROL_START_POS    0    //默认的起始点角度值,单位度
#define DEFAULT_PATROL_END_POS      90   //默认的终点角度值,单位度

#define PATROL_PAUSE_TIME   (1000)       //到达顶点后停顿时间,单位ms


enum patrol_state_e
{
    PATROL_IDLE,
    PATROL_CLOSE,
    PATROL_PAUSE,
    MOVE_TO_START,
    MOVE_TO_END,
};

typedef struct
{
    enum patrol_state_e now_state;  //当前状态
    Target_e now_endpoint;  //当前所在的端点位置

    float start_pos;    //起始点角度值
    float end_pos;      //终点角度值
    rt_uint32_t cnt;    //延时计数,提高状态迁移的响应速度
     
} patrol_t;


rt_err_t Patrol_Init(void);
void Patrol_Fun_Close(void);
void Patrol_Fun_Open(void);
void Patrol_Set_Pos(float start,float end);
rt_bool_t Patrol_If_Finsh(void);

#endif
