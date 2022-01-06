#ifndef __PATROL_H__
#define __PATROL_H__
#include "ratio_motor.h"


#define DEFAULT_PATROL_START_POS    0    //单位度
#define DEFAULT_PATROL_END_POS      90   //单位度

#define PATROL_PAUSE_TIME   (1000)       //单位ms,到达顶点后停顿时间


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
    enum patrol_state_e now_state;
    Target_e now_endpoint;

    float start_pos;
    float end_pos;
    rt_uint32_t cnt;
     
} patrol_t;


rt_err_t Patrol_Init(void);
void Patrol_Fun_Close(void);
void Patrol_Fun_Open(void);
void Patrol_Set_Pos(float start,float end);

#endif
