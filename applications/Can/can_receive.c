#include "can_receive.h"
#include "HCanID_data.h"
#include "drv_motor.h"

extern Motor_t M3508;

/***
* @brief    根据can1接收到的ID进入不同的处理函数
* @param    msg can1报文 
* @return   None
***/
void can1_rec(struct rt_can_msg *msg)
{
    switch(msg->id)
    {
        case MOTOR_ID_1:
            motor_readmsg(msg,&M3508.dji);
            return;

        case MOTOR_ID_2:

            return;
        
        case MOTOR_ID_3:

            return;
        
        case MOTOR_ID_4:

            return;

        default:
            return;
    }
}


/***
* @brief    根据can2接收到的ID进入不同的处理函数
* @param    msg can2报文 
* @return   None
***/
void can2_rec(struct rt_can_msg *msg)
{
    switch(msg->id)
    {
        default:
            return;
    }
}
