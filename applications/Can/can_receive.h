#ifndef __CAN_RECEIVE_H__
#define __CAN_RECEIVE_H__

#include <rtdevice.h>

//can1数据接收函数
void can1_rec(struct rt_can_msg *msg);
//can2数据接收函数
void can2_rec(struct rt_can_msg *msg);

#endif
