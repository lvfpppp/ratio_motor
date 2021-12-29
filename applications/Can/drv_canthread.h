#ifndef __DRV_CANTHREAD_H__
#define __DRV_CANTHREAD_H__
#include <rtdevice.h>
#include <rtthread.h>

//can设备名称
#define CAN1_DEV_NAME		"can1"				
#define CAN2_DEV_NAME		"can2"

//can1线程
#define THREAD_STACK_CAN1RX 		(768)
#define THREAD_PRIO_CAN1RX 			(2)
#define THREAD_TICK_CAN1RX 			(5)
//can2线程	
#define THREAD_STACK_CAN2RX 		(768)
#define THREAD_PRIO_CAN2RX 			(3)
#define THREAD_TICK_CAN2RX 			(5)

//CAN 设备句柄
extern rt_device_t can1_dev;     
extern rt_device_t can2_dev;


/**
 * @brief  can1初始化，can1数据处理线程和中断设定
 * @param  None
 * @retval rt_err_t
 */
rt_err_t Can1_Init(void);

/**
 * @brief  can2初始化，can2数据处理线程和中断设定
 * @param  None
 * @retval rt_err_t
 */
rt_err_t Can2_Init(void);


#endif
