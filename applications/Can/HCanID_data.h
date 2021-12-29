#ifndef __HCANID_DATA_H__
#define __HCANID_DATA_H__
#include <rtthread.h>
#include <rtdevice.h>
///////////////////////////////////////CAN ID///////////////////////////////////////

typedef enum
{
	//底盘四轮电机反馈 ID
	MOTOR_ID_1 = 0x201, 
	MOTOR_ID_2 = 0x202,
	MOTOR_ID_3 = 0x203,
	MOTOR_ID_4 = 0x204,

} drv_can1ID_e;

///////////////////////////////////////CAN 过滤表设置///////////////////////////////////////
/*当增加原过滤表没有的can接收ID时，需在下面宏内加上相对应的过滤表项和数量*/
/*从原drv_canthread.c中单独拉出来，便于修改以及尽量不对底层驱动做修改*/
#define MYCAN1_FILTER_ITEM	struct rt_can_filter_item items[MYCAN1_FILTER_ITEM_NUM] = \
							{ \
								RT_CAN_FILTER_ITEM_INIT(RIGHT_FRONT   		, 0, 0, 1, 0x7F8, RT_NULL, RT_NULL), /* std,match ID:0x200~0x207，hdr 为 - 1，设置默认过滤表 */\
								RT_CAN_FILTER_ITEM_INIT(CHASSIS_IMU_DATA1_RX, 0, 0, 1, 0x7F8, RT_NULL, RT_NULL), /* std,match ID:0x400~0x407，hdr 为 - 1，设置默认过滤表 */\
							}; \

#define MYCAN1_FILTER_ITEM_NUM 2	//can1硬件过滤表的数量								
	
#define MYCAN2_FILTER_ITEM  struct rt_can_filter_item items[MYCAN2_FILTER_ITEM_NUM] = \
							{ \
								RT_CAN_FILTER_ITEM_INIT(YAW_ID			, 0, 0, 1, 0x7F8, RT_NULL, RT_NULL), /* std,match ID:0x200~0x207，hdr 为 - 1，设置默认过滤表 */\
								RT_CAN_FILTER_ITEM_INIT(SCPR_RX			, 0, 0, 1, 0x7F8, RT_NULL, RT_NULL), /* std,match ID:0x090~0x097，hdr 为 - 1，设置默认过滤表 */\
								RT_CAN_FILTER_ITEM_INIT(GIMBAL_CCTRL_RX , 0, 0, 1, 0x7F8, RT_NULL, RT_NULL), /* std,match ID:0x100~0x107，hdr 为 - 1，设置默认过滤表 */\
								RT_CAN_FILTER_ITEM_INIT(GIMBAL_DATA_RX  , 0, 0, 1, 0x7F8, RT_NULL, RT_NULL), /* std,match ID:0x400~0x407，hdr 为 - 1，设置默认过滤表 */\
							}; \

#define MYCAN2_FILTER_ITEM_NUM 4	//can2硬件过滤表的数量
//////////////////////////////////////////////////////////////////////////////////////

							
#endif
