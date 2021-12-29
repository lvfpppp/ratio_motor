#include "drv_canthread.h"
#include "HCanID_data.h"
#include "drv_motor.h"
#include <board.h>

#define IF_INIT_OK	RT_ASSERT(res == RT_EOK); //是否初始化成功

Motor_t M3508;

#define CAN_RS_PIN    GET_PIN(A,15)
#define MAX_3508_RPM    (469.0f)


void Motor_Init(void)
{
    motor_init(&M3508,MOTOR_ID_1,(3591.0f/187.0f),ANGLE_CTRL_EXTRA,8192,180,-180);
    pid_init(&M3508.ang,1,0,0,1000,MAX_3508_RPM,-MAX_3508_RPM);
    pid_init(&M3508.spe,1,0,0,1000,10000,-10000);

    /*等待电机第一次通信完毕*/
	while(M3508.dji.Data_Valid == 0) 
    {rt_thread_mdelay(50);}
}

void Motor_Cal(void)
{
    while(1)
    {
        Motor_AnglePIDCalculate(&M3508,Motor_Read_NowAngle(&M3508));

        Motor_SpeedPIDCalculate(&M3508,M3508.ang.out);

        motor_current_send(can1_dev,STDID_launch,Motor_Read_OutSpeed(&M3508),0,0,0);

        rt_thread_mdelay(5);
    }
}

int main(void)
{
	rt_err_t res = RT_EOK;

    rt_pin_mode(CAN_RS_PIN, PIN_MODE_OUTPUT_OD);
    rt_pin_write(CAN_RS_PIN, PIN_LOW);

	res = Can1_Init();				IF_INIT_OK

    Motor_Init();
    Motor_Cal();
}
