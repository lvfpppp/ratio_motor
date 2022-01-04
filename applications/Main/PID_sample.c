#include "drv_canthread.h"
#include <board.h>
#include "canister.h"
#include "myUart.h"

#define CAN_RS_PIN     GET_PIN(A,15)
const rt_uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};

/* 通过VOFA+上位机显示波形。然后在debug时，
修改 en_angle_loop 值来切换速度闭环和角度闭环。
(从速度闭环切到角度闭环时,注意清空 M3508.dji.loop 变量。
否则设定角度闭环时，会转到loop为0时的角度才会停下。)
然后通过修改watch窗口的值实现调参的功能
 */
int pid_main(void)
{
	rt_err_t res = RT_EOK;

    /* can硬件电路要求,can_rs引脚拉低 */
    rt_pin_mode(CAN_RS_PIN, PIN_MODE_OUTPUT_OD);
    rt_pin_write(CAN_RS_PIN, PIN_LOW);

    /* 串口2初始化 */
    res = MyUart_Init();
    RT_ASSERT(res == RT_EOK);

    /* can初始化 */
	res = Can1_Init();
    RT_ASSERT(res == RT_EOK);

    /* 电机初始化 */
    Canister_Init();

    float ch[4];
    const Motor_t * debug_3508 = Canister_Read_MotorData();//todo:将指针写为常量的形式,测试,推git
    
    while(1)
    {
        //显示数据
        ch[0] = debug_3508->dji.extra_angle + 360.0f * debug_3508->dji.loop;//pid角度数据源
        ch[1] = debug_3508->ang.set;            //pid角度设定值
        ch[2] = debug_3508->dji.speed;          //pid速度数据源
        ch[3] = debug_3508->spe.set;            //pid速度设定值

        //调试角度环
        // ch[0] = debug_3508->dji.extra_angle + 360.0f * debug_3508->dji.loop;//pid角度数据源
        // ch[1] = debug_3508->ang.out;            //角度环输出
        // ch[2] = debug_3508->ang.set;            //pid角度设定值
        // ch[3] = debug_3508->ang.out_limit_up;   //角度环输出上限

        //调试速度环
        // ch[0] = debug_3508->dji.speed;          //pid速度数据源
        // ch[1] = debug_3508->spe.out;            //速度环输出
        // ch[2] = debug_3508->spe.set;            //pid速度设定值
        // ch[3] = debug_3508->spe.out_limit_up;   //速度环输出上限

        // 发送数据
        MyUart_Send((char *)ch, sizeof(float) * 4); //使用 VOFA+ 的 JustFloat协议 显示波形
        
        // 发送帧尾
        MyUart_Send(vofa_tail, 4);

        rt_thread_mdelay(5);
    }
}
