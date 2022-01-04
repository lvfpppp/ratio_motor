#include "drv_canthread.h"
#include <board.h>
#include "canister.h"
#include <stdio.h>

#define CAN_RS_PIN     GET_PIN(A,15)

void Adjust_Complete_Callback(float range)
{
    char txt[10];
    sprintf(txt,"%3.3f",range);
    rt_kprintf("motor adjust result:\n");
    rt_kprintf("min angle: 0\n");
    rt_kprintf("max angle: %s\n",txt);
}

float test_angle = 0;
void Test_Adjust(void)
{
    Register_Adjust_Callback(Adjust_Complete_Callback);

    Canister_Adjust_Init();
    Canister_Adjust_Start();

    while(1)
    {
        if (Canister_Get_Adjust_State() == 0)
            Canister_Set_Position(test_angle);
        
        rt_thread_mdelay(10);
    }
}

/* 上电后，电机会先顺时针旋转，寻找角度的最小值。
    然后在堵转后（依据掉速来判断），切换旋转方向。 
    然后遇到最大位置边界，便会停止到边界的最小值上。
    在校准完后，通过在debug窗口修改test_angle变量值，
    则可以测试角度闭环（基于校准好后的角度）*/
int adjust_main(void)
{
	rt_err_t res = RT_EOK;

    /* can硬件电路要求,can_rs引脚拉低 */
    rt_pin_mode(CAN_RS_PIN, PIN_MODE_OUTPUT_OD);
    rt_pin_write(CAN_RS_PIN, PIN_LOW);

	res = Can1_Init();
    RT_ASSERT(res == RT_EOK);

    Canister_Init();

    Test_Adjust();

    return 0;
}
