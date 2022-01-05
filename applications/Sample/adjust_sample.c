#include "canister.h"
#include <stdio.h>

extern void Board_Base_Init(void);

static void Adjust_Complete_Callback(float range)
{
    char txt[10];
    sprintf(txt,"%3.3f",range);
    rt_kprintf("motor adjust result:\n");
    rt_kprintf("min angle: 0\n");
    rt_kprintf("max angle: %s\n",txt);
}

static float test_angle = 0;
static void Test_Adjust(void)
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
    则可以测试角度闭环（基于校准好后的角度）
    (rt_kprintf函数需要设置串口2为控制台串口)
*/
int adjust_main(void)
{
    Board_Base_Init();

    Test_Adjust();

    return 0;
}
