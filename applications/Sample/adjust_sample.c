#include "ratio_motor.h"
#include "myUart.h"

extern void Board_Base_Init(void);

static void Adjust_Complete_Callback(float range)
{
    MyUart_Send_PrintfString("+-------------------------+\n");
    MyUart_Send_PrintfString("|  motor adjust result    |\n");
    MyUart_Send_PrintfString("|  min angle: 0              |\n");
    sprintf(Get_PrintfTxt(), "|  max angle: %06.3f    |\n",range);
    MyUart_Send_PrintfTxt();
    MyUart_Send_PrintfString("+-------------------------+\n");
}

static float test_angle = 0;
static void Test_Adjust(void)
{
    Register_Adjust_Callback(Adjust_Complete_Callback);

    RatioM_Adjust_Init();
    RatioM_Adjust_Start();

    while(1)
    {
        if (RatioM_Adjust_If_Finsh() == RT_TRUE)
            Ratio_Motor_Set_Position(test_angle);
        
        rt_thread_mdelay(10);
    }
}

/* 上电后，电机会先顺时针旋转，寻找角度的最小值。
    然后在堵转后（依据掉速来判断），切换旋转方向。 
    然后遇到最大位置边界，便会停止到边界的最小值上。
    在校准完后，通过在debug窗口修改test_angle变量值，
    则可以测试角度闭环（基于校准好后的角度）
*/
int adjust_main(void)
{
    Board_Base_Init();

    Test_Adjust();

    return 0;
}
