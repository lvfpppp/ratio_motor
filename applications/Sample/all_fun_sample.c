#include "ratio_motor.h"
#include "myUart.h"
#include "patrol.h"
#include "uart_agree.h"

extern void Board_Base_Init(void);


/**
 * @brief   电机pid的角度pid达到设定值附近时,反馈数据
 */
static void PID_Arrive_Callback(Motor_t const *motor,Target_e kind)
{
    RT_ASSERT(motor != RT_NULL);
    
    /* 输出到达设定值的当前角度值 */
    MyUart_Send_PrintfString("+------------------------------------+\n");
    sprintf(Get_PrintfTxt(),"|  PID steady state: %06.3f (angle)  |\n",Ratio_Motor_Read_NowPos());
    MyUart_Send_PrintfTxt();
    MyUart_Send_PrintfString("+------------------------------------+\n");
}

/**
 * @brief   反馈类型的部分初始化
 */
static void Feedback_Part_Init(void)
{
    Patrol_Init();

    Target_Set_Precision(1,MOTOR_SET);//修改辨识到达目标精度为 1度
    Register_Target_Callback(PID_Arrive_Callback,MOTOR_SET);
}

/**
 * @brief   校准完成回调
 * @param   range   电机输出轴的活动行程,单位度
 */
static void Adjust_Complete_Callback(float range)
{
    MyUart_Send_PrintfString("+-------------------------+\n");
    MyUart_Send_PrintfString("|  motor adjust result    |\n");
    MyUart_Send_PrintfString("|  min angle: 0              |\n");
    sprintf(Get_PrintfTxt(), "|  max angle: %06.3f    |\n",range);
    MyUart_Send_PrintfTxt();
    MyUart_Send_PrintfString("+-------------------------+\n");
}

/**
 * @brief   校准部分初始化,上电默认校准
 */
static void Adjust_Part_Init(void)
{
    Register_Adjust_Callback(Adjust_Complete_Callback);

    RatioM_Adjust_Init();
    RatioM_Adjust_Start();
}

/**
 * @brief   所有功能的主函数
 * @author  lfp
 */
int all_fun_main(void)
{
    Board_Base_Init();

    Adjust_Part_Init();

    Feedback_Part_Init();

    MyUart_Agree_Init();

    return 0;
}
