#include "uart_agree.h"
#include "myUart.h"
#include "patrol.h"

extern void rt_hw_cpu_reset(void);

const rt_uint16_t agree_head = 0xABAA;
const rt_uint16_t agree_tail = 0x55CD;

/**
 * @brief   列出所有命令信息
 */
static void List_All_Command(void)
{
    MyUart_Send_PrintfString("\n+-----------------------------------------");   MyUart_Send_PrintfString("------------------------+\n");
    MyUart_Send_PrintfString("|           head        |   cmd  | data1 |");   MyUart_Send_PrintfString(" data2 |            tail          |\n");
    MyUart_Send_PrintfString("|------------------- +-------+-------+");   MyUart_Send_PrintfString("-------+------------------- |\n");
    sprintf(Get_PrintfTxt(),"| 0x%02X ",agree_head); 
    MyUart_Send_PrintfTxt();
    sprintf(Get_PrintfTxt(),"(uint16) | (uint8) | (float) | (float) "); 
    MyUart_Send_PrintfTxt();
    sprintf(Get_PrintfTxt(),"| 0x%02X (uint16) |\n",agree_tail); 
    MyUart_Send_PrintfTxt();
    MyUart_Send_PrintfString("|-----------------------------------------");   MyUart_Send_PrintfString("--------------------------|\n");
    MyUart_Send_PrintfString("| $ cmd1: Set the position of the Angle loop.");MyUart_Send_PrintfString("                       |\n");
    MyUart_Send_PrintfString("| $ cmd2: Set up two positions for patrol.");   MyUart_Send_PrintfString("                            |\n");
    MyUart_Send_PrintfString("| $ cmd3: Start the calibration function.");    MyUart_Send_PrintfString("                               |\n");
    MyUart_Send_PrintfString("| $ cmd4: Set the max current and max speed."); MyUart_Send_PrintfString("                    |\n");
    MyUart_Send_PrintfString("| $ cmd5: Start patrol function.");             MyUart_Send_PrintfString("                                            |\n");
    MyUart_Send_PrintfString("| $ cmd6: End patrol function.");               MyUart_Send_PrintfString("                                             |\n");
    MyUart_Send_PrintfString("| $ cmd7: Printing Help Information.");         MyUart_Send_PrintfString("                                   |\n");
    MyUart_Send_PrintfString("| $ cmd8: Reset microcontroller.");             MyUart_Send_PrintfString("                                         |\n");
    MyUart_Send_PrintfString("+-----------------------------------------");   MyUart_Send_PrintfString("------------------------+\n\n");
}

/**
 * @brief   协议解析
 * @param   recv_cmd_p 已拆分的数据包
 */
static void Agree_Analysis(const Ratio_motor_Cmd_t *recv_cmd_p)
{
    RT_ASSERT(recv_cmd_p != RT_NULL);

    if (recv_cmd_p->head == agree_head && recv_cmd_p->tail == agree_tail)
    {
        switch (recv_cmd_p->cmd)
        {
        case 1: //设置角度闭环到一个位置
            if (Patrol_If_Finsh() == RT_TRUE) //确保巡逻功能可用
            {
                sprintf(Get_PrintfTxt(),"[CMD 1]: set pos %06.3f\n",recv_cmd_p->data1);
                MyUart_Send_PrintfTxt();

                Ratio_Motor_Set_Position(recv_cmd_p->data1);
            }
            else
                MyUart_Send_PrintfString("[CMD 1]: Motor is occupied by Patrol function.\n");
            break;

        case 2: //设置巡逻角度
			{
                float p_start = recv_cmd_p->data1;
                float p_end = recv_cmd_p->data2;

                // 输入限幅
                float ratio_motor_max = Ratio_Motor_Read_Pos_Range();
                VALUE_CLAMP(p_start,0,ratio_motor_max);
                VALUE_CLAMP(p_end,0,ratio_motor_max);

                sprintf(Get_PrintfTxt(),"[CMD 2]: start %06.3f, end %06.3f\n",p_start,p_end);
                MyUart_Send_PrintfTxt();

                Patrol_Set_Pos(p_start,p_end);
            }
            break;

        case 3: //开始校准电机
            RatioM_Adjust_Start();
            break;

        case 4: //设置最大电流和最大速度
            sprintf(Get_PrintfTxt(),"[CMD 4]: I %6.1f, V %6.1f\n",recv_cmd_p->data1,recv_cmd_p->data2);
            MyUart_Send_PrintfTxt();

            Ratio_Motor_Set_MaxCurrent(recv_cmd_p->data1);
            Ratio_Motor_Set_MaxSpeed(recv_cmd_p->data2);
            break;

        case 5: //开始巡逻
            MyUart_Send_PrintfString("[CMD 5]: Start patrol.\n");
            Patrol_Fun_Open();
            break;
        
        case 6: //停止巡逻
            MyUart_Send_PrintfString("[CMD 6]: Finish patrol.\n");
            Patrol_Fun_Close();
            break;

        case 7: //打印所有指令信息
            MyUart_Send_PrintfString("[CMD 7]: Prints all command information.\n");
            List_All_Command();
            break;
        
        case 8: //单片机复位
            MyUart_Send_PrintfString("[CMD 8]: Reset microcontroller.\n");
            rt_hw_cpu_reset();

        default: //无该指令
            MyUart_Send_PrintfString("[CMD]: Wrong command.\n");
            break;
        }
    }
}

/**
 * @brief   协议解析初始化
 */
void MyUart_Agree_Init(void)
{
    Register_MyUart_Recv_Callback(Agree_Analysis);
}
