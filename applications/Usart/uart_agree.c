#include "uart_agree.h"
#include "myUart.h"
#include "patrol.h"


static const rt_uint16_t agree_head = 0xABAA;
static const rt_uint16_t agree_tail = 0x55CD;


static void Agree_Analysis(const Canister_Cmd_t *recv_cmd_p)
{
    if (recv_cmd_p->head == agree_head && recv_cmd_p->tail == agree_tail)
    {
        switch (recv_cmd_p->cmd)
        {
        case 1:
            //设置角度闭环到一个位置
            sprintf(Get_PrintfTxt(),"CMD 1: set pos %3.3f\n\n",recv_cmd_p->data1);
            MyUart_Send_PrintfTxt();
            
            Canister_Set_Position(recv_cmd_p->data1);
            break;

        case 2:
            //设置巡逻角度
            sprintf(Get_PrintfTxt(),"CMD 2: min %3.3f, max %3.3f\n\n",recv_cmd_p->data1,recv_cmd_p->data2);
            MyUart_Send_PrintfTxt();
            
            Patrol_Set_Pos(recv_cmd_p->data1,recv_cmd_p->data2);
            break;

        case 3:
            //开始校准,阻塞至校准完,TODO:加个校准超时停止和打印
            MyUart_Send_PrintfString("CMD 3: Now start calibrating the motor.\n\n");

            Canister_Adjust_Start();
            //等待校准完毕
            while(Canister_Get_Adjust_State() == 1){
                rt_thread_mdelay(1);
            }
            if (Canister_Get_Adjust_State() == 0)
                MyUart_Send_PrintfString("CMD 3: successfully finished calibrating the motor.\n\n");
            else
                MyUart_Send_PrintfString("CMD 3: Fail to finish calibrating the motor.\n\n");
            break;

        case 4:
            //设置最大电流和最大速度。
            sprintf(Get_PrintfTxt(),"CMD 4: I %6.1f, V %6.1f\n\n",recv_cmd_p->data1,recv_cmd_p->data2);
            MyUart_Send_PrintfTxt();

            Canister_Set_MaxCurrent(recv_cmd_p->data1);
            Canister_Set_MaxSpeed(recv_cmd_p->data2);
            break;

        case 5:
            //开始巡逻
            MyUart_Send_PrintfString("CMD 5: Start patrol.\n\n");
            Patrol_Fun_Open();
            break;
        
        case 6:
            //停止巡逻
            MyUart_Send_PrintfString("CMD 6: Finish patrol.\n\n");
            Patrol_Fun_Close();
            break;

        default:
            //无该指令
            MyUart_Send_PrintfString("Wrong command.\n\n");
            break;
        }
    }
}

void MyUart_Agree_Init(void)
{
    Register_MyUart_Recv_Callback(Agree_Analysis);
}
