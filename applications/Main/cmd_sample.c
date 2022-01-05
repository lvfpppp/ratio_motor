#include "myUart.h"
#include <stdio.h>

extern void Board_Base_Init(void);

const rt_uint16_t agree_head = 0xABAA;
const rt_uint16_t agree_tail = 0x55CD;

char feedback_txt[50];

void Agree_Analysis(const Canister_Cmd_t *recv_cmd_p)
{
    if (recv_cmd_p->head == agree_head && recv_cmd_p->tail == agree_tail)
    {
        switch (recv_cmd_p->cmd)
        {
        case 1:
            //设置角度闭环到一个位置
            sprintf(feedback_txt,"CMD 1: set pos %3.3f\n",recv_cmd_p->data1);
            MyUart_Send(feedback_txt,rt_strlen(feedback_txt));
            break;

        case 2:
            //设置巡逻，两个角度之间来回跳转。
            sprintf(feedback_txt,"CMD 2: min %3.3f, max %3.3f\n",recv_cmd_p->data1,recv_cmd_p->data2);
            MyUart_Send(feedback_txt,rt_strlen(feedback_txt));
            break;

        case 3:
            //开始校准
            sprintf(feedback_txt,"CMD 3: Now start calibrating the motor.\n");
            MyUart_Send(feedback_txt,rt_strlen(feedback_txt));
            break;

        case 4:
            //设置最大电流和最大速度。
            sprintf(feedback_txt,"CMD 4: I %6.1f, V %6.1f\n",recv_cmd_p->data1,recv_cmd_p->data2);
            MyUart_Send(feedback_txt,rt_strlen(feedback_txt));
            break;

        default:
            //无该指令
            sprintf(feedback_txt,"Wrong command.\n");
            MyUart_Send(feedback_txt,rt_strlen(feedback_txt));
            break;
        }
    }
}


/*
(需要取消设置串口2为控制台串口)
*/
int cmd_main(void)
{
    /* 串口2初始化 */
    rt_err_t res = MyUart_Init();
    RT_ASSERT(res == RT_EOK);

    Board_Base_Init();

    Register_MyUart_Recv_Callback(Agree_Analysis);

    return 0;
}
