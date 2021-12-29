#include "drv_motor.h"


/**
* @brief：电机转动量计算（跨圈处理）
* @param float Angle1: 要转到的角度对应的数值
* @param float Angle2: 转动起点数值
* @param float Round_Len: 电机整圈对应的数值
* @return float: 输出从Angle2到Angle1需要的最短路径的距离长度, 范围 负半圈~正半圈，正好半圈时取正半圈
* @author：ych
*/
float Motor_Get_DeltaAngle(float Angle1, float Angle2, float Round_Len)
{
	float DAngle,HalfRound;
	DAngle = Angle1 - Angle2;
	HalfRound = Round_Len / 2;
	if (DAngle >= 0)
	{
		if (DAngle > HalfRound)
		{
			DAngle -= Round_Len;
		}
	}
	else
	{
		if (DAngle <= -HalfRound)
		{
			DAngle += Round_Len;
		}
	}
	return DAngle;
}


/**
 * @brief  对反馈角度进行换算为输出轴的0-360°
 * @param  motor：电机数据结构体
 * @retval None
 */
static void motor_angle_adjust(DjiMotor_t* motor)
{
	float angletemp = 0;
	float LEN;

	if(motor->oldangle_state == RT_ERROR)
	{//第一次
		motor->oldangle_state = RT_EOK;
		motor->old_angle = motor->angle;//第一次记录上次值为当前值
		motor->extra_angle = 0;
		motor->loop = 0;// motor初始化前CAN就会有通信，所以初始化后的第一次CAN通信时程序才运行到这里，此时需要清空loop的数值
		motor->Data_Valid = 1;
	}
	else
	{
		LEN = (float)motor->Encoder_LEN;
		angletemp = Motor_Get_DeltaAngle(motor->angle, motor->old_angle, LEN) / motor->ratio /LEN*360.0f + motor->extra_angle; //角度积分
		if (angletemp >= 360)
		{//积分大于一圈
			motor->extra_angle = angletemp - 360;
			motor->loop += 1;
		}
		else if(angletemp < 0)
		{//积分小于0
			motor->extra_angle = angletemp + 360;
			motor->loop -= 1;
		}
		else
		{//正常
			motor->extra_angle = angletemp;//更新积分值
		}
		motor->old_angle = motor->angle;
	}
}

/**
 * @brief  读取can中的电机数据
 * @param  rxmsg：rtcan结构体
 * @param  motor：电机数据结构
 * @retval None
 */
void motor_readmsg(struct rt_can_msg* rxmsg,DjiMotor_t* motor)
{
	motor->speed = (rt_int16_t)(rxmsg->data[2]<<8 | rxmsg->data[3]);
	motor->current = (rt_int16_t)(rxmsg->data[4]<<8 | rxmsg->data[5]);
	motor->temperature = rxmsg->data[6];
	motor->angle = (rxmsg->data[0]<<8) + rxmsg->data[1];	//转子角度
	
	motor_angle_adjust(motor);
}

/**
 * @brief  电机电流发送
 * @param  dev：can设备
 * @param  setcurn:设定电流 
 * @retval RT_EOK or RT_ERROR(成功，失败或其他报错)
 */
rt_size_t motor_current_send(rt_device_t dev, 
														SendID_e   	ID,
														rt_int16_t 	setcur1, 
														rt_int16_t 	setcur2, 
														rt_int16_t 	setcur3, 
														rt_int16_t 	setcur4)
{
	struct rt_can_msg txmsg;
	
	txmsg.id=ID;
	txmsg.ide = RT_CAN_STDID;txmsg.rtr=RT_CAN_DTR;
	txmsg.len=8;
	txmsg.data[0]=(rt_uint8_t)(setcur1>>8);txmsg.data[1]=(rt_uint8_t)(setcur1);
	txmsg.data[2]=(rt_uint8_t)(setcur2>>8);txmsg.data[3]=(rt_uint8_t)(setcur2);
	txmsg.data[4]=(rt_uint8_t)(setcur3>>8);txmsg.data[5]=(rt_uint8_t)(setcur3);
	txmsg.data[6]=(rt_uint8_t)(setcur4>>8);txmsg.data[7]=(rt_uint8_t)(setcur4);
	
	return rt_device_write(dev, 0, &txmsg, sizeof(txmsg));
}

/**
* @brief：电机角度环pid输出的计算，不会修改速度环设定值
* @brief：调用过程中需要保证同一个电机的设定值和传入的实际值的单位相同
* @param [Motor_t*]	Motor:需要角度环计算的电机的结构体
* @param [float]	AngleNow:角度实际值
* @return: [float] PID计算结果
* @author：ych
*/
float Motor_AnglePIDCalculate(Motor_t *Motor, float AngleNow)
{
	float Error;
	
	if(Motor->dji.Round_Len!=0)
	{// 电机结构体初始化正常，可以进行跨圈处理计算，获得PID需要的Error
		if (Motor->dji.Angle_CtrlMode != ANGLE_CTRL_FULL) //计算偏差量Error
		{
			Error = Motor_Get_DeltaAngle(Motor->ang.set, AngleNow, Motor->dji.Round_Len); 
		}
		else
		{// 如果电机使用的是可跨圈角度环模式，则不进行跨圈计算
			Error = Motor->ang.set - AngleNow;
		}
	}
	else
	{
		while(1);//电机结构体没有设定 闭环整圈长度 参数，可能没有调用电机结构体初始化函数。
	}
	PID_Calculate(&Motor->ang, Error); //完成PID计算
	return Motor->ang.out;
}

/**
* @brief：电机转速环pid输出的计算
* @param [Motor_t*]	Motor:需要速度环计算的电机的结构体
* @param [float]	SpeedNow:转速实际值
* @return: [float] PID计算结果
* @author：ych
*/
float Motor_SpeedPIDCalculate(Motor_t *Motor, float SpeedNow)
{
	float Error;
	Error = Motor->spe.set - SpeedNow;
	PID_Calculate(&Motor->spe, Error);	//完成PID计算
	return Motor->spe.out;
}

/**
* @brief：电机角度设定值读取
* @param [Motor_t*]	Motor:需要读取的电机的结构体
* @return [float] 读取角度设定值
* @author：ych
*/
float Motor_Read_SetAngle(Motor_t *Motor)
{
	return Motor->ang.set;
}

/**
* @brief：电机速度设定值读取
* @param [Motor_t*]	Motor:需要读取的电机的结构体
* @return [float] 读取速度设定值
* @author：ych
*/
float Motor_Read_SetSpeed(Motor_t *Motor)
{
	return Motor->spe.set;
}

/**
* @brief 电机角度读取
* @param ABS控制模式下，返回编码器的原始数据
* @param EXTRA控制模式下，返回电机减速箱输出轴的角度，以上电位置为零点，0~360°
* @param FULL控制模式下，返回电机减速箱输出轴上电后转动过的总角度，以上电位置为零点，单位°
* @param [Motor_t*]	Motor:需要读取的电机的结构体
* @return [float] 返回当前电机控制模式下闭环所需的角度数值
* @author：ych
*/
float Motor_Read_NowAngle(Motor_t *Motor)
{
	switch (Motor->dji.Angle_CtrlMode)
	{
	case ANGLE_CTRL_ABS:
		return Motor->dji.angle;
	case ANGLE_CTRL_EXTRA:
		return Motor->dji.extra_angle;
	case ANGLE_CTRL_FULL:
		return Motor->dji.extra_angle+360.0f*Motor->dji.loop;
	default:
		return 0;
	}
}

/**
* @brief：返回编码器的原始数据
* @param [Motor_t*]	Motor:需要读取的电机的结构体
* @return [float] 返回编码器的原始数据
* @author：ych
*/
float Motor_Read_NowEncoder(Motor_t *Motor)
{
	return Motor->dji.angle;
}

/**
* @brief：返回编码器转速数据
* @param [Motor_t*]	Motor:需要读取的电机的结构体
* @return [float] 返回编码器的转速数据
* @author：ych
*/
float Motor_Read_NowSpeed(Motor_t *Motor)
{
	return Motor->dji.speed;
}

/**
* @brief：返回速度pid计算结果
* @param [Motor_t*]	Motor:需要读取的电机的结构体
* @return [float] 返回pid计算结果
* @author：ych
*/
float Motor_Read_OutSpeed(Motor_t *Motor)
{
	return Motor->spe.out;
}

/**
* @brief：电机角度设定值绝对式修改
* @param [Motor_t*]	Motor:需要修改的电机的结构体
* @param [float]	Set:新的角度设定值
* @author：ych
*/
void Motor_Write_SetAngle_ABS(Motor_t *Motor, float Set)
{
	float SetCal = Set;
	if (Motor->dji.Angle_CtrlMode != ANGLE_CTRL_FULL)
	{ // 如果是不跨圈的闭环模式
		if (SetCal > Motor->dji.Set_MAX)
		{ // 设定值不可跨圈
			do
			{
				SetCal -= Motor->dji.Round_Len;
			} while (SetCal > Motor->dji.Set_MAX);
		}
		else if (SetCal <= Motor->dji.Set_MIN)
		{ // 设定值不可跨圈
			do
			{
				SetCal += Motor->dji.Round_Len;
			} while (SetCal <= Motor->dji.Set_MIN);
		}
	}
	Motor->ang.set = SetCal;
}

/**
* @brief：电机角度设定值增量式修改
* @param [Motor_t*]	Motor:需要修改的电机的结构体
* @param [float]	Set:新的角度设定值
* @author：ych
*/
void Motor_Write_SetAngle_ADD(Motor_t *Motor, float SetAdd)
{
	float SetCal;
	SetCal = Motor->ang.set + SetAdd;
	if (Motor->dji.Angle_CtrlMode != ANGLE_CTRL_FULL)
	{// 如果是不跨圈的闭环模式
		if (SetCal >= Motor->dji.Set_MAX)
		{ // 设定值不可跨圈
			do
			{
				SetCal -= Motor->dji.Round_Len;
			} while (SetCal >= Motor->dji.Set_MAX);
		}
		else if (SetCal < Motor->dji.Set_MIN)
		{ // 设定值不可跨圈
			do
			{
				SetCal += Motor->dji.Round_Len;
			} while (SetCal < Motor->dji.Set_MIN);
		}
	}
	Motor->ang.set = SetCal;
}

/**
* @brief：电机转速设定值绝对式修改
* @param [Motor_t*]	Motor:需要修改的电机的结构体
* @param [float]	Set:新的转速设定值,Set>0,从输出轴看电机，电机逆时针旋转
* @author：ych
*/
void Motor_Write_SetSpeed_ABS(Motor_t *Motor, float Set)
{
	Motor->spe.set = Set;
}

/**
* @brief：电机转速设定值增量式修改
* @param [Motor_t*]	Motor:需要修改的电机的结构体
* @param [float]	Set:新的转速设定值
* @author：ych
*/
void Motor_Write_SetSpeed_ADD(Motor_t *Motor, float SetADD)
{
	Motor->spe.set += SetADD;
}

/**
* @brief：使用角度环输出作为电机转速设定值
* @param [Motor_t*]	Motor:需要修改的电机的结构体
* @author：ych
*/
void Motor_Write_SetSpeed_FromAnglePID(Motor_t *Motor)
{
	Motor_Write_SetSpeed_ABS(Motor, Motor->ang.out);
}

/**
* @brief：屏蔽转速PID中的I
* @param [Motor_t*]	Motor:需要修改的电机的结构体
* @author：ych
*/
void Motor_Write_SpeedPID_Idat_Fix(Motor_t *Motor)
{
	Motor->spe.I_Dis = 1;
}

/**
* @brief：恢复转速PID中的I
* @param [Motor_t*]	Motor:需要修改的电机的结构体
* @author：ych
*/
void Motor_Write_SpeedPID_Idat_Recover(Motor_t *Motor)
{
	Motor->spe.I_Dis = 0;
}
/**
* @brief：电机结构体初始化
* @param [Motor_t*]	Motor:需要修改的电机的结构体
* @param [rt_uint32_t]	ID:电机的CANID
* @param [float]	radio:	电机的减速比，用于计算电机转动圈数
							填写电机输出轴转动一圈时，编码器数据转过的圈数，以完整的M3508为例，应填写19.0f
* @param [float]	Encoder_Len:电机转动一圈对应多少编码器角度单位，例如GM6020采用8192线编码器，则应填写8192
* @param [rt_int32_t]	Set_Max:设定值的最大值
* @param [rt_int32_t]	Set_Min:设定值的最小值
								如果使用编码器，则填写编码器能达到的数据范围 如0-8192（不是8191）
								如果闭环时将使用外部角度数据，则填写外部角度数据能达到的数据范围 如0~360 / -180~180
								闭环时PID输入的角度的范围必须与设定值范围和方向一致
* @author：ych
*/
void motor_init(Motor_t *motor, rt_uint32_t ID, float radio,Angle_CtrlMode_E ModeSet,rt_int32_t Encoder_Len, rt_int32_t Set_Max,rt_int32_t Set_Min)
{
	if (Set_Max < Set_Min)
	{
		while(1)
		{
			;//最大值不应小于最小值
		}
	}
	motor->dji.motorID = ID;
	motor->dji.ratio = radio;
	motor->dji.oldangle_state = RT_ERROR;
	motor->dji.Round_Len = Set_Max - Set_Min; // 设定编码器一圈对应的变化量
	motor->dji.Angle_CtrlMode = ModeSet; //设定电机闭环控制模式
	motor->dji.Encoder_LEN = Encoder_Len;
	motor->dji.Data_Valid = 0;// 刚初始化完成，数据还不可用，需要等待下一次CAN通信接收完成
	if(ModeSet!=ANGLE_CTRL_FULL)
	{ // 如果选用是不能跨圈的角度闭环模式
		// 记录设定值的允许范围
		motor->dji.Set_MIN = Set_Min;
		motor->dji.Set_MAX = Set_Max;
	}
}
