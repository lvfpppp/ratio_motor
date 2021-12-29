#ifndef __PID_H__
#define __PID_H__

#include <rtthread.h>


typedef struct
{
	float 	kp;					//pid三个参数
	float 	ki;
	float 	kd;

	float 	set;				//pid的设定值
	float 	err;				//偏差值
	float 	err_old;			//上次偏差值
	float 	err_LPF;			// 对ERR进行滞后滤波，微分项用

	float 	i_value;			//现在的积分值（该积分已经乘了ki参数了
	float  	i_limit;			//积分限幅
	rt_int8_t I_Dis;			// 为1时会停止修改I_Value

	float	out;				//pid的输出
	float	out_limit_up;		//输出上限幅
	float	out_limit_down;		//输出下限幅
}pid_t;
//电机PID闭环结构体

/*pid创建时参数初始化*/
#define PID_INIT(val_kp, val_ki, val_kd,i_lim,lim_up,lim_down)	\
    {                        									\
        .kp = val_kp,											\
		.ki = val_ki,											\
		.kd = val_kd,											\
		.set = 0,												\
		.err = 0,												\
		.err_old = 0,											\
		.err_LPF = 0,											\
		.i_value = 0,											\
		.i_limit = i_lim,										\
		.I_Dis = 0,												\
		.out = 0,												\
		.out_limit_up = lim_up,									\
		.out_limit_down	= lim_down								\
    }


extern void pid_init(pid_t *pid,
			float kp, float ki, float kd,
			float i_limit,
			float 	out_limit_up,
			float 	out_limit_down);
		
extern void PID_Calculate(pid_t* target,float Error);
extern void pid_clear(pid_t* target);
	
#endif
