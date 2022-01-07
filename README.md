<a name="IlKrN"></a>
# 前言	
&emsp;&emsp;本工程是基于RT-Thread v4.0.2版本开发的。主要编写代码全部置于application文件夹下。经过`scons --dist`后独立出来的代码，可以放在任意英文路径下使用`scons --target=mdk5`构建和keil编译。开箱即用。
<br />&emsp;&emsp;使用的硬件资源有STM32F103CBT6、can1、uart2。使用ST-Link下载和调试。使用[VOFA+上位机](https://www.vofa.plus/)显示程序输出和指令控制。
<a name="c7Pqm"></a>
# 指令
<a name="Mz4iK"></a>
## 指令表
| 数据头 | 指令 | 数据位1 | 数据位2 | 数据尾 |
| --- | --- | --- | --- | --- |
| 0xAB AA（uint16） | （uint8） | （float） | （float） | 0x55 CD（uint16） |

| **指令** | **数据位1** | **数据位2** |
| --- | --- | --- |
| 指令 1：设置角度闭环到一个位置 | 数据位 1：位置（角度），单位度 | 空（任意） |
| 指令 2：设置巡逻的两个角度 | 数据位 1：最小位置（起始位置），单位度 | 数据位 2：最大位置（终点位置），单位度 |
| 指令 3：开始校准（默认上电先校准） | 空（任意） | 空（任意） |
| 指令 4：设置最大电流和最大速度。（默认带有限幅） | 数据位 1：最大电流，单位：控制电流值范围0~16384，对应电调输出的转矩电流范围0~20A。 | 数据位 2：最大速度，单位：电机快转子的rpm，C620电调接受的最大值为9000左右。 |
| 指令 5：开始巡逻 | 空（任意） | 空（任意） |
| 指令 6：停止巡逻 | 空（任意） | 空（任意） |
| 指令 7：打印所有指令 | 空（任意） | 空（任意） |
| 指令 8：复位单片机 | 空（任意） | 空（任意） |

<a name="jq8jX"></a>
## 在线生成指令

- 将以下代码复制粘贴到 C语言在线网站 [http://www.dooccn.com/c/](http://www.dooccn.com/c/)
- 然后运行就会生成 `AA AB 04 00 00 40 1C 46 00 80 BB 45 CD 55 00 00 `。即为指令4，设置最大电流为10000，设置最大速度为6000。
- 可以通过修改结构体成员`cmd,data1,data2`的值，来生成其他指令。
```c
#include <stdio.h>
#include <cstdlib>
typedef unsigned short                  rt_uint16_t;    /**< 16bit unsigned integer type */
typedef unsigned char                   rt_uint8_t;     /**<  8bit unsigned integer type */
typedef unsigned int                    rt_uint32_t;    /**< 32bit unsigned integer type */

typedef struct
{
    rt_uint16_t head;
    rt_uint8_t cmd;
    float  data1;
    float  data2;
    rt_uint16_t tail;

} Canister_Cmd_t;

union uart_cmd
{
	Canister_Cmd_t cmd;
	rt_uint8_t buff[sizeof(Canister_Cmd_t)];
} canister_u;

int main(void) {
    
    canister_u.cmd.head  = 0xABAA;
    canister_u.cmd.cmd   = 4;
    canister_u.cmd.data1 = 10000;
    canister_u.cmd.data2 = 6000;
    canister_u.cmd.tail  = 0x55CD;

    for (int i = 0; i < sizeof(Canister_Cmd_t); i++)
	    printf("%02X ",canister_u.buff[i]);

	return 0;
}
```
<a name="i437C"></a>
## VOFA+ 命令设置

- 列举一些测试时，经常使用的指令，如下，

![image.png](/doc/image1.png )

| 设置闭环角度0 | set angle 0 | AA AB 01 00 00 00 00 00 00 00 00 00 CD 55 00 00 |
| --- | --- | --- |
| 设置巡逻角度为10和55 | set patrol angle 10<-->55 | AA AB 02 00 00 00 20 41 00 00 5C 42 CD 55 00 00 |
| 开始巡逻 | start patrol | AA AB 05 00 00 00 00 00 00 00 00 00 CD 55 00 00 |
| 结束巡逻 | stop patrol | AA AB 06 00 00 00 00 00 00 00 00 00 CD 55 00 00 |
| 开始校准 | start adjust | AA AB 03 00 00 00 00 00 00 00 00 00 CD 55 00 00 |
| 帮助（不够完善，没有特别多的作用） | help | AA AB 07 00 00 00 00 00 00 00 00 00 CD 55 00 00 |
| 复位单片机 | reset | AA AB 08 00 00 00 00 00 00 00 00 00 CD 55 00 00 |

- 也可以直接导入，[vofa+.cmds.json](/doc/vofa+.cmds.json)
- 注意选择`十六进制(Hex)`发送。
<a name="VbVBq"></a>
# 代码
<a name="VlQL9"></a>
## Sample宏
&emsp;&emsp;application/sample文件夹下有许多xxx_sample.c文件。有各个功能单独的sample.c，也有最后所有功能合起来的sample.c。功能单独的sample.c是项目前期编写后期整理的，大致和全功能差不多。因为可以方便单独测试功能，所以便选择保留下来。<br />&emsp;&emsp;选择不同的sample.c文件作为主体不需要重新scons。因为默认是在构建时把所有sample.c都放进工程里，所以只需要注释不同的宏，便能选择不同的sample.c作为主体，如下，
```c
// #define EN_ADJUST_TEST      //adjust_sample.c
// #define EN_PID_DEBUG        //PID_sample.c
// #define EN_UART_CMD         //cmd_sample.c
#define EN_ALL_FUN          //all_fun_sample.c
```
&emsp;&emsp;不同的sample是互不干扰的。所有sample.c都带入工程里进行编译，代价是一些ROM和的RAM开销。如果觉得画蛇添足，可以自行从工程中remove或者直接删除文件，仅保留all_fun_sample.c即可。
<a name="t5Qdj"></a>
## 实验现象
`使用 all_fun_sample.c 文件`
<a name="KomAy"></a>
### 1.上电校准
&emsp;&emsp;上电时默认会校准。电机会先顺时针（从3508输出轴方向看）以恒定低速旋转，堵转后再逆时针以恒定低速旋转。再堵转后变会停在原地，并输出数据。堵转校准的时候会预留2度的位置（**ADJUST_POS_MARGIN** 宏可调预留大小）。就是本来堵转校准时是卡着3.0°的最小角度，但程序中会保持5.0°的最小角度边界。同理在校准最大角度时，校准的是90.0°，实际写入的是88.0°的最大角度边界。
<br />&emsp;&emsp;成功校准，串口打印如下。最大可输入角度值为123.367°。（**最小的可输入角度永远为0°**）
```basic
[adjust]: Now start calibrating the motor.
[adjust]: Calibrate the motor successfully!
+-------------------------+
|  motor adjust result    |
|  min angle: 0           |
|  max angle: 123.367     |
+-------------------------+
```
<a name="YKFN4"></a>
### 2.上电校准失败
&emsp;&emsp;如果电机没有掉设定值1/2的速度，那么会认为没有堵转。当旋转时间超过宏 **ADJUST_TIMEOUT** ms，大致是输出轴旋转半圈多一点的时候，会自动停下。串口打印如下，
```basic
[adjust]: Now start calibrating the motor.
[adjust]: Motor calibration timeout!
```
<a name="Qbdjd"></a>
### 3.指令3 校准
&emsp;&emsp;不过校准失败还是成功都是可以通过指令重新校准的。如果本来校准正确，但重新校准失败，那么不会保留之前成功的数据。
```basic
[adjust]: Now start calibrating the motor.
[adjust]: Calibrate the motor successfully!
+-------------------------+
|  motor adjust result    |
|  min angle: 0           |
|  max angle: 37.812      |
+-------------------------+
```
<a name="yNcEm"></a>
### 4.指令1 设置角度
&emsp;&emsp;在我们正确校准完角度后，可以使用指令1来控制电机的角度闭环。设置时输入超校准范围的角度值时，会钳位在最小和最大的区间内。
```basic
[CMD 1]: set pos 00.000
+------------------------------------+
|  PID steady state: 00.805 (angle)  |
+------------------------------------+
```
巡逻时也无法设置角度`[CMD 1]: Motor is occupied by Patrol function.`
<a name="YNJkG"></a>
### 5.指令2 设置巡逻角度
&emsp;&emsp;默认是0~90°的来回折返。通过发送指令2，可以修改折返点的角度值，同样会被钳位在校准角度范围内。
```basic
[CMD 2]: start 10.000, end 37.812
```
<a name="dDelV"></a>
### 6.指令5/6 开始/结束巡逻
&emsp;&emsp;巡逻功能开启的必要条件是电机成功校准完。
```basic
[CMD 5]: Start patrol.
[patrol]: ready ... ...
[patrol]: start angle 10.000, end angle 37.812
}}}}}}}=================>>>
[patrol]: now in start (angle 10.611)
<<<================={{{{{{{
[patrol]: now in end   (angle 35.591)
+------------------------------------+
|  PID steady state: 37.063 (angle)  |
+------------------------------------+
}}}}}}}=================>>>
[patrol]: now in start (angle 12.284)
+------------------------------------+
|  PID steady state: 10.876 (angle)  |
+------------------------------------+
<<<================={{{{{{{
[patrol]: now in end   (angle 35.513)
+------------------------------------+
|  PID steady state: 36.974 (angle)  |
+------------------------------------+
[CMD 6]: Finish patrol.
```
<a name="TBMCm"></a>
### 7.指令7 列出所有指令
&emsp;&emsp;有点鸡肋。
```basic
[CMD 7]: Prints all command information.

+-----------------------------------------------------------------+
|        head     |   cmd   |  data1  |  data2  |      tail       |
|-----------------+---------+---------+---------+-----------------|
| 0xABAA (uint16) | (uint8) | (float) | (float) | 0x55CD (uint16) |
|-----------------------------------------------------------------|
| $ cmd1: Set the position of the Angle loop.                     |
| $ cmd2: Set up two positions for patrol.                        |
| $ cmd3: Start the calibration function.                         |
| $ cmd4: Set the max current and max speed.                      |
| $ cmd5: Start patrol function.                                  |
| $ cmd6: End patrol function.                                    |
| $ cmd7: Printing Help Information.                              |
| $ cmd8: Reset microcontroller.                                  |
+-----------------------------------------------------------------+
```
<a name="IeOFk"></a>
### 8.指令8 复位单片机
&emsp;&emsp;复位后电机开始旋转，重新校准。
```basic
[CMD 8]: Reset microcontroller.
[adjust]: Now start calibrating the motor.
```
<a name="BQMiT"></a>
### 9.指令4 设置最大电流/速度
```basic
[CMD 4]: I 10000.0, V 6000.0
```
<a name="L85nL"></a>
## sample
<a name="JlvoB"></a>
### PID_sample.c
&emsp;&emsp;通过VOFA+上位机显示波形。然后在keil debug时，修改 en_angle_loop 值来切换速度闭环和角度闭环。<br />(从速度闭环切到角度闭环时,注意清空 M3508.dji.loop 变量。否则设定角度闭环时，会转到loop为0时的角度才会停下。) 然后通过修改watch窗口的值（M3508的值）实现调参的功能。
<br />&emsp;&emsp;**速度环的调试波形如下，**<br />![image.png](/doc/speed.png)
<br />&emsp;&emsp;**角度环的调试波形如下，**<br />![image.png](/doc/angle.png)
<a name="ZPdHP"></a>
### cmd_sample.c
&emsp;&emsp;简易版的测试协议代码。输入cmd1~4就会返回发送的data1和data2。
```basic
CMD 1: set pos 00.000
CMD 2: min 10.000, max 55.000
CMD 3: Now start calibrating the motor.
CMD 4: I 10000.0, V 6000.0
```
<a name="wyRy9"></a>
### adjust_sample.c
&emsp;&emsp;（如今写完 all_fun_sample.c，该文件比较鸡肋）
<br />&emsp;&emsp;上电后，电机会先顺时针旋转，寻找角度的最小值。然后在堵转后（依据掉速来判断），切换旋转方向。然后遇到最大位置边界，便会停止到边界的最小值上。在校准完后，通过在debug窗口修改test_angle变量值，则可以测试角度闭环（基于校准好后的角度）。
<br />&emsp;&emsp;校准失败后，需要重新复位才能重新开始校准。第一次失败，第二次成功的串口输出。
```basic
[adjust]: Now start calibrating the motor.
[adjust]: Motor calibration timeout!

[adjust]: Now start calibrating the motor.
[adjust]: Calibrate the motor successfully!
+-------------------------+
|  motor adjust result    |
|  min angle: 0           |
|  max angle: 103.573     |
+-------------------------+
```
