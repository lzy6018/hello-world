/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef ORIGIN_H_
#define ORIGIN_H_

// 协议相关定义
#define ORIGIN_FRAME_LENTH               16    // 指令长度
#define ORIGIN_FRAME_START               0xAA  // 协议帧开始
#define ORIGIN_FRAME_END                 '/'   // 协议帧结束
#define ORIGIN_FRAME_CHECK_BEGIN          1    // 校验码开始的位置 
#define ORIGIN_FRAME_CHECKSUM            14    // 校验码的位置     
#define ORIGIN_FRAME_CHECK_NUM           13    // 需要校验的字节数
#define ORIGIN_FILL_VALUE                0x55  // 填充值

typedef union
{
    char      ch[16];
	double    float64[2];
	float     float32[4];
	int16_t   int16[8];
	int32_t   int32[4];
	int64_t   int64[2];
	uint16_t  uint16[8];
	uint32_t  uint32[4];
	uint64_t  uint64[2];
} format_bytes_t;

typedef union/* 轴编码+指令码联合体 */
{
	uint8_t cmd_var;
	struct
	{
		uint8_t cm:5;
		uint8_t id:2;	
		uint8_t rw:1;			
	}bit;
} cmd_org_t;


#define CONFIG_ID 5
#define CONFIG_DIR 1
#define CONFIG_MODE 2
#define CONFIG_TAO 3
#define CONFIG_I2R 4
#define CONFIG_RATIO 0
#define CONFIG_PZERO 6
#define CONFIG_PMIN 7
#define CONFIG_PMAX 8
#define CONFIG_VMAX 9
#define CONFIG_AMAX 10
#define CONFIG_JMAX 11

typedef struct //控制器的基本参数
{
 volatile unsigned char ID;    // W 电机编号                         /* 0 or 1 */
 volatile   signed char dir;   // W 方向+1或-1                       /* 1 or -1 */
 volatile unsigned char mode;  // W 运行模式                         /* A */
 volatile unsigned char Tao;   // W 电机热电流滤波时间常数，单位0.1s  /* 5 */
 volatile float I2R;    // W 额定电流的平方, 热量估算，最大值
 volatile float Ratio;  // W 360°/(减速比*1圈), 1mm/1圈

 volatile float pZero;  // W 角度0点偏移     /* mm */
 volatile float pMin;   // W 机械最小 角度
 volatile float pMax;   // W 限幅: W 机械最大 角度

 volatile float vMax;   // W 机械最大 速度
 volatile float aMax;   // W 机械最大 加速度
 volatile float jMax;   // W 机械最大 加加速度
} _TPara;


typedef enum
{
    CMD_MTR_BET=0x00,
  CMD_MTR_STA,
  CMD_MTR_CFG,
  CMD_MEC_POS,
  CMD_TRA_CFG,
  CMD_PID_POS,
  CMD_PID_SPD,
  CMD_PID_CUR,
  CMD_WHELL_POS,
  CMD_JOINT_POS,
  CMD_RPM_CUR,
  CMD_CUR
}CTRL_Mode_te;


#include "conf_general.h"
#include "shutdown.h"
#include "mc_interface.h"

uint8_t CheckSum(uint8_t *Ptr, uint8_t Num );


#endif /* APP_H_ */
