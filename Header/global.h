/*
 * global.h
 *
 *  Created on: 2021-5-11
 *      Author: ioa
 * 日志
 * \n 2022-03-11, 1.0.1, zly, MFSK基带符号长度由宏定义LONG_SYMBOL控制改成可配置参数控制phy_mfsk_symbol_bb，支持符号长度256、512、1024，涉及TIME_CYCLE_PREFIX_MAX、CARRIER_NUM、CONTROL_FRAME_CYCLE_PREFIX、DATA_FRAME_CYCLE_PREFIX
 */
#ifndef GLOBAL_H_
#define GLOBAL_H_

#include "stdint.h"

// 调试信息网络输出
#define DEBUG_MFSK			//!< MFSK调试信息输出
#define DEBUG_PHY			//!< 物理层调试信息输出
#define DEBUG_NET			//!< 网络层调试信息输出
#define DEBUG_FH     //temp for test 2016-10-10

//#define MPSK_NARROW_BAND	//!< mfsk带宽减为中心频率的1/4,ts101doppler_compensation函数里gen_carrier函数的参数doppler要多加一个*2
#define LDPC_MOD 			//!< 是否支持LDPC通信制式

#define PI                3.1415926f// π
#ifdef LDPC_MOD 			//!< 是否支持LDPC通信制式

//#define LDPC_HIGH_RATE

#ifdef LDPC_HIGH_RATE
#define LDPC_DATA_NUM 8//8选4
#else
#define LDPC_DATA_NUM 12//12选2
#endif
#endif

#define CARRIER_NUM(SYMBOL_BB) 		  	(SYMBOL_BB/256*120)

#endif /* GLOBAL_H_ */
