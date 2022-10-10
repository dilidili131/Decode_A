/*
* mfsk_dem.h
*
*  Created on: 2021-7-1
*      Author: ioa
*/

#ifndef MFSK_LONG_DEM_H_
#define MFSK_LONG_DEM_H_

//! MFSK解调状态
typedef enum tagMFSK_LONG_DEM_STATE
{
    MFSK_LONG_FALLCHIRP_SYNC,	//!< MFSK降chirp匹配
    MFSK_LONG_RISECHIRP_SYNC, 	//!< MFSK升chirp匹配
    MFSK_LONG_DEM_READY,		 	//!< MFSK同步完成，准备解调帧数据
    MFSK_LONG_DEM_FRAME,		 	//!< MFSK解调帧数据
    MXSK_LONG_FRAME_READY,   	//!< MXSK帧数据准备好，在同步模式下有效
    MFSK_LONG_DEM_WAIT_AUX,	 	//!< MFSK同步通道解调完成，等待其他通道解调完毕
    MFSK_LONG_DEM_COMPLETE,	 	//!< MFSK解调译码完成
    MFSK_LONG_DEM_ERROR		 	//!< MFSK解调错误
}MFSK_LONG_DEM_STATE;
/* ------------------------------------ 接口函数声明 ----------------------------------------- */
//! MFSK解调译码初始化，该函数在系统初始化时调用一次即可
//! 该函数内部调用MFSK_DemReset
extern void MFSK_Long_DemInit(unsigned int nNum);

//! MFSK接收解调译码复位
//! 说明：1. 各通道接收状态机全部复位
//! 	  2. 恢复默认解调译码参数：解调帧数设为1，解码方式针对重复编码，循环前缀长度设为20ms
extern void MFSK_Long_DemReset(unsigned char reset);//是否复位各通道接收状态

//! 设置和获取MFSK基带符号长度
void MFSK_Long_SetMFSKSymbolBB(unsigned int num);
unsigned int MFSK_Long_GetMFSKSymbolBB(void);

//! 设置和获取MFSK中心频率
void MFSK_Long_SetMFSKSignalSystem(unsigned int f0);
unsigned int MFSK_Long_GetMFSKSignalSystem(void);

//! 设置和获取解调帧数
//! 注意：在进入解调前必须设置解调帧数
//! \param[in] nNum 解调帧数
//! return 实际设置的解调帧数
extern unsigned int MFSK_Long_DemSetFrameNum(unsigned int nNum);
extern unsigned int MFSK_Long_DemGetFrameNum(void);

//! 设置和获取MFSK解调循环前缀长度，单位ms，范围4~20ms，超出该范围则默认为20ms
//! 注意：在进入解调前必须设置循环前缀长度！！！否则为默认设置
//! return 实际设置的循环前缀长度
extern unsigned int MFSK_Long_DemSetCyclePrefixLen(unsigned int ms);
extern unsigned int MFSK_Long_DemGetCyclePrefixLen(void);

//! 设置和获取MFSK解码方式
//! 注意：在进入解调前必须设置解码方式！！！否则为默认设置
//! \param[in] bIsRepeatCode true：针对重复编码进行解码，false：针对非重复编码进行解码
extern void MFSK_Long_DemSetDecodeMode(unsigned char bIsRepeatCode);
extern unsigned char MFSK_Long_DemGetDecodeMode(void);

//! 设置MFSK接收状态机状态
//! \param[in] state
//! return 实际设置的接收机状态
extern MFSK_LONG_DEM_STATE MFSK_Long_DemSetRecvState(MFSK_LONG_DEM_STATE state);
extern MFSK_LONG_DEM_STATE MFSK_Long_DemGetRecvState(void);

//! 设置和获取解调每帧符号数
//! 注意：在进入解调前必须设置解调每帧符号数
//! \param[in] nNum 解调每帧符号数
//! return 实际设置的解调每帧符号数
extern unsigned int MFSK_Long_DemSetSymbolNum(unsigned int nNum);
extern unsigned int MFSK_Long_DemGetSymbolNum(void);
/*!
* 解调状态机
* \param[in] dem_mode 模式选择，0为MFSK解调译码模式，1为MXSK同步模式，输出多帧基带帧数据
* \param[out] pDecodedData 存放MFSK解调译码后的字节数组指针，最后两个字节数据为crc16校验值
* \param[out] nOutLen 存放MFSK解调译码后的字节数组长度
* \param[in] sync_bb_data 基带数据源地址
* \return 解调译码状态，详见“MFSK_LONG_DEM_STATE”枚举定义
*/
extern MFSK_LONG_DEM_STATE MFSK_Long_DemRoll(unsigned char dem_mode, unsigned char is_DATA, char *pDecodedData, unsigned int *nOutLen, float *sync_bb_data);

//! 设置MXSK通信信号采样点数
extern unsigned int MXSK_Long_DemSetDataSampleNum(unsigned int len);


#endif /* MFSK_LONG_DEM_H_ */