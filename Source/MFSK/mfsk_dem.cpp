/*
 * mfsk_dem.c
 *
 *  Created on: 2021-7-1
 *      Author: ioa
 * 日志
 *支持符号长度256、512、1024，主要涉及CZT变换
 */
#include "stdafx.h"
#include "dsplib.h"
#include "string.h"
#include "math.h"
#include "global.h"
#include "MFSK/mfsk_dem.h"
#include "LDPC/q_ldpc.h"
#include "EnDecode/endecode.h"
#include "Net/enet_shell.h"
#include "MPSK/mpsk_dem.h"
/* -------------------------------------- 宏定义 -------------------------------------- */
#define CHIRP_LENGTH_BB_MAX_EXP	8 						 // 基带chirp点数相对于2的指数次幂
#define GUARD_LENGTH_BB_MAX	256              // 基带保护间隔
#define CHIRP_LENGTH_BB_MAX	256	             // 基带chirp
//循环前缀相关
#define CYCLE_PREFIX_BB_MAX(SYMBOL_BB) (SYMBOL_BB*3)		// 基带最大循环前缀，小数取整
#define CYCLE_PREFIX_BB_MIN 32 					 	 	 // 基带最小循环前缀，小数取整
#define TIME_CYCLE_PREFIX_MAX(F0,SYMBOL_BB)		(CYCLE_PREFIX_BB_MAX(SYMBOL_BB)*1000/F0) // 默认循环前缀最大时间，ms，非整数取整  @FS_PBkHz,changed by zly 2017_03_29
#define TIME_CYCLE_PREFIX_MIN(F0)	 	(CYCLE_PREFIX_BB_MIN*1000/F0) // 默认循环前缀最小时间，ms，非整数取整  @FS_PBkHz
#define SYMBOL_BB_MAX 1024

#define MXSK_SIGNAL_MAX_BB_LEN 16000 					 // 基带mxsk信号最大长度2秒@8kHz采样率

#define SYNC_GATE				CHIRP_LENGTH_BB_MAX_EXP	 // sqrt(1/SYNC_GATE)同步门限
#define MFSK_MAX_FRAME_NUM 		63						 // 支持的最大解调帧数
// czt
#define FS(F0) 					(F0*8)	// AD sample rate for pass band
#define FC(F0) 					F0	// carrier frequency
#define FS_BB(F0)					F0	// baseband processing sample rate

#define FFT_N(SYMBOL_BB)			(SYMBOL_BB*8)		// the mfsk fft size in AD sample rate
#define IC0(SYMBOL_BB)				(SYMBOL_BB/256*196)			// the lowest freq bin position
#define CZT_M(SYMBOL_BB) 			(SYMBOL_BB/256*120+1) 			// czt carrier number
#define CZT_N(SYMBOL_BB) 			(SYMBOL_BB)			// number of input baseband samples
#define CZT_L(SYMBOL_BB) 			(SYMBOL_BB*2)		// czt fft size
#define LOG_L(SYMBOL_BB)			(SYMBOL_BB/256+8)			//

#define MIN_DOPPLER_UPDATE 		0.00001f		// 0.015m/s	Doppler deviation=0.1Hz(@10kHz)(MFSK carrier sepeartion=39Hz)
#define MAX_DOPPLER 			0.01f		// 15m/s
#define VALID_FLAG 0x1234
#define SIZE_CFR16 8

#ifdef LDPC_MOD 			//!< 是否支持LDPC通信制式
#define MAX_ITER_DATA 10//add by zly 2017_03_02LDPC迭代译码最大次数
#define MAX_ITER_RTS 5//add by zly 2017_03_02LDPC迭代译码最大次数
#endif
#define INF 32767
#define scaler_Hadamard (1.0f)
#define scaler_24choose2 (0.52f)
#define scaler_12choose2 (0.8f)
#define scaler_8choose4 (0.5f)

#define Powcoff (1<<30)
/* ----------------------------------- 结构体定义 ------------------------------------ */
// czt
typedef struct tag_czt_table_struct
{
    float C[CZT_N(SYMBOL_BB_MAX)*2]; 			// initialized
    float Wnn[CZT_N(SYMBOL_BB_MAX)*2]; 		// initialized
    float H[CZT_L(SYMBOL_BB_MAX)*2]; 			// initialized
    float twi_table[CZT_L(SYMBOL_BB_MAX)]; 	// initialized
    float F[CZT_L(SYMBOL_BB_MAX)*2]; 			// temporatory
    int32_t flag_initialized;
}czt_table_struct;

typedef struct tag_mfsk_demod_struct
{
    czt_table_struct czt_table;
    float doppler;
    float A[2];
    float W[2];
    float Wsqrt[2];
}mfsk_demod_struct;

typedef struct ty_mfsk_cfr_sync_PAR
{
    float cfr16_0[CHIRP_LENGTH_BB_MAX*2];
    float cfr16_1[CHIRP_LENGTH_BB_MAX*2*2];
    float CARRIER_0[CARRIER_NUM(SYMBOL_BB_MAX)];
    float CARRIER_1[CARRIER_NUM(SYMBOL_BB_MAX)];
} MFSK_CFR_SYNC_PAR;//CHIRP_LENGTH_BB_MAX*24+CARRIER_NUM*8，最大16k
#pragma pack(push)//保存对齐状态
#pragma pack(8)
unsigned char data_a_space[SYMBOL_BB_MAX/512*20*1024+16]={0};//开辟出一块片内空间，类似SDRAM_BANK2的使用
#pragma pack(pop)//恢复对齐状态
static float* cfr16_0 = ((MFSK_CFR_SYNC_PAR*)data_a_space)->cfr16_0;
static float* cfr16_1 = ((MFSK_CFR_SYNC_PAR*)data_a_space)->cfr16_1;
static float* CARRIER_0= ((MFSK_CFR_SYNC_PAR*)data_a_space)->CARRIER_0;
static float* CARRIER_1= ((MFSK_CFR_SYNC_PAR*)data_a_space)->CARRIER_1;
#pragma pack(push)//保存对齐状态
#pragma pack(8)
static float CorrPowSecond2[CHIRP_LENGTH_BB_MAX*3];//4通道共用,相关需要前后各填CHIRP_LENGTH_BB_MAX-1个0
#pragma pack(pop)//恢复对齐状态
/* ---------------------------- 全局变量声明 ------------------------------------*/
// 白化序列，周期512
static const short whiten_seq[512] =
        {
#include "mfsk_dem_para/whiten_seq.par"
        };
/* ---------------------------------- 局部函数声明 ------------------------------------ */
static short find_shift(unsigned int value);

// czt
static void update_doppler(mfsk_demod_struct *s1, float doppler);
static void mfsk_demod(mfsk_demod_struct *s, float input[],float output[], float doppler, int *out_exp);
static void czt_intializing(czt_table_struct *czt_table, float* A, float* W, float* Wsqrt);
static void czt_processing(czt_table_struct *czt_table, float in[], float out[]);
// 调幅
static void wave_range(float input[], unsigned int len, float output[], float para);
/* ---------------------------------- 局部变量定义 ------------------------------------ */
static unsigned int mfsk_symbol_bb = 256;//MFSK基带符号长度
static unsigned int mfsk_signal_system = 10;//中心频率
static unsigned int mfsk_FrameNum = 1; 			// 接收总帧数，默认为1帧
static unsigned int mfsk_SymbolNum = 12;  // 每帧的符号数（注意！：与Frame_Data缓存大小相关）

static unsigned int mfsk_CyclePrefixBB = 160;		 // 基带循环前缀，默认设置为20ms@8kHz采样率
static unsigned int mfsk_SymbolLengthBB = 256 + 160; // 基带符号长度，默认设置为52ms@8kHz采样率
static const unsigned int mfsk_GuardLengthBB = GUARD_LENGTH_BB_MAX;	 // 基带保护间隔，默认设置为62ms@8kHz采样率

static unsigned char mfsk_bIsRepeatCode = 1;	// 是否重复编码，1为重复，0为不重复

static unsigned int mfsk_SyncSampleNum = 0;
static unsigned int sync_sample_num = 0;

// 基带头帧信号缓存，目前将数据放在外部sdram中，用于计算信号均值和均方根
static float sync_pBPDataHeadFram[32*1024];
#pragma pack(push)//保存对齐状态
#pragma pack(8)
// 匹配滤波器
static const float fir_corr_fall_chirp[CHIRP_LENGTH_BB_MAX*2] = // 降chirp匹配滤波系数
        {
#include "mfsk_dem_para/fallbbchirp_256.par"
        };
static const float fir_corr_rise_chirp[CHIRP_LENGTH_BB_MAX*2] =// 升chirp匹配滤波系数
        {
#include "mfsk_dem_para/risebbchirp_256.par"
        };
static float fir_corr_delay_sync_fall[CHIRP_LENGTH_BB_MAX*2];	// 同步通道降chirp匹配滤波器延时
static float fir_corr_delay_sync_rise[CHIRP_LENGTH_BB_MAX*2];	// 同步通道升chirp匹配滤波器延时
#pragma pack(pop)//恢复对齐状态
// 同步通道同步和解调相关变量
static MFSK_DEM_STATE sync_StateRecv = MFSK_FALLCHIRP_SYNC;    // 同步通道状态
static unsigned char wait_rise_chirp_num = 0; //匹配到第1路降chirp后，等待升chirp组数
static unsigned int sync_DemDataReady = 0; 		 // 同步通道数据准备解码标志
#pragma pack(push)//保存对齐状态
#pragma pack(8)
static float sync_FrameData[(MXSK_SIGNAL_MAX_BB_LEN + GUARD_LENGTH_BB_MAX*3 + CHIRP_LENGTH_BB_MAX + CHIRP_LENGTH_BB_MAX*4)*2]; 	// 同步通道帧数据缓存
#pragma pack(pop)//恢复对齐状态
static unsigned int sync_FrameIndex = 0; 																	// 同步通道帧数据索引
static unsigned int sync_FrameNo = 0; 																	// 同步通道帧号

#pragma pack(push)//保存对齐状态
#pragma pack(8)
static unsigned int sync_SigPowOld[CHIRP_LENGTH_BB_MAX]; 			// 同步通道信号能量
static unsigned int sync_SigPowNew[CHIRP_LENGTH_BB_MAX];
static unsigned int sync_SigPowAvg[CHIRP_LENGTH_BB_MAX*2];  		// 同步通道信号平均能量
static unsigned int sync_CorrPow[CHIRP_LENGTH_BB_MAX*3]; 			// 同步通道相关能量
static float sync_CorrPowSecond1[CHIRP_LENGTH_BB_MAX];
static float CorrPowSecResult[CHIRP_LENGTH_BB_MAX*2];  	// 通道二次相关结果
#pragma pack(pop)//恢复对齐状态
static uint64_t sync_SigPowSum = 0;  				   			// 同步通道信号能量和
static unsigned int sync_CorrPowPeak = 0; 							// 同步通道相关能量峰值
static unsigned int sync_CorrPowPeakPos1 = 0; 						// 同步通道相关能量峰值位置1
static unsigned int sync_CorrPowPeakPos2 = 0; 						// 同步通道相关能量峰值位置2
static short CorrPowShift = 0;
static float CorrPowSecPeak = 0; 						// 通道二次相关峰值
static short CorrPowSecPeakPos = 0; 					// 通道二次相关峰值位置

static float sync_RiseChirpCorr = 0.0;	// mfsk升chirp头相关系数

#pragma pack(push)//保存对齐状态
#pragma pack(8)
// czt相关
static mfsk_demod_struct czt_struct_sync;
#pragma pack(pop)//恢复对齐状态

// 哈达码译码
static const short Hadamard_32x20[32][20] =
        {
#include "mfsk_dem_para/hadamard_32x20.par"
        };
static const short Hadamard_Index_32select1[32] =
        {
#include "mfsk_dem_para/hadamard_index_32select1.par"
        };
#ifdef LDPC_MOD 			//!< 是否支持LDPC通信制式
// MT_MFSK DATA载波符号LLR转换
#ifdef LDPC_HIGH_RATE
static const short Nchoosek_data[64][LDPC_DATA_NUM] =
{
	#include "mfsk_dem_para/nchoosek64x8.par"
};
#else
static const short Nchoosek_data[64][LDPC_DATA_NUM] =
{
	#include "mfsk_dem_para/nchoosek64x12.par"
};
#endif
// MT_MFSK RTS载波符号LLR转换
static const short Nchoosek_256x24[256][24] =
{
	#include "mfsk_dem_para/nchoosek256x24_rts.par"
};
static short l_chan_sync[120 * 63 * 64];
static unsigned int l_chan_index_sync = 0;
static unsigned int Nchoosek_MatchSum[256];
static unsigned char ite_num = 0;
#endif

static unsigned int Hadamard_MatchSum[32];
static unsigned int Hadamard_MatchSumMax = 0;
static short Hadamard_MatchSumMaxIndex = 0;

static short decision_hard0_sync[120*12/20*MFSK_MAX_FRAME_NUM]; 		// 同步通道哈达码硬判决结果，5bit流，最大支持64帧MFSK数据
static short decision_hard1_sync[120*12/20*MFSK_MAX_FRAME_NUM]; 		// 同步通道哈达码硬判决结果，5bit流，最大支持64帧MFSK数据
static unsigned int decision_hard0_index_sync = 0;

// 对偶码译码
static unsigned int decision_soft0_sync[120*12/20*32*MFSK_MAX_FRAME_NUM]; // 同步通道软判决信息，最大支持64帧MFSK数据
static unsigned int decision_soft1[120*12/20*32*MFSK_MAX_FRAME_NUM]; 	 // 解交织后的软判决信息，最大支持64帧MFSK数据
static unsigned int decoded_data[120*12/20*MFSK_MAX_FRAME_NUM];     		 // 解对偶码后的数据，5bit流，最大支持64帧MFSK数据

static unsigned int decision_soft0_len_sync = 0;
static unsigned int decision_soft0_index_sync = 0;

static const short dualk5_table[32] = {0, 17, 1, 16, 2, 19, 3, 18, 4, 21, 5, 20, 6, 23, 7, 22, 8, 25, 9, 24, 10, 27, 11, 26, 12, 29, 13, 28, 14, 31, 15, 30};
static short dualk5_state = 0; // 初始状态

// 兼容MPSK 同步相关变量
// 信号长度
static unsigned int mxsk_demDataLen = 0;
static unsigned int sync_FrameDataReady = 0;
typedef struct
{
    float doppler[4];
    float mpsk_bb_data[4][(2136*2*2)];
}MPSK_DEM_STRUCT;
float mpsk_data1_buf[1024*31];
float mpsk_data2_buf[1024*34];
MPSK_DEM_STRUCT *mpsk_in_float = (MPSK_DEM_STRUCT *)mpsk_data2_buf;
/* ------------------------------------- 接口函数实现 ----------------------------------------- */
void MFSK_DemInit(unsigned int nNum)
{
    memset(CorrPowSecond2,0,sizeof(CorrPowSecond2));//4通道共用,相关需要前后各填CHIRP_LENGTH_BB_MAX-1个0
    MFSK_DemSetSymbolNum(nNum);
    MFSK_DemReset(1);
    MPSK_DemInit();
}

void MFSK_DemReset(unsigned char reset)//是否复位各通道接收状态
{
    // 清匹配滤波器状态
    memset(fir_corr_delay_sync_fall, 0, sizeof(fir_corr_delay_sync_fall));
    memset(fir_corr_delay_sync_rise, 0, sizeof(fir_corr_delay_sync_rise));

    // 清能量状态缓存
    sync_SigPowSum = 0;
    memset(sync_SigPowOld, 0, sizeof(sync_SigPowOld));
    memset(sync_SigPowNew, 0, sizeof(sync_SigPowNew));
    memset(sync_CorrPow, 0, sizeof(sync_CorrPow));
    memset(sync_SigPowAvg, 0, sizeof(sync_SigPowAvg));

    decision_soft0_len_sync = 0;
    decision_soft0_index_sync = 0;

    decision_hard0_index_sync = 0;
#ifdef LDPC_MOD 			//!< 是否支持LDPC通信制式
    l_chan_index_sync = 0;
#endif
    if(reset==1)
    {
        sync_StateRecv = MFSK_FALLCHIRP_SYNC;
    }

    sync_FrameNo = 0;
    sync_DemDataReady = 0;// 清“通道数据准备好”标志位

    mfsk_FrameNum = 1;
    mfsk_CyclePrefixBB = 160;
    mfsk_SymbolLengthBB = mfsk_symbol_bb + 160;
    mfsk_bIsRepeatCode = 1;

    mxsk_demDataLen = mfsk_SymbolLengthBB * mfsk_SymbolNum; // 兼容mpsk

    mfsk_SyncSampleNum = 0;
    sync_sample_num = 0;

    // czt初始化
    czt_struct_sync.czt_table.flag_initialized = 0;
}

unsigned int MFSK_DemGetSyncSampleNum(void)
{
    return mfsk_SyncSampleNum;
}

void MFSK_DemResetSyncSampleNum(void)
{
    mfsk_SyncSampleNum = 0;
    sync_sample_num = 0;
}
//! 设置和获取MFSK基带符号长度
void MFSK_SetMFSKSymbolBB(unsigned int length)
{
    mfsk_symbol_bb = length;
}
unsigned int MFSK_GetMFSKSymbolBB(void)
{
    return mfsk_symbol_bb;
}
//! 设置和获取MFSK中心频率
void MFSK_SetMFSKSignalSystem(unsigned int f0)
{
    mfsk_signal_system = f0;
}
unsigned int MFSK_GetMFSKSignalSystem(void)
{
    return mfsk_signal_system;
}
unsigned int MFSK_DemSetFrameNum(unsigned int nNum)
{
    if( (nNum == 0) || (nNum > MFSK_MAX_FRAME_NUM) )
    {
        mfsk_FrameNum = 1;
    }
    else
    {
        mfsk_FrameNum = nNum;
    }

    return mfsk_FrameNum;
}

unsigned int MFSK_DemGetFrameNum(void)
{
    return mfsk_FrameNum;
}

void MFSK_DemSetDecodeMode(unsigned char bIsRepeatCode)
{
    mfsk_bIsRepeatCode = bIsRepeatCode;
}

unsigned char MFSK_DemGetDecodeMode(void)
{
    return mfsk_bIsRepeatCode;
}

MFSK_DEM_STATE MFSK_DemSetRecvState(MFSK_DEM_STATE state)
{
    if(state==MFSK_RISECHIRP_SYNC)
    {
        wait_rise_chirp_num = 5;
    }
    sync_StateRecv = state;

    return sync_StateRecv;
}

MFSK_DEM_STATE MFSK_DemGetRecvState(void)
{
    return sync_StateRecv;
}

// 获取mfsk同步头相关系数
float MFSK_DemGetSyncCorr(void)
{
    return sync_RiseChirpCorr;
}

unsigned int MFSK_DemSetSymbolNum(unsigned int nNum)
{
    if(nNum == 0)
    {
        mfsk_SymbolNum = 256*12/mfsk_symbol_bb;
        return mfsk_SymbolNum;
    }

    mfsk_SymbolNum = nNum;
    return mfsk_SymbolNum;
}

unsigned int MFSK_DemGetSymbolNum(void)
{
    return mfsk_SymbolNum;
}

// 设置循环前缀长度，单位毫秒
unsigned int MFSK_DemSetCyclePrefixLen(unsigned int ms)
{
    if(ms < TIME_CYCLE_PREFIX_MIN(mfsk_signal_system) || ms > TIME_CYCLE_PREFIX_MAX(mfsk_signal_system,mfsk_symbol_bb))
    {
        ms = TIME_CYCLE_PREFIX_MAX(mfsk_signal_system,mfsk_symbol_bb);
    }
    mfsk_CyclePrefixBB = ms * mfsk_signal_system/1000;
    mfsk_SymbolLengthBB = mfsk_CyclePrefixBB + mfsk_symbol_bb;

    mxsk_demDataLen = mfsk_SymbolLengthBB * mfsk_SymbolNum; // 兼容mpsk

    return ms;
}

// 兼容MPSK同步
unsigned int MXSK_DemSetDataSampleNum(unsigned int len)
{
    if(len > MXSK_SIGNAL_MAX_BB_LEN)
    {
        return 0;
    }

    mxsk_demDataLen = len;
    return mxsk_demDataLen;
}

MFSK_DEM_STATE MFSK_DemRoll(unsigned char dem_mode, unsigned char is_DATA, char *pDecodedData, unsigned int *nOutLen, float *sync_bb_data)
{
    unsigned int i = 0, j = 0, k = 0, l = 0, m = 0, n = 0;

    float doppler_factor = 0.0; // 多普勒拉伸压缩系数

    INT32_DATA *p1Int32Data = 0;
    INT32_DATA *p2Int32Data = 0;

    unsigned int bit5 = 0;
    unsigned int bit = 0;
    unsigned int len = 0;
    unsigned int *p_bit5;
    char *p_char;
    float temp1 = 0;
    float temp2 = 0;
    short temp_int16 = 0;
    unsigned char max_ite = 5;
#ifdef MPSK_NARROW_BAND	//!< mpsk带宽减为中心频率的1/4
    float *p_cf_temp;
#endif
    sync_sample_num += GUARD_LENGTH_BB_MAX*8;
    //---------------------------------------------------------------------------------------------
    // 同步
    switch(sync_StateRecv)
    {
        case MFSK_FALLCHIRP_SYNC: // 搜索降chirp
            // 降chirp匹配滤波
            DSPF_sp_blk_move(fir_corr_delay_sync_fall,cfr16_1,CHIRP_LENGTH_BB_MAX*2);
            DSPF_sp_blk_move(sync_bb_data, cfr16_1 + (CHIRP_LENGTH_BB_MAX - 1) * 2, CHIRP_LENGTH_BB_MAX * 2);
            DSPF_sp_blk_move(cfr16_1+CHIRP_LENGTH_BB_MAX*2,fir_corr_delay_sync_fall,CHIRP_LENGTH_BB_MAX*2);
            DSPF_sp_fir_cplx(cfr16_1+(CHIRP_LENGTH_BB_MAX-1)*2, fir_corr_fall_chirp, cfr16_0, CHIRP_LENGTH_BB_MAX, CHIRP_LENGTH_BB_MAX);

            for(i = 0; i < CHIRP_LENGTH_BB_MAX; i++)
            {
                // 计算信号能量
                sync_SigPowNew[i] = (unsigned int)floorf((cfr16_1[(i+CHIRP_LENGTH_BB_MAX-1)*2] * cfr16_1[(i+CHIRP_LENGTH_BB_MAX-1)*2] + cfr16_1[(i+CHIRP_LENGTH_BB_MAX-1)*2+1] * cfr16_1[(i+CHIRP_LENGTH_BB_MAX-1)*2+1])*Powcoff);
                // 计算相关能量
                sync_CorrPow[i] = (unsigned int)floorf((cfr16_0[i*2] * cfr16_0[i*2] + cfr16_0[i*2+1] * cfr16_0[i*2+1])*Powcoff);

                // 计算信号平均能量
                sync_SigPowSum = sync_SigPowSum + sync_SigPowNew[i] - sync_SigPowOld[i];
                sync_SigPowAvg[i] = sync_SigPowSum >> CHIRP_LENGTH_BB_MAX_EXP;
            }
            memcpy(sync_SigPowOld, sync_SigPowNew, CHIRP_LENGTH_BB_MAX * 4); // 保存旧的信号能量

            // 同步头搜索
            for( i = 0; i < CHIRP_LENGTH_BB_MAX; i++ )
            {
                // 过门限判决：y > x * 0.125
                if((sync_CorrPow[i] >= 64) && ((sync_CorrPow[i] * SYNC_GATE) > sync_SigPowAvg[i]))
                {
#ifdef DEBUG_MFSK
                    {
						float temp = 0;
						temp = sqrtf((float)sync_CorrPow[i] / sync_SigPowAvg[i]);
						LOG1("物理层：Sync fall chirp head found! %.2f(根号)", temp);
					}
#endif

                    // 此帧数据提前进入升chirp匹配状态
                    memset(fir_corr_delay_sync_rise, 0, sizeof(fir_corr_delay_sync_rise)); // 清匹配滤波器延时
                    DSPF_sp_blk_move(fir_corr_delay_sync_rise,cfr16_1,CHIRP_LENGTH_BB_MAX*2);
                    DSPF_sp_blk_move(cfr16_1+CHIRP_LENGTH_BB_MAX*2,fir_corr_delay_sync_rise,CHIRP_LENGTH_BB_MAX*2);

                    // 匹配升chirp
                    DSPF_sp_fir_cplx(cfr16_1+(CHIRP_LENGTH_BB_MAX-1)*2, fir_corr_rise_chirp, cfr16_0, CHIRP_LENGTH_BB_MAX, CHIRP_LENGTH_BB_MAX);

                    // 计算并保存相关能量
                    for(j = 0; j < CHIRP_LENGTH_BB_MAX; j++)
                    {
                        sync_CorrPow[j + CHIRP_LENGTH_BB_MAX] = (unsigned int)floorf((cfr16_0[j*2] * cfr16_0[j*2] + cfr16_0[j*2+1] * cfr16_0[j*2+1])*Powcoff);
                    }
                    sync_StateRecv = MFSK_RISECHIRP_SYNC;
                    wait_rise_chirp_num = 3;
                    break;
                }
            }
            break;
        case MFSK_RISECHIRP_SYNC: // 搜索升chirp
            wait_rise_chirp_num --;

            // 升chirp匹配滤波
            DSPF_sp_blk_move(fir_corr_delay_sync_rise,cfr16_1,CHIRP_LENGTH_BB_MAX*2);
            DSPF_sp_blk_move(sync_bb_data, cfr16_1 + (CHIRP_LENGTH_BB_MAX - 1) * 2, CHIRP_LENGTH_BB_MAX * 2);
            DSPF_sp_blk_move(cfr16_1+CHIRP_LENGTH_BB_MAX*2,fir_corr_delay_sync_rise,CHIRP_LENGTH_BB_MAX*2);
            DSPF_sp_fir_cplx(cfr16_1+(CHIRP_LENGTH_BB_MAX-1)*2, fir_corr_rise_chirp, cfr16_0, CHIRP_LENGTH_BB_MAX, CHIRP_LENGTH_BB_MAX);

            memcpy(sync_CorrPow, sync_CorrPow + CHIRP_LENGTH_BB_MAX, CHIRP_LENGTH_BB_MAX * 4); // 保存旧的相关系数
            for( i = 0, j = CHIRP_LENGTH_BB_MAX; i < CHIRP_LENGTH_BB_MAX; i++, j++)
            {
                // 计算此帧信号能量
                sync_SigPowNew[i] = (unsigned int)floorf((cfr16_1[(i+CHIRP_LENGTH_BB_MAX-1)*2] * cfr16_1[(i+CHIRP_LENGTH_BB_MAX-1)*2] + cfr16_1[(i+CHIRP_LENGTH_BB_MAX-1)*2+1] * cfr16_1[(i+CHIRP_LENGTH_BB_MAX-1)*2+1])*Powcoff);
                // 计算此帧相关系数
                sync_CorrPow[j] = (unsigned int)floorf((cfr16_0[i*2] * cfr16_0[i*2] + cfr16_0[i*2+1] * cfr16_0[i*2+1])*Powcoff);

                // 计算信号平均能量
                sync_SigPowSum = sync_SigPowSum + sync_SigPowNew[i] - sync_SigPowOld[i];
                sync_SigPowAvg[i] = sync_SigPowSum >> CHIRP_LENGTH_BB_MAX_EXP;
            }
            memcpy(sync_SigPowOld, sync_SigPowNew, CHIRP_LENGTH_BB_MAX * 4); // 保存旧的信号能量

            // 升chirp搜索
            for(i = 0; i < CHIRP_LENGTH_BB_MAX; i++)
            {
                if((sync_CorrPow[i + CHIRP_LENGTH_BB_MAX] >= 64) && (sync_CorrPow[i + CHIRP_LENGTH_BB_MAX] * SYNC_GATE) > sync_SigPowAvg[i]) // 过门限判决：y > x * 0.125
                {
#ifdef DEBUG_MFSK
                    LOG("物理层：Sync rise chirp head found!");
#endif

                    // 2013-04-20移至此处
                    if(sync_sample_num < CHIRP_LENGTH_BB_MAX*16) // 2012-08-29修正
                    {
                        mfsk_SyncSampleNum = sync_sample_num + (i - 8) * 8 - CHIRP_LENGTH_BB_MAX*8;
                    }
                    else
                    {
                        mfsk_SyncSampleNum = sync_sample_num + (i - 8) * 8 - CHIRP_LENGTH_BB_MAX*16;
                    }

                    // 同步头搜索示意图

                    // |________|_____^__|________|  // sync_CorrPow数据组成，假设^为峰值位置
                    //    512       512     512
                    //     旧        新      冗余

                    // 保存过门限位置
                    sync_CorrPowPeakPos1 = i + CHIRP_LENGTH_BB_MAX;

                    sync_FrameIndex = 0;
                    sync_FrameNo = 0;
                    j = i * 8; // 此处认为是通带过门限位置不正确 待修改

                    // 从过门限位置处开始保存基带帧数据
                    for(; i < CHIRP_LENGTH_BB_MAX; i++)
                    {
                        sync_FrameData[sync_FrameIndex*2] = cfr16_1[(i+CHIRP_LENGTH_BB_MAX-1)*2];
                        sync_FrameData[sync_FrameIndex*2+1] = cfr16_1[(i+CHIRP_LENGTH_BB_MAX-1)*2+1];
                        sync_FrameIndex++;
                    }
                    sync_StateRecv = MFSK_DEM_READY;
                    break;
                }
            }
            if(sync_StateRecv == MFSK_RISECHIRP_SYNC)
            {
                if(wait_rise_chirp_num==0)//同步通道搜索升chirp超时
                {
                    MFSK_DemReset(1);
                }
            }
            break;
        case MFSK_DEM_READY:
            // 升chirp匹配滤波
            DSPF_sp_blk_move(fir_corr_delay_sync_rise,cfr16_1,CHIRP_LENGTH_BB_MAX*2);
            DSPF_sp_blk_move(sync_bb_data, cfr16_1 + (CHIRP_LENGTH_BB_MAX - 1) * 2, CHIRP_LENGTH_BB_MAX * 2);
            DSPF_sp_blk_move(cfr16_1+CHIRP_LENGTH_BB_MAX*2,fir_corr_delay_sync_rise,CHIRP_LENGTH_BB_MAX*2);
            DSPF_sp_fir_cplx (cfr16_1+(CHIRP_LENGTH_BB_MAX-1)*2, fir_corr_rise_chirp, cfr16_0, CHIRP_LENGTH_BB_MAX, CHIRP_LENGTH_BB_MAX);

            // 2013-01-06 add
            for( i = 0, j = CHIRP_LENGTH_BB_MAX*2; i < CHIRP_LENGTH_BB_MAX; i++, j++)
            {
                // 计算此帧信号能量
                sync_SigPowNew[i] = (unsigned int)floorf((cfr16_1[(i+CHIRP_LENGTH_BB_MAX-1)*2] * cfr16_1[(i+CHIRP_LENGTH_BB_MAX-1)*2] + cfr16_1[(i+CHIRP_LENGTH_BB_MAX-1)*2+1] * cfr16_1[(i+CHIRP_LENGTH_BB_MAX-1)*2+1])*Powcoff);

                // 求一次冗余CorrPow并保存，见上面的示意图。目的是寻找相关峰值
                sync_CorrPow[j] = (unsigned int)floorf((cfr16_0[i*2] * cfr16_0[i*2] + cfr16_0[i*2+1] * cfr16_0[i*2+1])*Powcoff);

                // 计算信号平均能量
                sync_SigPowSum = sync_SigPowSum + sync_SigPowNew[i] - sync_SigPowOld[i];
                sync_SigPowAvg[i + CHIRP_LENGTH_BB_MAX] = sync_SigPowSum >> CHIRP_LENGTH_BB_MAX_EXP;

                // 保存帧数据
                sync_FrameData[sync_FrameIndex*2] = cfr16_1[(i+CHIRP_LENGTH_BB_MAX-1)*2];
                sync_FrameData[sync_FrameIndex*2+1] = cfr16_1[(i+CHIRP_LENGTH_BB_MAX-1)*2+1];
                sync_FrameIndex++;
            }

            // 峰值搜索
            sync_CorrPowPeak = 0; // 初始化sync_CorrPowPeak，2011-11-09日修正
            for(i = sync_CorrPowPeakPos1; i < ( sync_CorrPowPeakPos1 + CHIRP_LENGTH_BB_MAX/2 ); i++)
            {
                if(sync_CorrPow[i] > sync_CorrPowPeak)
                {
                    sync_CorrPowPeak = sync_CorrPow[i];
                    k = i;
                }
            }

            sync_RiseChirpCorr = sqrtf( (((float)sync_CorrPowPeak) / sync_SigPowAvg[k - CHIRP_LENGTH_BB_MAX]) );
#ifdef DEBUG_MFSK
            LOG2("物理层：sync rise chirp [corr = %.2f]，滑动能量窗0x%x，升chirp到达时间%x", sync_RiseChirpCorr,sync_SigPowAvg[k - CHIRP_LENGTH_BB_MAX]);
#endif

            CorrPowShift = find_shift(sync_CorrPowPeak); // 减少二次相关精度损失
            sync_CorrPowPeakPos1 = k - sync_CorrPowPeakPos1;  // 求出实际峰值位置，以便去除帧数据中多余的数据

            for(i = 0, j = k - CHIRP_LENGTH_BB_MAX/2; i < CHIRP_LENGTH_BB_MAX; i++, j++)
            {
                sync_CorrPowSecond1[i] =  (float)(sync_CorrPow[j] >> CorrPowShift)/ 0x8000;
            }

            sync_StateRecv = MFSK_DEM_FRAME;
            //LOG1("数据长度：%d", mxsk_demDataLen);
            break;
        case MFSK_DEM_FRAME:;
            j = 0; // j变量在本状态下专用，其它代码勿用！
            // 存储Symbol_Num个符号基带数据
            while( ( sync_FrameIndex < ( sync_CorrPowPeakPos1 + mfsk_GuardLengthBB + mxsk_demDataLen + mfsk_GuardLengthBB + CHIRP_LENGTH_BB_MAX + CHIRP_LENGTH_BB_MAX/2 + CHIRP_LENGTH_BB_MAX ) ) && ( j < CHIRP_LENGTH_BB_MAX ) )
            {
                sync_FrameData[sync_FrameIndex * 2] = *(sync_bb_data+j * 2);
                sync_FrameData[sync_FrameIndex * 2 + 1] = *(sync_bb_data + j * 2 + 1);
                sync_FrameIndex++;
                j++;
            }

            // 是否存够一帧数据，包括两个保护间隔、Symbol_Num个符号、第二个补偿用chirp，和备用冗余数据
            if(sync_FrameIndex == ( sync_CorrPowPeakPos1 + mfsk_GuardLengthBB + mxsk_demDataLen + mfsk_GuardLengthBB + CHIRP_LENGTH_BB_MAX + CHIRP_LENGTH_BB_MAX/2 + CHIRP_LENGTH_BB_MAX ) )
            {
                ///////////////////////////////////////////////////////////////////////////////////////////
                // 提前匹配CHIRP_LENGTH_BB_MAX点
                memset(cfr16_1,0,sizeof(fir_corr_delay_sync_rise));
                DSPF_sp_blk_move(&sync_FrameData[(sync_FrameIndex - CHIRP_LENGTH_BB_MAX*3)*2],cfr16_1+(CHIRP_LENGTH_BB_MAX-1)*2,CHIRP_LENGTH_BB_MAX*2);
                //DSPF_sp_blk_move(cfr16_1+CHIRP_LENGTH_BB_MAX*2,fir_corr_delay_sync_rise,(CHIRP_LENGTH_BB_MAX-1)*2);
                DSPF_sp_fir_cplx(cfr16_1+(CHIRP_LENGTH_BB_MAX-1)*2, fir_corr_rise_chirp, cfr16_0, CHIRP_LENGTH_BB_MAX, CHIRP_LENGTH_BB_MAX);
                for( i = 0; i < CHIRP_LENGTH_BB_MAX; i++ )
                {
                    sync_CorrPow[i] = (unsigned int)floorf((cfr16_0[i*2] * cfr16_0[i*2] + cfr16_0[i*2+1] * cfr16_0[i*2+1])*Powcoff);
                }

                // chirp匹配, 得到sync_CorrPowPeakPos2和sync_CorrPowSecond2
                DSPF_sp_blk_move(&sync_FrameData[(sync_FrameIndex - (CHIRP_LENGTH_BB_MAX*3-1))*2],cfr16_1,CHIRP_LENGTH_BB_MAX*4);
                //DSPF_sp_blk_move(cfr16_1+CHIRP_LENGTH_BB_MAX*2,fir_corr_delay_sync_rise,(CHIRP_LENGTH_BB_MAX-1)*2);
                DSPF_sp_fir_cplx(cfr16_1+(CHIRP_LENGTH_BB_MAX-1)*2, fir_corr_rise_chirp, cfr16_0, CHIRP_LENGTH_BB_MAX, CHIRP_LENGTH_BB_MAX);
                sync_CorrPowPeak = 0;
                for(i = 0; i < CHIRP_LENGTH_BB_MAX; i++)
                {
                    sync_CorrPow[i + CHIRP_LENGTH_BB_MAX] = (unsigned int)floorf((cfr16_0[i*2] * cfr16_0[i*2] + cfr16_0[i*2+1] * cfr16_0[i*2+1])*Powcoff);
                    if( sync_CorrPow[i + CHIRP_LENGTH_BB_MAX] > sync_CorrPowPeak ) // 峰值搜索
                    {
                        sync_CorrPowPeak = sync_CorrPow[ i + CHIRP_LENGTH_BB_MAX ];
                        sync_CorrPowPeakPos2 = i; // 保存峰值位置
                    }
                }

                CorrPowShift = find_shift( sync_CorrPowPeak );// 确保二次相关不溢出并保证最大精度
                for(i = 0; i < CHIRP_LENGTH_BB_MAX; i++)
                {
                    CorrPowSecond2[i+CHIRP_LENGTH_BB_MAX-1] = sqrtf((float)( sync_CorrPow[ i + CHIRP_LENGTH_BB_MAX ] >> CorrPowShift )/ 0x8000);
                }

                for(i = 0; i <= CHIRP_LENGTH_BB_MAX/2; i++)
                {
                    temp1 = sqrtf(sync_CorrPowSecond1[i]);
                    sync_CorrPowSecond1[i] = sqrtf(sync_CorrPowSecond1[CHIRP_LENGTH_BB_MAX - i]); // 反转
                    sync_CorrPowSecond1[CHIRP_LENGTH_BB_MAX - i] = temp1;
                }

                // 开始解调同步通道的一帧数据
                ///////////////////////////////////////////////////////////////////
                // 二次相关
                DSPF_sp_convol(CorrPowSecond2, sync_CorrPowSecond1, CorrPowSecResult, CHIRP_LENGTH_BB_MAX, CHIRP_LENGTH_BB_MAX*2-1);

                //////////////////////////////////////////////////////////////////
                // 计算拉伸压缩系数
                CorrPowSecPeak = 0;
                CorrPowSecPeakPos = 0;
                for( i = CHIRP_LENGTH_BB_MAX/2; i < CHIRP_LENGTH_BB_MAX*3/2; i++ )
                {
                    if( CorrPowSecResult[i] > CorrPowSecPeak )
                    {
                        CorrPowSecPeak = CorrPowSecResult[i];
                        CorrPowSecPeakPos = i;
                    }
                }
                doppler_factor = (CorrPowSecPeakPos - CHIRP_LENGTH_BB_MAX) / (mfsk_GuardLengthBB + mxsk_demDataLen + mfsk_GuardLengthBB + CHIRP_LENGTH_BB_MAX+0.0f);
                //doppler_factor = 0.0;
                if(dem_mode == 0)//mfsk
                {
                    // 解调MFSK一帧数据，一帧包含mfsk_SymbolNum个符号
                    for(i = 0; i < mfsk_SymbolNum; i++)
                    {
                        // 解调一个符号的数据
                        mfsk_demod(&czt_struct_sync, sync_FrameData + (sync_CorrPowPeakPos1 + mfsk_GuardLengthBB + (mfsk_CyclePrefixBB / 2) + k)*2, cfr16_0, doppler_factor, (int*)&n );//200~300k个cycles

                        for( n = 0; n < CARRIER_NUM(mfsk_symbol_bb); n++ )
                        {
                            CARRIER_0[n] = sqrtf(cfr16_0[n*2]*cfr16_0[n*2]+cfr16_0[n*2+1]*cfr16_0[n*2+1]);
                        }
                        DSPF_sp_mat_trans(CARRIER_0, 20, CARRIER_NUM(mfsk_symbol_bb)/20, CARRIER_1);

                        // 生成译码软判决信息
                        for(n = 0; n < CARRIER_NUM(mfsk_symbol_bb)/20; n++)
                        {
                            Hadamard_MatchSumMax = 0;
                            Hadamard_MatchSumMaxIndex = 0;

                            // 哈达码匹配
                            for( m = 0; m < 32; m++ )
                            {
                                temp1 = 0;
                                for( l = 0; l < 20; l++ )
                                {
                                    if( Hadamard_32x20[m][l] == 1 )
                                    {
                                        temp1 += CARRIER_1[ n * 20 + l];
                                    }
                                }
                                Hadamard_MatchSum[m] = floorf(temp1*scaler_Hadamard);
                                if( Hadamard_MatchSumMax <  Hadamard_MatchSum[m] )
                                {
                                    Hadamard_MatchSumMax = Hadamard_MatchSum[m];
                                    Hadamard_MatchSumMaxIndex = m;
                                }

                                // 软判决信息，每20bit生成32个匹配值
                                decision_soft0_sync[decision_soft0_index_sync++] = Hadamard_MatchSum[m];
                            }
                            decision_soft0_len_sync++;

                            // 哈达码硬判决生成5bit流
                            decision_hard0_sync[decision_hard0_index_sync++] = Hadamard_Index_32select1[Hadamard_MatchSumMaxIndex];
                        }

                        k += ( mfsk_symbol_bb + mfsk_CyclePrefixBB);
                    }
                }
                else if(dem_mode == 1) // 同步模式 输出基带同步后的帧数据和多普勒拉伸压缩系数
                {
#ifdef MPSK_NARROW_BAND	//!< mpsk带宽减为中心频率的1/4
                    p_cf_temp = sync_FrameData + (sync_CorrPowPeakPos1 + mfsk_GuardLengthBB)*2;
					for(i = 0; i < mxsk_demDataLen/2; i++)
					{
						memcpy(&mpsk_in_float->mpsk_bb_data[0][i*2], p_cf_temp, 8);
						p_cf_temp+=4;
					}
					memcpy(&mpsk_in_float->doppler[0], &doppler_factor, 4);
#else
                    memcpy(&mpsk_in_float->mpsk_bb_data[0][0] , sync_FrameData + (sync_CorrPowPeakPos1 + mfsk_GuardLengthBB)*2, mxsk_demDataLen * 8);
                    memcpy(&mpsk_in_float->doppler[0], &doppler_factor, 4);
#endif
                    sync_FrameDataReady = 1;
                }
#ifdef LDPC_MOD 			//!< 是否支持LDPC通信制式
                else if(dem_mode == 2)//mt_mfsk
				{
					// 解调MFSK一帧数据，一帧包含mfsk_SymbolNum个符号
					for(i = 0; i < mfsk_SymbolNum; i++)
					{
						// 解调一个符号的数据
						mfsk_demod(&czt_struct_sync, sync_FrameData + (sync_CorrPowPeakPos1 + mfsk_GuardLengthBB + (mfsk_CyclePrefixBB / 2) + k)*2, cfr16_0, doppler_factor, (int*)&n );//200~300k个cycles
					  	if(is_DATA==1)
					  	{
					  		for( n = 0; n < CARRIER_NUM(mfsk_symbol_bb); n++ )
							{
								CARRIER_0[n] = sqrtf(cfr16_0[n*2]*cfr16_0[n*2]+cfr16_0[n*2+1]*cfr16_0[n*2+1]);
							}
					  		DSPF_sp_mat_trans(CARRIER_0, LDPC_DATA_NUM, CARRIER_NUM(mfsk_symbol_bb)/LDPC_DATA_NUM, CARRIER_1);
					  		// 生成译码软判决信息
							for(n = 0; n < CARRIER_NUM(mfsk_symbol_bb)/LDPC_DATA_NUM; n++)
							{
								// 码匹配
								for( m = 0; m < 64; m++ )
								{
									temp1 = 0;
									for( l = 0; l < LDPC_DATA_NUM; l++ )
									{
										if( Nchoosek_data[m][l] == 1 )
										{
											temp1 += CARRIER_1[ n * LDPC_DATA_NUM + l];
										}
									}
									#ifdef LDPC_HIGH_RATE
									Nchoosek_MatchSum[m] = floorf(temp1*scaler_8choose4);
									#else
									Nchoosek_MatchSum[m] = floorf(temp1*scaler_12choose2);
									#endif
									// 软判决信息，每8或12bit生成64个匹配值
									if(Nchoosek_MatchSum[m]>INF)
									{
										l_chan_sync[l_chan_index_sync++] = INF;
										#ifdef DEBUG_MFSK
										ENetShell_DebugInfo("物理层：数据mfsk符号LLR超过%x！",Nchoosek_MatchSum[m]);
										#endif
									}
									else
									{
										l_chan_sync[l_chan_index_sync++] = Nchoosek_MatchSum[m];
									}
								}
							}
					  	}
						else
						{
							for( n = 0; n < CARRIER_NUM(mfsk_symbol_bb); n++ )
							{
								CARRIER_0[n] = sqrtf(cfr16_0[n*2]*cfr16_0[n*2]+cfr16_0[n*2+1]*cfr16_0[n*2+1]);
							}
							DSPF_sp_mat_trans(CARRIER_0, 24, CARRIER_NUM(mfsk_symbol_bb)/24, CARRIER_1);
							// 生成译码软判决信息
							for(n = 0; n < CARRIER_NUM(mfsk_symbol_bb)/24; n++)
							{
								// 码匹配
								for( m = 0; m < 256; m++ )
								{
									temp1 = 0;
									for( l = 0; l < 24; l++ )
									{
										if( Nchoosek_256x24[m][l] == 1 )
										{
											temp1 += CARRIER_1[ n * 24 + l];
										}
									}
									Nchoosek_MatchSum[m] = floorf(temp1*scaler_24choose2);
									// 软判决信息，每24bit生成256个匹配值，5个12bit共生成5*256个匹配值
									if(Nchoosek_MatchSum[m]>INF)
									{
										l_chan_sync[l_chan_index_sync++] = INF;
										#ifdef DEBUG_MFSK
										ENetShell_DebugInfo("物理层：控制帧mfsk符号LLR超过1%x！",Nchoosek_MatchSum[m]);
										#endif
									}
									else
									{
										l_chan_sync[l_chan_index_sync++] = Nchoosek_MatchSum[m];
									}
								}
							}
						}

						k += (mfsk_symbol_bb + mfsk_CyclePrefixBB);
					}
				}
#endif
                sync_FrameNo++;

                // 测试用 ==========================================================================================================
#ifdef DEBUG_MFSK
                ENetShell_DebugInfo("物理层：sync %12.6f %03d  %02d frame", doppler_factor, CorrPowSecPeakPos, sync_FrameNo);
#endif
                // 测试用 ==========================================================================================================

                if(sync_FrameNo == mfsk_FrameNum) // 数据帧接收完毕
                {
                    sync_DemDataReady = 1;
                    if(is_DATA==0)//开始后面可能有的数据帧保存
                    {
                        // 将相关系数2保存为相关系数1，准备下次二次相关
                        DSPF_sp_blk_move(&sync_FrameData[(sync_FrameIndex - (CHIRP_LENGTH_BB_MAX*2-1))*2],cfr16_1,CHIRP_LENGTH_BB_MAX*4);
                        //DSPF_sp_blk_move(cfr16_1+CHIRP_LENGTH_BB_MAX*2,fir_corr_delay_sync_rise,(CHIRP_LENGTH_BB_MAX-1)*2);
                        DSPF_sp_fir_cplx(cfr16_1+(CHIRP_LENGTH_BB_MAX-1)*2, fir_corr_rise_chirp, cfr16_0, CHIRP_LENGTH_BB_MAX, CHIRP_LENGTH_BB_MAX);
                        for(i = 0; i < CHIRP_LENGTH_BB_MAX; i++)
                        {
                            sync_CorrPow[i + CHIRP_LENGTH_BB_MAX*2] = (unsigned int)floorf((cfr16_0[i*2] * cfr16_0[i*2] + cfr16_0[i*2+1] * cfr16_0[i*2+1])*Powcoff);
                        }
                        for(i = 0; i < CHIRP_LENGTH_BB_MAX; i++)
                        {
                            sync_CorrPowSecond1[i] = (float)(sync_CorrPow[sync_CorrPowPeakPos2 + CHIRP_LENGTH_BB_MAX/2 + i] >> CorrPowShift)/0x8000;
                        }

                        // 开始保存下一帧数据
                        for(i = 0; i < CHIRP_LENGTH_BB_MAX*2 - sync_CorrPowPeakPos2; i++)
                        {
                            sync_FrameData[i*2] = sync_FrameData[(sync_FrameIndex - CHIRP_LENGTH_BB_MAX*2 + sync_CorrPowPeakPos2 + i)*2];
                            sync_FrameData[i*2+1] = sync_FrameData[(sync_FrameIndex - CHIRP_LENGTH_BB_MAX*2 + sync_CorrPowPeakPos2 + i)*2+1];
                        }

                        sync_FrameIndex = CHIRP_LENGTH_BB_MAX*2 - sync_CorrPowPeakPos2;
                        sync_CorrPowPeakPos1 = 0;

                        for(; j < CHIRP_LENGTH_BB_MAX; j++)
                        {
                            sync_FrameData[sync_FrameIndex * 2] = *(sync_bb_data + j * 2);
                            sync_FrameData[sync_FrameIndex * 2 + 1] = *(sync_bb_data + j * 2+1);
                            sync_FrameIndex++;
                        }
                    }
                }
                else
                {
                    // 将相关系数2保存为相关系数1，准备下次二次相关
                    DSPF_sp_blk_move(&sync_FrameData[(sync_FrameIndex - (CHIRP_LENGTH_BB_MAX*2-1))*2],cfr16_1,CHIRP_LENGTH_BB_MAX*4);
                    //DSPF_sp_blk_move(cfr16_1+CHIRP_LENGTH_BB_MAX*2,fir_corr_delay_sync_rise,(CHIRP_LENGTH_BB_MAX-1)*2);
                    DSPF_sp_fir_cplx(cfr16_1+(CHIRP_LENGTH_BB_MAX-1)*2, fir_corr_rise_chirp, cfr16_0, CHIRP_LENGTH_BB_MAX, CHIRP_LENGTH_BB_MAX);
                    for(i = 0; i < CHIRP_LENGTH_BB_MAX; i++)
                    {
                        sync_CorrPow[i + CHIRP_LENGTH_BB_MAX*2] = (unsigned int)floorf((cfr16_0[i*2] * cfr16_0[i*2] + cfr16_0[i*2+1] * cfr16_0[i*2+1])*Powcoff);
                    }
                    for(i = 0; i < CHIRP_LENGTH_BB_MAX; i++)
                    {
                        sync_CorrPowSecond1[i] = (float)(sync_CorrPow[sync_CorrPowPeakPos2 + CHIRP_LENGTH_BB_MAX/2 + i] >> CorrPowShift)/0x8000;
                    }

                    // 开始保存下一帧数据
                    for(i = 0; i < CHIRP_LENGTH_BB_MAX*2 - sync_CorrPowPeakPos2; i++)
                    {
                        sync_FrameData[i*2] = sync_FrameData[(sync_FrameIndex - CHIRP_LENGTH_BB_MAX*2 + sync_CorrPowPeakPos2 + i)*2];
                        sync_FrameData[i*2+1] = sync_FrameData[(sync_FrameIndex - CHIRP_LENGTH_BB_MAX*2 + sync_CorrPowPeakPos2 + i)*2+1];
                    }

                    sync_FrameIndex = CHIRP_LENGTH_BB_MAX*2 - sync_CorrPowPeakPos2;
                    sync_CorrPowPeakPos1 = 0;

                    for(; j < CHIRP_LENGTH_BB_MAX; j++)
                    {
                        sync_FrameData[sync_FrameIndex * 2] = *(sync_bb_data + j * 2);
                        sync_FrameData[sync_FrameIndex * 2 + 1] = *(sync_bb_data + j * 2 +1);
                        sync_FrameIndex++;
                    }
                }
            }
            break;
    }
    // --------------------------------------------------------------------------------------------------------------------------------------
    // mfsk解调模式
    // 如果各通道帧数据准备好则开始解调MFSK
    if(dem_mode == 0)
    {
        if(sync_DemDataReady == 1)
        {
            sync_DemDataReady = 0;// 清“通道数据准备好”标志位

            // 解交织
            Endecode_32IntDeIntlev((INT32_DATA*)decision_soft0_sync, decision_soft0_len_sync, (INT32_DATA*)decision_soft1);
            Endecode_5bitDeIntlev2(decision_hard0_sync, decision_hard0_index_sync, decision_hard1_sync);

            if(mfsk_bIsRepeatCode==1) // 为重复编码，因此解调后的数据要前后两两叠加
            {
                p1Int32Data = p2Int32Data = (INT32_DATA*)decision_soft1;

                decision_soft0_len_sync = decision_soft0_len_sync / 2;

                for(i = 0; i < decision_soft0_len_sync; i++)
                {
                    for(j = 0; j < 32; j++)
                    {
                        p1Int32Data->data[j] = (p2Int32Data->data[j]) + (p2Int32Data + 1)->data[j];
                    }
                    p2Int32Data += 2;
                    p1Int32Data += 1;
                }
            }

            // 软判决解对偶码，得到5bit流
            Endecode_DeDualK5(decision_soft1, decoded_data, decision_soft0_len_sync / 2, 1);

#ifdef DEBUG_MFSK
            if(mfsk_bIsRepeatCode==1)
			{
				for(i = 0, j = 0; i < decision_soft0_len_sync / 2; i++)
				{
					temp_int16 = decoded_data[i];

					decision_hard0_sync[j++] = (temp_int16 ^ dualk5_state);
					decision_hard0_sync[j++] = (temp_int16 ^ dualk5_state);

					decision_hard0_sync[j++] = (dualk5_table[temp_int16] ^ dualk5_state);
					decision_hard0_sync[j++] = (dualk5_table[temp_int16] ^ dualk5_state);

					dualk5_state = temp_int16;
				}
			}
			else
			{
				for(i = 0, j = 0; i < decision_soft0_len_sync / 2; i++)
				{
					temp_int16 = decoded_data[i];

					decision_hard0_sync[j++] = (temp_int16 ^ dualk5_state);
					decision_hard0_sync[j++] = (dualk5_table[temp_int16] ^ dualk5_state);

					dualk5_state = temp_int16;
				}
			}
			j = 0;
			for(i = 0; i < decision_hard0_index_sync; i++)
			{
				if(decision_hard0_sync[i] != decision_hard1_sync[i])
				{
					j++;
				}
			}

			LOG3("物理层(mfsk)：收到%d个符号字，错误%d个，错误率%.4f", decision_hard0_index_sync, j, ((float)j) / decision_hard0_index_sync);
#endif

            // 解白化
            j = 0;
            len = decision_soft0_len_sync / 2 - 1; // 有效5bit数
            for(i = 0; i < len; i++)
            {
                bit5 = 0;

                // 形成5bit白化数据
                for(k = 0; k < 5; k++)
                {
                    bit5 |= whiten_seq[j++] << k;
                    if(j == 512) j = 0;
                }
                decoded_data[i] ^= bit5;
            }

            // 根据有效比特数将5比特流转为字节，并分离16bit的crc校验值
            len = len * 5; // 有效比特数
            p_char = pDecodedData;
            p_bit5 = decoded_data;
            *p_char = 0;
            j = 0;
            k = 0;
            n = 0;
            for(i = 0; i < len; i++)
            {
                bit = (*p_bit5 >> j++) & 0x0000001;
                *p_char |= bit << k++;

                if( (k == 8) || i == (len - 16 - 1) )
                {
                    p_char++;
                    *p_char = 0;
                    k = 0;
                    n++;

                    if(n > 4096) // 2012-12-11 add
                    {
                        break;
                    }
                }
                if(j == 5)
                {
                    j = 0;
                    p_bit5++;
                }
            }
            *nOutLen = n;

            short crc16_0;
            short crc16_1;
            crc16_0 = Endecode_Crc16(pDecodedData, n - 2);
            crc16_1 = pDecodedData[n - 1];
            crc16_1 = crc16_1 << 8 | (unsigned char)pDecodedData[n - 2];

#ifdef DEBUG_MFSK
            if(crc16_0 == crc16_1)
			{
				LOG("物理层(mfsk)：校验正确!");
			}
			else
			{
				LOG("物理层(mfsk)：校验错误!");
			}
#endif

            MFSK_DemReset(0);

            //	FILE *fp;
            //	unsigned int len = 0;
            //	len = (unsigned int)((unsigned int)pPPDataSave - (unsigned int)0x1000000);
            //	fp = fopen("D:\\ppdata_sample_problem.pcm", "wb");
            //	fwrite((char*)0x1000000, sizeof(short), len, fp);
            //	fclose(fp);

            //	pPPDataSave = (short*)0x1000000;

            return MFSK_DEM_COMPLETE;
        }
    }
        // --------------------------------------------------------------------------------------------------------------------------------------
        // 同步模式 兼容MXSK
    else if(dem_mode == 1)
    {
        // 如果各通道帧数据准备好
        if(sync_FrameDataReady == 1)
        {
            sync_FrameDataReady = 0;// 清“通道数据准备好”标志位
#ifdef MPSK_NARROW_BAND
            memcpy(&mpsk_in_float->mpsk_bb_data[1][0], &mpsk_in_float->mpsk_bb_data[0][0], mxsk_demDataLen * 4);
			memcpy(&mpsk_in_float->doppler[1], &doppler_factor, 4);

			memcpy(&mpsk_in_float->mpsk_bb_data[2][0], &mpsk_in_float->mpsk_bb_data[0][0], mxsk_demDataLen * 4);
			memcpy(&mpsk_in_float->doppler[2], &doppler_factor, 4);

			memcpy(&mpsk_in_float->mpsk_bb_data[3][0], &mpsk_in_float->mpsk_bb_data[0][0], mxsk_demDataLen * 4);
			memcpy(&mpsk_in_float->doppler[3], &doppler_factor, 4);
			}
#else
            memcpy(&mpsk_in_float->mpsk_bb_data[1][0], &mpsk_in_float->mpsk_bb_data[0][0], mxsk_demDataLen * 8);
            memcpy(&mpsk_in_float->doppler[1], &doppler_factor, 4);

            memcpy(&mpsk_in_float->mpsk_bb_data[2][0], &mpsk_in_float->mpsk_bb_data[0][0], mxsk_demDataLen * 8);
            memcpy(&mpsk_in_float->doppler[2], &doppler_factor, 4);

            memcpy(&mpsk_in_float->mpsk_bb_data[3][0], &mpsk_in_float->mpsk_bb_data[0][0], mxsk_demDataLen * 8);
            memcpy(&mpsk_in_float->doppler[3], &doppler_factor, 4);
#endif
            temp1 = DSPF_sp_vecsum_sq(&mpsk_in_float->mpsk_bb_data[0][0], 2136*2*2*4);
            temp2 = sqrt((2136*2*2*4)/temp1);
            wave_range(&mpsk_in_float->mpsk_bb_data[0][0], 2136*2*2*4, &mpsk_in_float->mpsk_bb_data[0][0], temp2*0.707);
            mpsk_receiver(&mpsk_in_float->mpsk_bb_data[0][0], pDecodedData, &mpsk_in_float->doppler[0], sync_FrameNo);// in_float占用data2_buf
            return MXSK_FRAME_READY;
        }

        if(sync_DemDataReady == 1)
        {
            sync_DemDataReady = 0;// 清“通道数据准备好”标志位

            MFSK_DemReset(0);
            return MFSK_DEM_COMPLETE;
        }
        return sync_StateRecv;
    }
#ifdef LDPC_MOD 			//!< 是否支持LDPC通信制式
    // --------------------------------------------------------------------------------------------------------------------------------------
	// mt_mfsk解调模式
	// 如果各通道帧数据准备好则开始解调MFSK
	else if(dem_mode == 2)
	{
		if(sync_DemDataReady == 1)
		{
			sync_DemDataReady = 0;// 清“通道数据准备好”标志位
			ite_num = 0;
			max_ite = is_DATA == 1 ? MAX_ITER_DATA : MAX_ITER_RTS;
			while (1)
			{
				*nOutLen = ldpc_decoder(l_chan_sync, (unsigned char *)pDecodedData, mfsk_FrameNum, mfsk_SymbolNum, ite_num, is_DATA, mfsk_symbol_bb);//DATA输入960*64，输出240字节,校验正确返回长度，校验错误返回0;RTS输入60*8，输出11字节
				ite_num++;
				if (*nOutLen > 0 || ite_num > max_ite)//校验正确或者达到最大迭代次数
				{
#ifdef DEBUG_MFSK
					if (*nOutLen == 0)
					{
						if (is_DATA == 1)
						{
							LOG1("物理层(mtmfsk)：MT_MFSK数据包校验错误,迭代次数达到%02d!", ite_num - 1);
						}
						else
						{
							LOG1("物理层(mtmfsk)：MT_MFSK控制帧校验错误,迭代次数达到%02d!", ite_num - 1);
						}

					}
#endif
					break;

				}
			}
			MFSK_DemReset(0);
			return MFSK_DEM_COMPLETE;
		}
	}
#endif
    return sync_StateRecv;
}

/* ---------------------------- 局部函数实现 ------------------------------- */
// find_shift
static short find_shift(unsigned int value )
{
    int i = 0;
    int shift = 0;

    for(i = 0; i < 32; i++)
    {
        if( (value & 0x80000000) == 0x80000000)
        {
            shift = i;
            if(shift <= 16)
                shift = 16 - shift + 1 + 4;
            else if( shift > 16 && shift < 20)
                shift = 4 - (shift - 16);
            else
                shift = 0;
            break;
        }
        else
        {
            value <<= 1;
        }
    }

    return shift;
}
// find_right_shift
static unsigned int find_right_shift(unsigned int value )
{
    int i = 0;
    unsigned int shift = 0;

    for(i = 0; i < 32; i++)
    {
        if( (value & 0x80000000) == 0x80000000)
        {
            shift = i;
            if(shift <= 12)
                shift = (12 - shift + 1)/2;
            else
                shift = 0;
            break;
        }
        else
        {
            value <<= 1;
        }
    }

    return shift;
}

// mfsk_demod
// 解调 czt by wyb
static void mfsk_demod(mfsk_demod_struct *czt_s, float input[], float output[], float doppler, int *out_exp)
{
    update_doppler(czt_s, doppler);

    czt_processing(&(czt_s->czt_table), input, output);

    *out_exp = -(int)LOG_L(mfsk_symbol_bb);
}

// update_doppler
// 更新多普勒拉伸压缩系数
static void update_doppler(mfsk_demod_struct *czt_s, float doppler)
{
    float phase_temp = 0;
    int is_update_czt_table = 0;

    //////////////////////////////////
    // 	check  doppler  in range	//
    //////////////////////////////////
    if ( fabsf(doppler) > MAX_DOPPLER )
        doppler = 0.0;

    //////////////////////////////////
    // 	decide whether update		//
    //////////////////////////////////
    if ( czt_s->czt_table.flag_initialized != VALID_FLAG )
        is_update_czt_table = 1;
    else
    {
        if ( fabsf( czt_s->doppler - doppler ) < MIN_DOPPLER_UPDATE )  // doppler variation is too small
            is_update_czt_table = 0;	// no need to update
        else
            is_update_czt_table = 1;
    }

    if( is_update_czt_table == 0 )
        return;

    //////////////////////////////////
    // 	calc A, W, Wsqrt			//
    //////////////////////////////////
    //A=exp(1j*2*pi*(F_0*(1-doppler)-Fc)/Fs_bb);
    //phase_temp = 2.0 * PI * ( FS * IC0 / FFT_N * ( 1.0 - doppler ) - FC ) / FS_BB;  -- 2014-9-15 mask wzk
    phase_temp = 2.0 * PI * ( FS(mfsk_signal_system) * IC0(mfsk_symbol_bb) / FFT_N(mfsk_symbol_bb) * ( 1.0 - doppler ) - FC(mfsk_signal_system) ) / FS_BB(mfsk_signal_system);  // -- 2014-9-15 modify wzk

    czt_s->A[0] = cosf(phase_temp);
    czt_s->A[1] = sinf(phase_temp);

    //W=exp(-1j*2*pi*F_1*(1-doppler)/Fs_bb);
    //phase_temp = -2.0 * PI * FS / FFT_N * ( 1 - doppler ) / FS_BB;  -- 2014-9-15 mask wzk
    phase_temp = -2.0 * PI * FS(mfsk_signal_system) / FFT_N(mfsk_symbol_bb) * ( 1 - doppler ) / FS_BB(mfsk_signal_system); // -- 2014-9-15 modify wzk
    czt_s->W[0] = cosf(phase_temp);
    czt_s->W[1] = sinf(phase_temp);
    czt_s->Wsqrt[0] = cosf(phase_temp/2.0);
    czt_s->Wsqrt[1] = sinf(phase_temp/2.0);

    //////////////////////////////////
    // 	call czt initial			//
    //////////////////////////////////
    czt_intializing( &(czt_s->czt_table), &(czt_s->A[0]), &(czt_s->W[0]), &(czt_s->Wsqrt[0]) );

    //////////////////////////////////
    // 	save doppler value			//
    //////////////////////////////////
    czt_s->doppler = doppler;
}
// czt_intializing
// czt初始化
static void czt_intializing( czt_table_struct *czt_table, float *A, float *W, float *Wsqrt )
{
    float D[2];
    float D0[2];
    float Wnf[2];
    float Wnnf[2];
    float Cf[2];
    float temp[2];

    int i;

    //D0=Wsqrt*conj(A);
    temp[0] = A[0];
    temp[1] = -A[1];
    cmltf(Wsqrt,temp,D0);

    Wnnf[0] = 1.0;
    Wnnf[1] = 0.0;
    czt_table->Wnn[0] = Wnnf[0];
    czt_table->Wnn[1] = Wnnf[1];

    Wnf[0]=1.0;
    Wnf[1]=0.0;

    Cf[0]=1.0;
    Cf[1]=0.0;
    czt_table->C[0]=Cf[0];
    czt_table->C[1]=Cf[1];

    D[0] = D0[0];
    D[1] = D0[1];
    //recursion to calculate Wnn[n]=W^(n^2/2) C[n]=A^(-n)*W^(1/2*n^2)
    temp[0] = Wsqrt[0];
    temp[1] = -Wsqrt[1];
    for( i = 1; i < CZT_N(mfsk_symbol_bb); i++ ) // 迭代
    {
        //Wn=Wn*W;
        cmltf( W, Wnf, Wnf);

        //Wnn(ii)=Wnn(ii-1)*Wn*conj(Wsqrt);
        cmltf( Wnnf, Wnf, Wnnf);
        cmltf( Wnnf, temp, Wnnf);
        czt_table->Wnn[i*2] = Wnnf[0];
        czt_table->Wnn[i*2+1] = Wnnf[1];

        //C(ii)=C(ii-1)*D;
        cmltf( Cf, D, Cf);
        czt_table->C[i*2] =Cf[0];
        czt_table->C[i*2+1] =Cf[1];

        cmltf( Wnf, D0, D);
    }

    //H(1:M)=conj(Wnn(1:M));
    for(i = 0; i < CZT_M(mfsk_symbol_bb); i++)
    {
        czt_table->H[i*2] = czt_table->Wnn[i*2];
        czt_table->H[i*2+1] = -czt_table->Wnn[i*2+1];
    }

    //H(M+1:L-N+1)=0; %#ok<*BDSCI>
    memset( &czt_table->H[CZT_M(mfsk_symbol_bb)*2], 0, (CZT_L(mfsk_symbol_bb) - CZT_N(mfsk_symbol_bb) - CZT_M(mfsk_symbol_bb)) * SIZE_CFR16 );

    //H(L-N+2:L)=conj(Wnn(N:-1:2));
    for (i = CZT_L(mfsk_symbol_bb) - CZT_N(mfsk_symbol_bb) + 1; i < CZT_L(mfsk_symbol_bb); i++)
    {
        czt_table->H[i*2] = czt_table->Wnn[(CZT_L(mfsk_symbol_bb) - i)*2];
        czt_table->H[i*2+1] = -czt_table->Wnn[(CZT_L(mfsk_symbol_bb) - i)*2+1];
    }

    //H(:)=fft(H,L);
    if ( czt_table->flag_initialized != VALID_FLAG )
    {
        gen_w_r2(czt_table->twi_table, CZT_L(mfsk_symbol_bb));
        bit_rev(czt_table->twi_table, CZT_L(mfsk_symbol_bb)>>1);
    }
    DSPF_sp_cfftr2_dit(czt_table->H, czt_table->twi_table, CZT_L(mfsk_symbol_bb));
    //bit_rev(czt_table->H, CZT_L);
    czt_table->flag_initialized = VALID_FLAG;

    return;
}

// czt_processing
// czt
static void czt_processing( czt_table_struct *czt_table, float in[], float out[] )
{
    unsigned int i = 0;

    //F(1:N)=C(1:N).*x;
    for( i = 0; i < CZT_N(mfsk_symbol_bb); i++ )
    {
        cmltf( &czt_table->C[i*2], &in[i*2], &czt_table->F[i*2]);
    }
    memset( &czt_table->F[CZT_N(mfsk_symbol_bb)*2], 0, (CZT_L(mfsk_symbol_bb)-CZT_N(mfsk_symbol_bb))*SIZE_CFR16 );

    //F(:)=fft(F,L);
    DSPF_sp_cfftr2_dit(czt_table->F, czt_table->twi_table, CZT_L(mfsk_symbol_bb));
    //bit_rev(czt_table->F, CZT_L);

    //F(:)=F.*H;
    for( i = 0; i < CZT_L(mfsk_symbol_bb); i++ )
    {
        cmltf( &czt_table->F[i*2], &czt_table->H[i*2], &czt_table->F[i*2]);
    }

    //F(:)=ifft(F,L);
    //bit_rev(czt_table->F, CZT_L);
    DSPF_sp_icfftr2_dif(czt_table->F, czt_table->twi_table, CZT_L(mfsk_symbol_bb));
    // for ifft: The arguments block_exponent and scale_method have been added for future expansion. These arguments are ignored by the function. To avoid overflow, the function scales the output by 1/fft_size.

    //X(:)=F(1:M).*Wnn(1:M);
    for( i = 0; i < CZT_M(mfsk_symbol_bb); i++ )
    {
        cmltf( &czt_table->F[i*2], &czt_table->Wnn[i*2], &czt_table->F[i*2]);
    }

    // 去掉中心频率，即去掉直流分量
    DSPF_sp_blk_move(czt_table->F,out,CARRIER_NUM(mfsk_symbol_bb));
    DSPF_sp_blk_move(czt_table->F+CARRIER_NUM(mfsk_symbol_bb)+2,out+CARRIER_NUM(mfsk_symbol_bb),CARRIER_NUM(mfsk_symbol_bb));
    return;
}
// 调幅
static void wave_range(float input[], unsigned int len, float output[], float para)
{
    unsigned int i = 0;
    for (i = 0; i < len; i++)
    {
        output[i] = input[i] * para;
    }
}