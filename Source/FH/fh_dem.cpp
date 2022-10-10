/*
 * fh_dem.c
 *
 *  Created on: 2021-10-26
 *      Author: ioa
 */
#include "stdafx.h"
#include <string.h>
#include "dsplib.h"
#include "math.h"
#include "global.h"
#include "FH/fh_dem.h"
#include "FH/FH_Related_Func.h"
#include "EnDecode/endecode.h"
#include "Net/enet_shell.h"
/* -------------------------------------宏定义------------------------------------------ */
//跳频维特比译码模块
#define VITERBI_DEC_IN_HALF       	1584//240、432

#define RECV_BYTE_NUM																	64
#define RECV_BIT_NUM																	(RECV_BYTE_NUM*8)			//512(64、128)
#define RECV_CRC_IN_NUM																	(RECV_BIT_NUM + 16)			//528(80、144)
#define RECV_DOUBLE_NUM																	(RECV_CRC_IN_NUM * 2)		//1056(160、288)
#define	RHO_TRIGGERGATE 																0.0625
#define SYN_MAX_INDEX																	(FH_CHIRP_BASE_LEN +FH_CHIRP_BASE_LEN +RECV_DOUBLE_NUM * 128)
#define SYN_STATUS_SEARCH_TRIGGER 														0
#define SYN_STATUS_BUFFER_DATA 															1
#define Powcoff (1<<30)
/* -------------------------------------局部变量定义------------------------------------------ */
//跳频维特比译码模块
static const uint16_t Pre_State_Table[2][256] =
        {
                {0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,94,96,98,100,102,104,106,108,110,112,114,116,118,120,122,124,126,128,130,132,134,136,138,140,142,144,146,148,150,152,154,156,158,160,162,164,166,168,170,172,174,176,178,180,182,184,186,188,190,192,194,196,198,200,202,204,206,208,210,212,214,216,218,220,222,224,226,228,230,232,234,236,238,240,242,244,246,248,250,252,254,0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,94,96,98,100,102,104,106,108,110,112,114,116,118,120,122,124,126,128,130,132,134,136,138,140,142,144,146,148,150,152,154,156,158,160,162,164,166,168,170,172,174,176,178,180,182,184,186,188,190,192,194,196,198,200,202,204,206,208,210,212,214,216,218,220,222,224,226,228,230,232,234,236,238,240,242,244,246,248,250,252,254},
                {1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31,33,35,37,39,41,43,45,47,49,51,53,55,57,59,61,63,65,67,69,71,73,75,77,79,81,83,85,87,89,91,93,95,97,99,101,103,105,107,109,111,113,115,117,119,121,123,125,127,129,131,133,135,137,139,141,143,145,147,149,151,153,155,157,159,161,163,165,167,169,171,173,175,177,179,181,183,185,187,189,191,193,195,197,199,201,203,205,207,209,211,213,215,217,219,221,223,225,227,229,231,233,235,237,239,241,243,245,247,249,251,253,255,1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31,33,35,37,39,41,43,45,47,49,51,53,55,57,59,61,63,65,67,69,71,73,75,77,79,81,83,85,87,89,91,93,95,97,99,101,103,105,107,109,111,113,115,117,119,121,123,125,127,129,131,133,135,137,139,141,143,145,147,149,151,153,155,157,159,161,163,165,167,169,171,173,175,177,179,181,183,185,187,189,191,193,195,197,199,201,203,205,207,209,211,213,215,217,219,221,223,225,227,229,231,233,235,237,239,241,243,245,247,249,251,253,255}
        };
static const unsigned char Pre_En_Table1[4][256] =
        {
                {0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0},
                {0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1},
                {0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0},
                {0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1}
        };
static const unsigned char Pre_En_Table2[4][256] =
        {
                {1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1},
                {1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0},
                {1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,0,1,0,1},
                {1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0}
        };
static const unsigned char Pre_In_Table[2][256] =
        {
                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
        };
static unsigned char cnv_surv_dec[256][VITERBI_DEC_IN_HALF] 		= {0};
static uint16_t cnv_surv_state[256][VITERBI_DEC_IN_HALF] 	= {0};
static unsigned char dec_out1[256] 								= {0};
static unsigned char dec_out2[256] 								= {0};
static float weight_final1[256] 							= {0};
static float weight_final2[256]  						= {0};
static float cnv_sum[256] 								= {0};
static float 	weight[16][VITERBI_DEC_IN_HALF] 			= {0};
static float 	cnv_dis_sum_temp[256] 						= {0};

static float fh_chirp[FH_CHIRP_BASE_LEN*4] =
        {
#include "fh_dem_para/FH_Chirp_bb.par"
        };
//-------------同步相关------------//
static float cfBaseBand[FH_CHIRP_BASE_LEN * 4] = { 0 };
static float cfBaseBandBak[FH_CHIRP_BASE_LEN*2] = {0};
#pragma pack(push)//保存对齐状态
#pragma pack(8)
static unsigned int FH_SigPowOld[FH_CHIRP_BASE_LEN]; 			// 信号能量
static unsigned int FH_SigPowNew[FH_CHIRP_BASE_LEN];
static unsigned int FH_SigPowAvg[FH_CHIRP_BASE_LEN];  		// 信号平均能量
static float recv_syn_w[FH_CHIRP_BASE_LEN*2];
static float fRHO[FH_CHIRP_BASE_LEN];
static float recv_syn_rho[SYN_MAX_INDEX];//2048+2048+288*128 = 40k
static float recv_syn_baseband[SYN_MAX_INDEX*2];
static float  fRHO_old[FH_CHIRP_BASE_LEN * 2] = { 0 };
//----------------------------Frequency Demodulation---------------------------------//
static float cp5[128*2];
static float recv_demod_baseband[64][RECV_DOUBLE_NUM] = {0};
static float recv_demod_out[4][RECV_DOUBLE_NUM] = {0};
static float deinterleave_out[4][RECV_DOUBLE_NUM] = {0};
static float dec_in[4][3*RECV_DOUBLE_NUM] = {0};
#pragma pack(pop)//恢复对齐状态

static int64_t FH_SigPowSum = 0;  				   		// 信号能量和
static int syn_status=SYN_STATUS_SEARCH_TRIGGER;
static int buffer_index=0;
//-------------------Viterbi Decode--------------------//
static unsigned char temp_dec_out[3*RECV_CRC_IN_NUM] = {0};
static unsigned char dec_out_msg[RECV_BIT_NUM] = {0};
static unsigned char dec_out_crc16[16] = {0};
static unsigned char dec_out_msg_byte[RECV_BYTE_NUM] = {0};
//-----------------------Whiten------------------------//
static unsigned char recv_whiten_in[RECV_BYTE_NUM] = {0};
static char FH_DecodeChar[RECV_BYTE_NUM+1] = {'\0'};
/* -------------------------------------接口函数声明------------------------------------------ */
static void viterbi419(float 	input[4][2*VITERBI_DEC_IN_HALF], unsigned char 	output[]);
static void euclidean_dis(float pDataSrc[4][2*VITERBI_DEC_IN_HALF], float output[4][VITERBI_DEC_IN_HALF]);
static int FH_Demodulation(char* out);
static float cabsf(float *a);
static void FH_DeModulate_Reset(void);
/*****************************************************************************
 Prototype    : viterbi419
 Description  : 维特比译码
 Input        : input[4][2*VITERBI_DEC_IN_HALF]，output[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void viterbi419(float input[4][2*VITERBI_DEC_IN_HALF], unsigned char output[])
{
    int i;
    int j;
    int l;
    int index;
    int surv 	= 0;

    memset(cnv_surv_dec, 0, sizeof(cnv_surv_dec));
    memset(cnv_surv_state, 0, sizeof(cnv_surv_state));
    memset(weight_final1,	0,	sizeof(weight_final1));
    memset(weight_final2,	0,	sizeof(weight_final2));
    memset(weight,			0,	sizeof(weight));
    memset(cnv_dis_sum_temp,0,	sizeof(cnv_dis_sum_temp));
    memset(cnv_sum,			0,	sizeof(cnv_sum));

    euclidean_dis(input, weight);

    for (i = 0; i <= VITERBI_DEC_IN_HALF - 1; i++)
    {
        memset(dec_out1,0,sizeof(dec_out1));
        memset(dec_out2,0,sizeof(dec_out2));
        for(j = 0;j <= 256 - 1;j++)
        {
            dec_out1[j] 					= 8*Pre_En_Table1[0][j]  + 4*Pre_En_Table1[1][j] + 2*Pre_En_Table1[2][j] + Pre_En_Table1[3][j];
            dec_out2[j] 					= 8*Pre_En_Table2[0][j]  + 4*Pre_En_Table2[1][j] + 2*Pre_En_Table2[2][j] + Pre_En_Table2[3][j];
            weight_final1[j] 				= weight[dec_out1[j]][i] + cnv_sum[Pre_State_Table[0][j]];
            weight_final2[j] 				= weight[dec_out2[j]][i] + cnv_sum[Pre_State_Table[1][j]];
            if(weight_final1[j] < weight_final2[j])
            {
                cnv_dis_sum_temp[j] = weight_final2[j];
                cnv_surv_dec[j][i] = Pre_In_Table[1][j];
                cnv_surv_state[j][i]= Pre_State_Table[1][j];
            }
            else
            {
                cnv_dis_sum_temp[j] = weight_final1[j];
                cnv_surv_dec[j][i] = Pre_In_Table[0][j];
                cnv_surv_state[j][i] = Pre_State_Table[0][j];
            }
        }

        for(l = 0;l <= 256 - 1;l++)
        {
            cnv_sum[l] 						= cnv_dis_sum_temp[l];
        }
    }

    for(index = VITERBI_DEC_IN_HALF - 1;index >= 0;index--)
    {
        output[index] = cnv_surv_dec[surv][index];
        surv = cnv_surv_state[surv][index];
    }
}

/*****************************************************************************
 Prototype    : euclidean_dis
 Description  :
 Input        : input[4][2*VITERBI_DEC_IN_HALF]，output[4][VITERBI_DEC_IN_HALF]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void euclidean_dis(float pDataSrc[4][2*VITERBI_DEC_IN_HALF],
                          float output[4][VITERBI_DEC_IN_HALF])
{
    int i;
    int m;
    int n;

    for (i = 0; i < VITERBI_DEC_IN_HALF; i++)
    {
        for(m = 0;m < 4;m++)
        {
            for(n = 0;n < 4;n++)
            {
                output[4 * m + n][i] = pDataSrc[m][2 * i] * pDataSrc[m][2 * i] + pDataSrc[n][2 * i + 1] * pDataSrc[n][2 * i + 1];
            }
        }
    }
}
/*****************************************************************************
 Prototype    : FH_DeModulate_Init
 Description  :
 Input        : NULL
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
void FH_DeModulate_Init()
{
    memset(recv_syn_rho, 	 	0, 	sizeof(recv_syn_rho));
    memset(recv_syn_baseband, 	0, 	sizeof(recv_syn_baseband));

    gen_w_r2(recv_syn_w, FH_CHIRP_BASE_LEN*2);
    bit_rev(recv_syn_w, FH_CHIRP_BASE_LEN);
}

void FH_DeModulate_Reset()
{
    // 清能量状态缓存
    FH_SigPowSum = 0;
    memset(FH_SigPowOld, 0, sizeof(FH_SigPowOld));
    memset(FH_SigPowNew, 0, sizeof(FH_SigPowNew));
    syn_status						= SYN_STATUS_SEARCH_TRIGGER;
    buffer_index					= 0;
}

/*****************************************************************************
 Prototype    : FH_SyncDecode
 Description  : 跳频Chirp头同步
 Input        : *pInput,*pOutput
 Output       : NULL
 Return Value :
*****************************************************************************/
int FH_SyncDecode(float* src, uint16_t group_num, char* out)
{
    int 			i					= 0;
    int 			j					= 0;
    float  *pOldData 			= cfBaseBand;
    float  *pNewData 			= cfBaseBand+FH_CHIRP_BASE_LEN*2;
    uint16_t adfh_read_index = 0; //跳频接收解调读指针index
    while (adfh_read_index<group_num)
    {
        DSPF_sp_blk_move(cfBaseBandBak, pOldData, FH_CHIRP_BASE_LEN * 2);
        DSPF_sp_blk_move(src + FH_CHIRP_BASE_LEN * 2 * adfh_read_index, pNewData, FH_CHIRP_BASE_LEN * 2);
        adfh_read_index++;
        DSPF_sp_blk_move(pNewData, cfBaseBandBak, FH_CHIRP_BASE_LEN * 2);

        if (adfh_read_index == 1)//第一组数据
        {
            for (i = 0; i < FH_CHIRP_BASE_LEN; i++)
            {
                FH_SigPowNew[i] = (unsigned int)floorf((pNewData[i * 2] * pNewData[i * 2] + pNewData[i * 2 + 1] * pNewData[i * 2 + 1])*Powcoff);
                FH_SigPowSum += FH_SigPowNew[i];
            }
            memcpy(FH_SigPowOld, FH_SigPowNew, FH_CHIRP_BASE_LEN * 4); // 保存旧的信号能量
            continue;
        }
        for (i = 0; i < FH_CHIRP_BASE_LEN; i++)
        {
            // 计算信号能量
            FH_SigPowNew[i] = (unsigned int)floorf((pNewData[i * 2] * pNewData[i * 2] + pNewData[i * 2 + 1] * pNewData[i * 2 + 1])*Powcoff);
            // 计算信号平均能量
            FH_SigPowSum = FH_SigPowSum + FH_SigPowNew[i] - FH_SigPowOld[i];
            FH_SigPowAvg[i] = sqrtf(FH_SigPowSum*1.0f / FH_CHIRP_BASE_LEN);
        }
        memcpy(FH_SigPowOld, FH_SigPowNew, FH_CHIRP_BASE_LEN * 4); // 保存旧的信号能量

        DSPF_sp_cfftr2_dit(cfBaseBand, recv_syn_w, FH_CHIRP_BASE_LEN * 2);
        //bit_rev(cfBaseBand, FH_CHIRP_BASE_LEN*2);
        for (j = 0; j < FH_CHIRP_BASE_LEN * 2; j++)
        {
            cmltf(&fh_chirp[j * 2], &cfBaseBand[j * 2], &cfBaseBand[j * 2]);
        }
        //bit_rev(cfBaseBand, FH_CHIRP_BASE_LEN*2);
        DSPF_sp_icfftr2_dif(cfBaseBand, recv_syn_w, FH_CHIRP_BASE_LEN * 2);
        for (j = 0; j < FH_CHIRP_BASE_LEN; j++)
        {
            fRHO[j] = cabsf(&cfBaseBand[j * 2]) / FH_SigPowAvg[j] / FH_CHIRP_BASE_LEN;

        }
        if (syn_status == SYN_STATUS_SEARCH_TRIGGER)
        {
            DSPF_sp_blk_move(fRHO_old + FH_CHIRP_BASE_LEN, fRHO_old, FH_CHIRP_BASE_LEN);
            DSPF_sp_blk_move(fRHO, fRHO_old + FH_CHIRP_BASE_LEN, FH_CHIRP_BASE_LEN);
        }

        for (j = 0; j < FH_CHIRP_BASE_LEN; j++)
        {
            if (syn_status == SYN_STATUS_SEARCH_TRIGGER)
            {
                if ((FH_SigPowAvg[j]>2) && (fRHO[j] > RHO_TRIGGERGATE))
                {
                    syn_status = SYN_STATUS_BUFFER_DATA;
                    buffer_index = 0;
#ifdef DEBUG_FH
                    {
						LOG1("物理层：Find FH Chirp %3.3f !", fRHO[j]);
					}
#endif
                    recv_syn_rho[buffer_index] = fRHO[j];
                    recv_syn_baseband[buffer_index * 2] = cfBaseBandBak[j * 2];
                    recv_syn_baseband[buffer_index * 2 + 1] = cfBaseBandBak[j * 2 + 1];
                    buffer_index++;
                }
            }
            else
            {
                if (syn_status == SYN_STATUS_BUFFER_DATA)
                {
                    if (buffer_index < SYN_MAX_INDEX)
                    {
                        recv_syn_rho[buffer_index] = fRHO[j];
                        recv_syn_baseband[buffer_index * 2] = cfBaseBandBak[j * 2];
                        recv_syn_baseband[buffer_index * 2 + 1] = cfBaseBandBak[j * 2 + 1];
                        buffer_index++;
                    }
                    else
                    {
                        FH_DeModulate_Reset();
                        return FH_Demodulation(out);
                    }
                }
            }
        }
    }
    FH_DeModulate_Reset();
    return 0;
}

/*****************************************************************************
 Prototype    : FH_Demodulation
 Description  : 跳频解调
 Input        : *pOutput
 Output       : NULL
 Return Value :
*****************************************************************************/
static int FH_Demodulation(char* out)
{
    int 	i 					= 0;
    int 	j 					= 0;
    short 	recv_crc16_true 	= 0;
    int 	index_rho_peak		= 0;
    float 	rho_peak			= 0;
    short 	recv_crc16_out;
    float *recv_fh_baseband;

    unsigned char *addr0;
    unsigned char *addr1;
    unsigned char *addr2;

    memset(recv_demod_baseband,	0, sizeof(recv_demod_baseband));
    memset(deinterleave_out,	0, sizeof(deinterleave_out));
    memset(dec_in,				0, sizeof(dec_in));
    memset(dec_out_msg,			0, sizeof(dec_out_msg));
    memset(recv_whiten_in,		0, sizeof(recv_whiten_in));
    memset(dec_out_msg_byte,	0, sizeof(dec_out_msg_byte));

    for(j = 0;j < FH_CHIRP_BASE_LEN;j++)
    {
        if(rho_peak < recv_syn_rho[j])
        {
            rho_peak 		= recv_syn_rho[j];
            index_rho_peak 	= j;
        }
    }

    recv_fh_baseband = &recv_syn_baseband[(FH_CHIRP_BASE_LEN + index_rho_peak)*2];//跨过chirp和保护间隔
    for(i = 0;i < RECV_DOUBLE_NUM;i++)
    {
        DSPF_sp_blk_move((recv_fh_baseband+i*128*2),cp5,128*2);
        DSPF_sp_cfftr2_dit(cp5, recv_syn_w, 128);
        bit_rev(cp5, 128);
        for(j = 0;j < 64;j++)
        {
            recv_demod_baseband[j][i] = cp5[j*4] * cp5[j*4] + cp5[j*4+1] * cp5[j*4+1];
        }
    }
    for(i = 0;i < RECV_DOUBLE_NUM;i++)
    {
        for(j = 0;j < 4;j++)
        {
            if(Mod_Hop_Map[i] < 64)
            {
                recv_demod_out[j][i] = recv_demod_baseband[Mod_Hop_Map[i] + j][i];
            }
            else
            {
                recv_demod_out[j][i] = recv_demod_baseband[Mod_Hop_Map[i] - 64 + j][i];
            }
        }

    }
//---------------------------Deinterleave------------------------------//
    for(i = 0;i < RECV_DOUBLE_NUM;i++)
    {
        for(j = 0;j < 4;j++)
        {
            deinterleave_out[j][Interleave_Seq[i]] = recv_demod_out[j][i];
        }
    }

//----------------------------Decode-----------------------------------//
    for(i = 0;i < RECV_DOUBLE_NUM;i++)
    {
        for(j = 0;j < 4;j++)
        {
            dec_in[j][i] 		= deinterleave_out[j][i];
            dec_in[j][i+RECV_DOUBLE_NUM] 	= deinterleave_out[j][i];
            dec_in[j][i+2*RECV_DOUBLE_NUM] 	= deinterleave_out[j][i];
        }
    }

    viterbi419(dec_in, temp_dec_out);

    for (i = 0; i < RECV_BIT_NUM; i++)
    {
        dec_out_msg[i] = temp_dec_out[RECV_CRC_IN_NUM + i];
    }

    addr0 = dec_out_msg;
    addr1 = dec_out_msg_byte;

    j = 0;

    for(i = 0;i < RECV_BYTE_NUM;i++)
    {
        for(j = 0;j < 8;j++)
        {
            *addr1 |= ((*addr0++) << j);
        }
        *addr1++;
    }

    addr1 = dec_out_msg_byte;


    for(i = 0;i < 16;i++)
    {
        dec_out_crc16[i] = temp_dec_out[RECV_BIT_NUM + RECV_CRC_IN_NUM + i];
    }


    for(i = 0;i < 16;i++)
    {
        recv_crc16_true |= dec_out_crc16[i] << i;
    }


//---------------------------------------------------------------------//
    recv_crc16_out = Endecode_Crc16((char*)dec_out_msg_byte, RECV_BYTE_NUM);

    addr0 = dec_out_msg;
    addr2 = recv_whiten_in;

    j = 8;
    for(i = 0;i < RECV_BIT_NUM;i++)
    {
        *addr2 |=  ((*addr0++) << --j);
        if(j == 0)
        {
            j = 8;
            *addr2++;
        }
    }

    if(recv_crc16_true == recv_crc16_out)
    {
        FH_Whiten(recv_whiten_in, (unsigned char*)out);
        *(out+64) = '\0';
#ifdef DEBUG_FH
        {
			LOG2("物理层：Chirp Peak %3.3f !解码结果(字符显示)：%s", rho_peak, out);
		}
#endif
        return 1;
    }
#ifdef DEBUG_FH
    {
		LOG1("物理层：Chirp Peak %3.3f !校验错误", rho_peak);
	}
#endif
    return 0;
}
static float cabsf(float *a)
{
    float ret;
    ret = sqrtf(a[0] * a[0] + a[1] * a[1]);
    return ret;
}