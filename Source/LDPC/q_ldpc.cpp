/*
 * q_ldpc.c
 *
 *  Created on: 2021-7-29
 *      Author: ioa
 */
#include "stdafx.h"
#include <string.h>
#include <math.h>
#include "LDPC/q_ldpc.h"
#include "EnDecode/endecode.h"
#include "global.h"
#include "Net/enet_shell.h"
////////////////// definings //////////////
#ifdef  LDPC_MOD
#define M_ENC_RTS 256
#define M_ENC_DATA 64
#define LEN_IN_RTS 11
#define LEN_OUT_RTS 60

#define GROUP_BASE 120//交织表和度分布存储基数，分布方式是120组、240组、360组...

#define LEN_BUF (GROUP_BASE*63*64)

#define NM_DATA 7
#define NM_RTS 13
#define INF 32767
#define L_CHAN_BIT 10
#define scaler 0.8f

//译码临时数组
typedef struct ty_LDPC_DECODE_TEMP
{
	short ff1[LEN_BUF];
	short ff2[LEN_BUF];
	short fb1[LEN_BUF];
	short fb2[LEN_BUF];
} LDPC_DECODE_TEMP;
//译码相关
typedef struct ty_LDPC_DECODE_PAR
{
	short  x_copy[M_ENC_RTS];
	short  y_copy[M_ENC_RTS];
	short  x_nm[NM_RTS];
	short  y_nm[NM_RTS];
	unsigned char  x_idx[NM_RTS];
	unsigned char  y_idx[NM_RTS];
	short l_spa[M_ENC_RTS];
	unsigned char gf_prod_vector[M_ENC_RTS];
} LDPC_DECODE_PAR;//不足2k

static short*  x_copy = {0};
static short*  y_copy = {0};

static short*  x_nm = {0};
static short*  y_nm = {0};

static unsigned char*  x_idx = {0};
static unsigned char*  y_idx = {0};

static short* l_spa = {0};
static unsigned char* gf_prod_vector = {0};
static const unsigned char whiten_DATA[GROUP_BASE * 63]=
{
	#include "whiten_order64.txt"
};
static const unsigned char whiten_RTS[LEN_OUT_RTS]={
#include "whiten_RTS.txt"
};
static const unsigned int intl_idx[63]={0,120,360,720,1200,1800,2520,3360,4320,5400,6600,7920,9360,10920,12600,14400,16320,18360,20520,22800,25200,27720,30360,33120,36000,39000,42120,45360,48720,52200,55800,59520,63360,67320,71400,75600,79920,84360,88920,93600,98400,103320,108360,113520,118800,124200,129720,135360,141120,147000,153000,159120,165360,171720,178200,184800,191520,198360,205320,212400,219600,226920,234360};
static uint16_t data_int[GROUP_BASE*63] = {0};
static uint16_t control_int[60] = {31,51,16,23,44,10,3,53,34,28,21,41,6,14,58,50,25,43,8,19,36,59,49,0,29,22,40,7,52,1,32,18,38,24,11,46,2,17,30,54,37,48,4,12,20,55,42,26,35,9,15,57,47,27,39,33,5,56,13,45};
static uint16_t rep_len_DATA[3 * 63]=
{
	#include "degree_dist.txt"
};
////////////////// little vectors//////////////

static unsigned char  rep_d_DATA[]= {2,3,6};
static unsigned char  rep_d_RTS []= {2,3,13};
static uint16_t  rep_len_RTS[]=  {3, 5, 3};
static const unsigned char gf_64_log[]={0,1,6,2,12,7,26,3,32,13,35,8,48,27,18,4,24,33,16,14,52,36,54,9,45,49,38,28,41,19,56,5,62,25,11,34,31,17,47,15,23,53,51,37,44,55,40,10,61,46,30,50,22,39,43,29,60,42,21,20,59,57,58};
static const unsigned char gf_64_exp[]={1,2,4,8,16,32,3,6,12,24,48,35,5,10,20,40,19,38,15,30,60,59,53,41,17,34,7,14,28,56,51,37,9,18,36,11,22,44,27,54,47,29,58,55,45,25,50,39,13,26,52,43,21,42,23,46,31,62,63,61,57,49,33};
static const unsigned char gf_256_log[]={
0,1,25,2,50,26,198,3,223,51,238,27,104,199,75,4,100,224,14,52,141,239,129,28,193,105,248,200,8,76,113,5,138,101,47,225,36,15,33,53,147,142,218,240,18,130,69,29,181,194,125,
106,39,249,185,201,154,9,120,77,228,114,166,6,191,139,98,102,221,48,253,226,152,37,179,16,145,34,136,54,208,148,206,143,150,219,189,241,210,19,92,131,56,70,64,30,66,182,163,195,72,126,
110,107,58,40,84,250,133,186,61,202,94,155,159,10,21,121,43,78,212,229,172,115,243,167,87,7,112,192,247,140,128,99,13,103,74,222,237,49,197,254,24,227,165,153,119,38,184,180,124,17,68,
146,217,35,32,137,46,55,63,209,91,149,188,207,205,144,135,151,178,220,252,190,97,242,86,211,171,20,42,93,158,132,60,57,83,71,109,65,162,31,45,67,216,183,123,164,118,196,23,73,236,127,
12,111,246,108,161,59,82,41,157,85,170,251,96,134,177,187,204,62,90,203,89,95,176,156,169,160,81,11,245,22,235,122,117,44,215,79,174,213,233,230,231,173,232,116,214,244,234,168,80,88,175
};

static const unsigned char gf_256_exp[]={
1,2,4,8,16,32,64,128,29,58,116,232,205,135,19,38,76,152,45,90,180,117,234,201,143,3,6,12,24,48,96,192,157,39,78,156,37,74,148,53,106,212,181,119,238,193,159,35,70,140,5,
10,20,40,80,160,93,186,105,210,185,111,222,161,95,190,97,194,153,47,94,188,101,202,137,15,30,60,120,240,253,231,211,187,107,214,177,127,254,225,223,163,91,182,113,226,217,175,67,134,17,34,
68,136,13,26,52,104,208,189,103,206,129,31,62,124,248,237,199,147,59,118,236,197,151,51,102,204,133,23,46,92,184,109,218,169,79,158,33,66,132,21,42,84,168,77,154,41,82,164,85,170,73,
146,57,114,228,213,183,115,230,209,191,99,198,145,63,126,252,229,215,179,123,246,241,255,227,219,171,75,150,49,98,196,149,55,110,220,165,87,174,65,130,25,50,100,200,141,7,14,28,56,112,224,
221,167,83,166,81,162,89,178,121,242,249,239,195,155,43,86,172,69,138,9,18,36,72,144,61,122,244,245,247,243,251,235,203,139,11,22,44,88,176,125,250,233,207,131,27,54,108,216,173,71,142,
};

////////////////// large vectors //////////////
static short l_app[LEN_BUF]={0};
static short l_acc_out[LEN_BUF]={0};
static short* ff1 = {0};
static short* ff2 = {0};
static short* fb1 = {0};
static short* fb2 = {0};
static const unsigned char error_num_table[]={0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4,1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,4,5,5,6,5,6,6,7,5,6,6,7,6,7,7,8};
static unsigned char ldpc_bitstream_buf0[GROUP_BASE*63]; // 数据流缓存，支持ldpc 1/3 码率最大63帧
// 对于RST帧，输入为8bit* 11， 输出为8bit* 60，级联24选2调制，发射到12个120子载波，1帧
#pragma pack(push)//保存对齐状态
#pragma pack(8)
unsigned char bank_3_space[LEN_BUF * 4 * 2];//在SD_RAM_BANK3开辟出一块1M的片内空间，类似SDRAM_BANK2的使用
#pragma pack(pop)//恢复对齐状态
/* ---------------------------- 局部函数声明 --------------------------------*/
static unsigned char gf_prod_scaler(unsigned char a, unsigned char b, int order);
static void gf_prod(unsigned char a, unsigned char b[], int order);

static void spa_rep_node(int degree, short l_in[], short l_out[], unsigned char * decision, int order);
static void outer_MAX(short l_in[], short l_out[], unsigned char p_decision[], int order, unsigned char num_rep_degree, unsigned char rep_d[], uint16_t rep_len[]);
static void sub_idx0(short in[], int order);
static void siso_acc(short l_chan[], short l_app[],  short l_ext[],int order, int len);
static void ems( short x[], short y[],short z[],int order );
static float get_ber(short x1_soft[],unsigned char x2[],int order,int len);
static int vector_max(short x[], int nx);// 返回矢量中最大值位置
/* ---------------------------- 编码 --------------------------------*/
static unsigned char gf_prod_scaler(unsigned char a, unsigned char b, int order)
{
	int j;
	const unsigned char *gf_log;
	const unsigned char *gf_exp;

	if (a==0 || b==0)
	{
		return 0;
	}
	else
	{
		if(order==64)
		{
			gf_log=gf_64_log;
			gf_exp=gf_64_exp;
		}
		else//256
		{
			gf_log = gf_256_log;
			gf_exp = gf_256_exp;
		}
		j=gf_log[b-1]+gf_log[a-1];
		if(j>=order-1) j -= order-1;
		return gf_exp[j];
	}
}

static void gf_prod(unsigned char a, unsigned char b[], int order)
{
	int i;
	int j;
	const unsigned char *gf_log;
	const unsigned char *gf_exp;

	if (a==0)
	{
		memset(b,0,sizeof(unsigned char)*order);
	}
	else
	{
		b[0]=0;
		if(order==64)
		{
			gf_log=gf_64_log;
			gf_exp=gf_64_exp;
		}
		else//256
		{
			gf_log = gf_256_log;
			gf_exp = gf_256_exp;
		}
		for(i=1;i<order;i++)
		{
			j=gf_log[i-1]+gf_log[a-1];
			if(j>=order-1) j -= order-1;
			b[i]=gf_exp[j];
		}
	}
}
unsigned int ldpc_encoder(char *pDataSrc, unsigned int nInLen, unsigned int nBitValidNum, unsigned char syn_num, unsigned char *pDataDes, unsigned char is_DATA, unsigned int phy_mfsk_symbol_bb)
{
	unsigned int i;
	int j;
	int d;
	int k;

	unsigned int len;
	int order;
	int num_rep_degree=3;
	uint16_t *interleaver;
	const unsigned char *whiten;
	unsigned char *rep_d = {0};
	uint16_t *rep_len = {0};
	unsigned char *msg_repeat=(unsigned char*)&l_app[0];

	//add by zly 2017-02-28
	short crc16 = 0;
	unsigned int valid_byte_num = 0;
	unsigned int bit_num = 0;
	unsigned int byte_num = 0;
	valid_byte_num = (unsigned long)ceilf(nBitValidNum / 8.0f); // 有效字节数
	unsigned char *bit_6 = pDataDes;
	char *src_8 = pDataSrc;
	unsigned char temp,temp1,temp2;
	unsigned char frame_num;
	FILE* file;
	errno_t err;
	if (is_DATA==1)
	{
		len=syn_num * CARRIER_NUM(phy_mfsk_symbol_bb)/LDPC_DATA_NUM ;//每帧组数
		frame_num = (unsigned char)ceilf( (nBitValidNum + 16.0f) /(len*2)); // 发射帧数，码率是1/3，每组发送6bit，因而每帧发送的原始bit数为len*2
		len = len * frame_num;//数据包总组数group_num
		bit_num = len*2 - 16;//码率是1/3，每组发送6bit，数据包发送的原始bit数为len*2
		byte_num = (unsigned int)ceilf( bit_num / 8.0f );

		order=M_ENC_DATA;
		rep_d=rep_d_DATA;
		if((len%GROUP_BASE)!=0)
		{
			#ifdef DEBUG_MFSK
			LOG("物理层(mtmfsk)：不是基础组的整数倍!");
			#endif
			return 0;
		}
		rep_len=&rep_len_DATA[(len/GROUP_BASE-1)*3];//6bit组度分布放置方式是组数分别为120、240、360....
		interleaver=data_int;
		whiten = whiten_DATA;
		err = fopen_s(&file, "ldpcintl.txt", "rb");
		if (err != 0)
		{
			#ifdef DEBUG_MFSK
			LOG("物理层(mtmfsk)：打开LDPC交织表失败!");
			#endif
			return 0;
		}
		else
		{
			fseek(file, intl_idx[len/GROUP_BASE-1]*2, 0);//6bit组交织表放置方式是组数分别为120、240、360....
			fread(interleaver, len*2, 1, file);
			fclose(file);
		}
		bit_num = valid_byte_num * 8 - nBitValidNum; // 无效比特个数
		pDataSrc[valid_byte_num - 1]  &= (0xff >> bit_num); // 无效比特清0
		for(i=valid_byte_num;i<byte_num;i++)
		{
			pDataSrc[i] = 0;//填充字节到byte_num字节
		}

	}
	else
	{
		len=LEN_OUT_RTS;
		order=M_ENC_RTS;
		rep_d=rep_d_RTS;
		rep_len=rep_len_RTS;
		interleaver=control_int;
		whiten = whiten_RTS;
		// 输入非法
		if(nInLen > 9 || nInLen < 2)//信源长度是限定11*8bit-2byteCRC
			return 0;

		bit_num = valid_byte_num * 8 - nBitValidNum; // 无效比特个数
		pDataSrc[valid_byte_num - 1]  &= (0xff >> bit_num); // 无效比特清0
		for(i=valid_byte_num;i<9;i++)
		{
			pDataSrc[i] = 0;//填充字节到9字节
		}
		byte_num = 9;
	}
	// 生成CRC校验
	crc16 = Endecode_Crc16(pDataSrc, byte_num); // crc16的bit流顺序为从低字节的低bit开始往高排列
	pDataSrc[byte_num] = (crc16 & 0x00ff);
	pDataSrc[byte_num+1] = ((crc16 >> 8) & 0x00ff);
	if(is_DATA==1)
	{
		//字节转换成6bit一组
		for(i=0;i<(len*2/24);i++)//len*6/3/4/6，数据包总组数*每组2个原始信息比特，每次组合6*4bit一次处理24bit，码率1/3
		{
			temp = *src_8++;
			temp1 = *src_8++;
			temp2 = *src_8++;
			*bit_6++ = (temp & 0x3f);
			*bit_6++ = ((temp & 0xC0)>>6)|(temp1&0x0f)<<2;
			*bit_6++ = ((temp1 & 0xf0)>>4)|(temp2&0x03)<<4;
			*bit_6++ = (temp2&0xfC)>>2;
		}
		bit_6 = pDataDes;
		i=0;// repeat
		for(d=0;d<num_rep_degree;d++)
		{
			for(j=0;j<rep_len[d];j++)
			{
				for (k=0;k<rep_d[d];k++) msg_repeat[i++]=*bit_6;
				bit_6++;
			}
		}
	}
	else
	{
		src_8 = pDataSrc;
		i=0;// repeat
		for(d=0;d<num_rep_degree;d++)
		{
			for(j=0;j<rep_len[d];j++)
			{
				for (k=0;k<rep_d[d];k++) msg_repeat[i++]=*src_8;
				src_8++;
			}
		}
	}

	// whiten
	for(i=0;i<len;i++) msg_repeat[i] = gf_prod_scaler(msg_repeat[i],whiten[i],order);
	// interleaver
	for(i=0;i<len;i++) pDataDes[i] = msg_repeat[interleaver[i]];
	// acc
	for(i=1;i<len;i++) pDataDes[i] ^= pDataDes[i-1];
	return len;
}

/* ---------------------------- 译码 --------------------------------*/
static void spa_rep_node(int degree, short l_in[], short l_out[], unsigned char * decision, int order)
{
	int i;
	int j;
	memset(l_spa,0,M_ENC_RTS*2);
	for(i=0;i<order;i++)
	{
		if(degree==2)
		{
			l_spa[i]=l_in[i]+l_in[order+i];
			l_out[i]=l_in[order+i];
			l_out[order+i]=l_in[i];
		}
		else if(degree==3)
		{
			l_spa[i]=l_in[i]+l_in[order+i]+l_in[order*2+i];
			l_out[i]  =l_spa[i]-l_in[i];
			l_out[order*1+i]=l_spa[i]-l_in[order*1+i];
			l_out[order*2+i]=l_spa[i]-l_in[order*2+i];
		}
		else
		{
			l_spa[i]=l_in[i];
			for(j=1;j<degree;j++)	l_spa[i]+=l_in[order*j+i];
			for(j=0;j<degree;j++)	l_out[order*j+i]  =l_spa[i]-l_in[order*j+i];
		}
	}
	*decision=vector_max(l_spa,order);
}

static void outer_MAX(short l_in[], short l_out[], unsigned char p_decision[], int order, unsigned char num_rep_degree, unsigned char rep_d[], uint16_t rep_len[])
{
	int d;
	int i;
	int i_msg;
	int j;

	i=0;i_msg=0;
	for(d=0;d<num_rep_degree;d++)
	{
		for(j=0;j<rep_len[d];j++)
		{
			spa_rep_node(rep_d[d], &l_in[i*order], &l_out[i*order], &p_decision[i_msg], order);
			i+=rep_d[d];
			i_msg++;
		}
	}

}

unsigned int ldpc_decoder(short l_chan[], unsigned char decision[], unsigned char frame_num, unsigned char syn_num, unsigned char ite_num, unsigned char is_DATA, unsigned int phy_mfsk_symbol_bb)
{
	LDPC_DECODE_PAR * ldpc_decode_par = {0};
	ldpc_decode_par = ((LDPC_DECODE_PAR *)data_a_space);
	x_copy = ldpc_decode_par->x_copy;
	y_copy = ldpc_decode_par->y_copy;

	x_nm = ldpc_decode_par->x_nm;
	y_nm = ldpc_decode_par->y_nm;

	x_idx = ldpc_decode_par->x_idx;
	y_idx = ldpc_decode_par->y_idx;

	l_spa = ldpc_decode_par->l_spa;
	gf_prod_vector = ldpc_decode_par->gf_prod_vector;
	ff1 = ((LDPC_DECODE_TEMP *)bank_3_space)->ff1;
	ff2 = ((LDPC_DECODE_TEMP *)bank_3_space)->ff2;
	fb1 = ((LDPC_DECODE_TEMP *)bank_3_space)->fb1;
	fb2 = ((LDPC_DECODE_TEMP *)bank_3_space)->fb2;
	int i;
	int j;
	int len;
	int len_in;
	int order;
	int num_rep_degree=3;
	unsigned char *rep_d;
	uint16_t *rep_len;
	uint16_t *interleaver;
	const unsigned char *whiten;
	short *l_acc_out_itvd;
	short *l_acc_out_itvd_dewhited;
	short max_l_chan;
	short max_bit_l_chan;
	short *l_rep_out;
	short *l_rep_out_whitened;
	unsigned int byte_num;

	short crc16_0;
	short crc16_1;
	unsigned char *dec_6 = ldpc_bitstream_buf0;
	unsigned char *dec_8 = decision;
	unsigned char temp,temp1,temp2,temp3;

	FILE* file;
	errno_t err;

	if (is_DATA==1)
	{
		len = CARRIER_NUM(phy_mfsk_symbol_bb)/LDPC_DATA_NUM*syn_num*frame_num;//数据包总组数
		len_in= len/3;//码率1/3，有效信息总组数
		order=M_ENC_DATA;
		rep_d=rep_d_DATA;
		if((len%GROUP_BASE)!=0)
		{
			#ifdef DEBUG_MFSK
			LOG("物理层(mtmfsk)：不是基础组的整数倍!");
			#endif
			return 0;
		}
		rep_len=&rep_len_DATA[(len/GROUP_BASE-1)*3];//6bit组度分布放置方式是组数分别为120、240、360....
		interleaver=data_int;
		whiten = whiten_DATA;
		if(ite_num == 0)
		{
			err = fopen_s(&file, "ldpcintl.txt", "rb");
			if (err != 0)
			{
				#ifdef DEBUG_MFSK
				LOG("物理层(mtmfsk)：打开LDPC交织表失败!");
				#endif
				return 0;
			}
			else
			{
				fseek(file, intl_idx[len/GROUP_BASE-1]*2, 0);//6bit组交织表放置方式是组数分别为120、240、360....
				fread(interleaver, len*2, 1, file);
				fclose(file);
			}
		}
	}
	else
	{
		len=LEN_OUT_RTS;
		len_in=LEN_IN_RTS;
		order=M_ENC_RTS;
		rep_d=rep_d_RTS;
		rep_len=rep_len_RTS;
		interleaver=control_int;
		whiten = whiten_RTS;
	}
	if(ite_num == 0)
	{
		max_l_chan=l_chan[vector_max(l_chan, order*len)];//求最大值量化位数

		for(max_bit_l_chan=0;max_bit_l_chan<16;max_bit_l_chan++)
		{
			if(max_l_chan==0) break;
			max_l_chan>>=1;
		}

		if(L_CHAN_BIT-max_bit_l_chan>0) //移位，保证最大值量化位数为L_CHAN_BIT
		{
			for(i=0;i<order*len;i++) l_chan[i] <<= (L_CHAN_BIT-max_bit_l_chan);
		}
		else if(L_CHAN_BIT-max_bit_l_chan<0)
		{
			for(i=0;i<order*len;i++) l_chan[i] >>= (-L_CHAN_BIT+max_bit_l_chan); // 前提：l_chan恒大于零
		}

		memset(l_app,0,sizeof(short)*order*len);

	}
	// 累加器译码
	siso_acc(&l_chan[0],&l_app[0],l_acc_out,order,len);

	// 解交织
	l_acc_out_itvd=&ff2[0];
	for(i=0;i<len;i++)
	{
		memcpy(&l_acc_out_itvd[interleaver[i]*order], &l_acc_out[i*order],
			sizeof(short)*order);
	}

	//解白化
	l_acc_out_itvd_dewhited=&ff1[0];
	for(i=0;i<len;i++) //P_dewhite
	{
		gf_prod((unsigned char)whiten[i],gf_prod_vector,order);
		for(j=0;j<order;j++)
		{
			l_acc_out_itvd_dewhited[i*order+j]=l_acc_out_itvd[i*order+gf_prod_vector[j]];
		}
	}

	// 重复器译码
	l_rep_out=&ff2[0];
	memset(l_rep_out,0,sizeof(short)*order*len);

	if(is_DATA==1)
	{
		outer_MAX(l_acc_out_itvd_dewhited,l_rep_out,ldpc_bitstream_buf0, order, num_rep_degree, rep_d, rep_len);
		for(i=0;i<len*2/24;i++)//数据包总组数len，每组有效信息bit2，一次处理24bit
		{
			temp = *dec_6++;
			temp1 = *dec_6++;
			temp2 = *dec_6++;
			temp3 = *dec_6++;
			*dec_8++ = (temp & 0x3f)|(temp1&0x03)<<6;
			*dec_8++ = ((temp1 & 0x3C)>>2)|(temp2&0x0f)<<4;
			*dec_8++ = ((temp2 & 0x30)>>4)|(temp3&0x3f)<<2;
		}
		byte_num = len*2/8 - 2;//原始信息字节数，去掉CRC
	}
	else
	{
		outer_MAX(l_acc_out_itvd_dewhited,l_rep_out,decision, order, num_rep_degree, rep_d, rep_len);
		byte_num = len_in - 2;
	}
	crc16_0 = Endecode_Crc16((char *)decision, byte_num);
	crc16_1 = decision[byte_num+1];
	crc16_1 = crc16_1 << 8 | (unsigned char)decision[byte_num];
	if(crc16_0 == crc16_1)
	{
		ldpc_encoder((char *)decision, byte_num, 8 * byte_num, syn_num, ldpc_bitstream_buf0, is_DATA, phy_mfsk_symbol_bb);
		if(is_DATA==1)
		{
			#ifdef DEBUG_MFSK
			LOG2("物理层(mtmfsk)：MT_MFSK数据包校验正确,迭代次数%02d,调制硬判决误比特率%.4f!",ite_num,get_ber(l_chan,ldpc_bitstream_buf0,order,len));
			#endif
		}
		else
		{
			#ifdef DEBUG_MFSK
			LOG2("物理层(mtmfsk)：MT_MFSK控制帧校验正确,迭代次数%02d,调制硬判决误比特率%.4f!",ite_num,get_ber(l_chan,ldpc_bitstream_buf0,order,len));
			#endif
		}
		return (byte_num+2);
	}
	else
	{
		// 白化
		l_rep_out_whitened=&ff1[0];
		for(i=0;i<len;i++)
		{
			gf_prod((unsigned char)whiten[i],gf_prod_vector,order);
			for(j=0;j<order;j++)
			{
				l_rep_out_whitened[i*order+gf_prod_vector[j]]=l_rep_out[i*order+j];
			}
		}
		// 交织
		for(i=0;i<len;i++)
		{
			memcpy( &l_app[i*order], &l_rep_out_whitened[interleaver[i]*order],
				sizeof(short)*order);
		}

		for (j=0;j<order*len;j++)// scaler
		{
			l_app[j]=  (short)floorf((float)l_app[j]*scaler);
		}
		return 0;
	}
}

static void sub_idx0(short in[], int order)
{
	int j;
	for (j=1;j<order;j++)
	{
		in[j]-=in[0];
	}
	in[0]=0;
}

static void siso_acc(short l_chan[], short l_app[], short l_ext[],  int order, int len)
{
	int i,j;
	short tmp;
	// ff2(:,1) = L_app(:,1)+L_chan(:,1);
	// ff2(:,1)=ff2(:,1)-ff2(1,1);
	tmp=l_app[0]+l_chan[0];
	ff2[0]=0;
	for (j=1;j<order;j++)
	{
		ff2[j]=l_app[j]+l_chan[j];
		ff2[j]-=tmp;
	}

	// feed forward
	for (i=1; i<len; i++)
	{
		ems( &ff2[(i-1)*order], &l_app[i*order],&ff1[i*order],order);
		sub_idx0(&ff1[i*order],order);
		ff2[i*order]=0;
		tmp=ff1[i*order]+l_chan[i*order];
		for (j=1;j<order;j++)
		{
			ff2[i*order+j]=ff1[i*order+j]+l_chan[i*order+j];
			ff2[i*order+j]-=tmp;
		}
	}
	// feed back
	memcpy(&fb2[(len-1)*order],&l_chan[(len-1)*order],order*sizeof(short));
	sub_idx0(&fb2[(len-1)*order],order);
	for (i=len-2; i>=0; i--)
	{
		ems( &fb2[(i+1)*order], &l_app[(i+1)*order],&fb1[i*order],order);
		sub_idx0(&fb1[i*order],order);
		fb2[i*order]=0;
		tmp=fb1[i*order]+l_chan[i*order];
		for (j=1;j<order;j++)
		{
			fb2[i*order+j]=fb1[i*order+j]+l_chan[i*order+j];
			fb2[i*order+j]-=tmp;
		}
	}
	// together
	l_ext[0]=0;
	tmp=fb1[0]+l_chan[0];
	for (j=1;j<order;j++)
	{
		l_ext[j]=fb1[j]+l_chan[j];
		l_ext[j]-=tmp;
	}

	for (i=1; i<len; i++)
	{
		ems( &ff2[(i-1)*order], &fb2[i*order],&l_ext[i*order],order);
		sub_idx0(&l_ext[i*order],order);
	}
}

static void ems(short x[], short y[],short z[],int order)// EMS算法，对两个变量求和，在LLR域上计算
{
	int i;
	int j;
	int k;
	short tmp;
	int NM;
	if (order == M_ENC_RTS) NM = NM_RTS;
	if (order == M_ENC_DATA) NM = NM_DATA;
	memcpy (x_copy, x, order*sizeof(short));
	memcpy (y_copy, y, order*sizeof(short));
	for(i=0;i<NM;i++)// 找前NM个最大值位置，及最大值
	{
		x_idx[i]=vector_max(x_copy,order);
		x_nm[i]=x_copy[x_idx[i]];
		x_copy[x_idx[i]]=-INF;
		y_idx[i]=vector_max(y_copy,order);
		y_nm[i]=y_copy[y_idx[i]];
		y_copy[y_idx[i]]=-INF;
//		printf("No. %d, x[%d]=%.0f\n",i,x_idx[i],x_nm[i]);
//		printf("No. %d, y[%d]=%.0f\n",i,y_idx[i],y_nm[i]);
	}
	j=x_idx[0];
	for(i=0;i<order;i++)// Conf(q,1)
	{
		k=j^i;
		z[k]=x_nm[0]+y[i];
	}

	j=y_idx[0];
	for(i=0;i<order;i++)// Conf(q,1)
	{
		k=i^j;
		tmp=y_nm[0]+x[i];
		z[k]=tmp>z[k]?tmp:z[k];
	}
	for(i=0;i<NM;i++)// Conf(NM,2)
	{
		for(j=0;j<NM;j++)
		{
			k=y_idx[j]^x_idx[i];
			tmp=x_nm[i]+y_nm[j];
			z[k]=tmp>z[k]?tmp:z[k];
		}
	}
}
static float get_ber(short x1_soft[],unsigned char x2[],int order,int len)
{
	int i;
	int err=0;
	unsigned char x1;

	for (i=0;i<len;i++)
	{
		x1=vector_max(x1_soft+i*order,order);
		err+=error_num_table[x1^x2[i]];
	}
	if(order==256)
	{
		return err/(float)(8*len);
	}
	else
	{
		return err/(float)(6*len);
	}

}
static int vector_max(short x[], int nx)// 返回矢量中最大值位置
{
	int i;
	int j=0;
	short  z0=x[0];

	for (i=1;i<nx;i++)
	{
		if (z0<x[i])
		{
			z0=x[i];
			j=i;
		}
	}
	return j;
}
#endif

