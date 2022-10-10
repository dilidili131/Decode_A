/*
 * mpsk_dem.c
 *
 *  Created on: 2021-9-1
 *      Author: ioa
 */
#include "stdafx.h"
#include "dsplib.h"
#include "string.h"
#include "math.h"
#include "global.h"
#include "MPSK/mpsk_dem.h"
#include "EnDecode/endecode.h"
#include "Net/enet_shell.h"
/* --------------------------------------- 宏定义 -------------------------------------------- */
//多普勒补偿
#define FARROW_L							4     												// polynomial degree
#define FARROW_K							8     												// FIR columns
#define FARROW_FILTERDELAY  				(FARROW_K/2)										// the farrow resampling has no delay
#define FARROW_FRAME_LEN					(2136*2)
#define VALID_FLAG 0x1234

//turboeq
#define TURBO_EQ_ITER 							1
#define TURBO_DEC_ITER 							2

//equalizer
#define Fs 								2
#define chnum 							4
#define SAMPLE_SIZE 					(Fs*chnum)
#define PLL_k1 							7e-3
#define PLL_k2 							2.5e-5
#define MAX_LEN_W  						2000
#define MAX_SPARSE_LEN_W 				250
#define SYMBOL_NUM	 					2136
#define CPOW(real,imag)					((real*real)+(imag*imag))
#define CEIL(a,b)						((a%b!=0)?(a/b+1):(a/b))

//turbocode
#define K 								1936
#define Ms 								16
#define Mo 								4
#define K1  							121
#define K2  							16
/* --------------------------------------结构体定义------------------------------------------- */
typedef struct
{
    float re;
    float im;
} complex_float;

//多普勒补偿
typedef struct tag_doppler_carrier_struct
{
    float carrier[SYMBOL_NUM * Fs*2];
    int32_t flag_initialized;
    float doppler;
}doppler_carrier_struct;

//equalizer
typedef struct
{
    int 	L1;
    int 	L2;
    int 	LEN_W;
    int 	symbnum;
    int 	train_len;
    int 	weight_idx;
    float 	mu;
} paradef;
typedef struct
{
    float W[MAX_LEN_W*2];
    float U[MAX_LEN_W*2];
    float U1[MAX_LEN_W*2];
    float t2[MAX_LEN_W*2];
} equalizer_buf;

//turbocode
typedef struct
{
    float alpha	[Ms*K];
    float beta	[Ms*K];
    float gamma	[Mo*K];
    float temp  [K1*2*Ms];
    float temp1 [K1*2*Ms];
    float temp2 [K1*2*Ms];
    float temp3 [K1*2*Ms];
    float i1_in [2*K];
    float i2_in [2*K];
    float i1	[2*K];
    float i2	[2*K];
    float buf1	[2*K];
    float buf2	[2*K];
    float buf3	[2*K];
}TURBODEC_STRUCT;//(16+16+4)*1936+121*2*16*4+2*1936*7=112288float
/* -------------------------------------全局变量------------------------------------------ */
extern float mpsk_data1_buf[];
extern float mpsk_data2_buf[];
extern unsigned char data_a_space[];//farrow_fiter_float
/* -------------------------------------全局函数声明------------------------------------------ */;
extern void mfsk_mat_trans(const float *x, const int rows, const int cols, float *y);//转换前列是奇数
extern void cmltf(float *a, float *b, float *out);
/* -------------------------------------局部变量------------------------------------------ */
static int bad_framenum = 0;
static float in_float_sdram[2 * Fs * chnum*SYMBOL_NUM];
//多普勒补偿
#pragma pack(push)//保存对齐状态
#pragma pack(8)
static float carrier_coeffs_b[3];
static float carrier_coeffs_a[3];
static float farrow_coeffs[FARROW_L+1][FARROW_K]=
        {
                0.032474036700817f,-0.067658026152908f,0.102878064226158f,0.882509689823010f,0.103152822081251f,-0.067940555744021f,0.032719896220746f,-0.000405508339707f,
                -0.045499412321274f,0.251765112012690f,-0.861611835465720f,0.025119624519583f,0.820264497775569f,-0.228511365187044f,0.039410320580367f,0.010708355868965f,
                -0.056168164068607f,-0.020654429986749f,0.746863090299732f,-1.432684428413126f,0.837734083854112f,-0.114682917234370f,0.008834315960858f,-0.022160711880417f,
                0.131713590640932f,-0.275795782501970f,0.185862454454762f,0.482620906683598f,-0.983583189584255f,0.730213926803076f,-0.283051234706633f,0.096532173182813f,
                -0.063399784553399f,0.145346707209532f,-0.242604681501185f,0.146628354633359f,0.103919225780439f,-0.215566998106516f,0.134172454268777f,-0.052129974480106f,
        };
static doppler_carrier_struct carrier_struct;
#pragma pack(pop)//恢复对齐状态

//turboeq
static float llr[K*2];														// turbo decoder input
static float *Lext   = (float*)(mpsk_data1_buf+0x5c00);							// turbo decoder output
static float *d1_out = ( float*)(mpsk_data1_buf+0x5c00+K*2);				// equalizer output;
static float llr_in[SYMBOL_NUM*2]=												// equalizer input(trian need initialized)
        {
#include "mpsk_dem_para/train_DSP.dat"
        };
static unsigned char mpsk_bit_out[1936];

//equalizer
static paradef para1;
static int idx [MAX_LEN_W]; // STATISTIC
static const float PSK_MOD[8] =
        {
                0.707107f,  0.707107f,
                -0.707107f,  0.707107f,
                -0.707107f, -0.707107f,
                0.707107f, -0.707107f
        };

//turbocode
TURBODEC_STRUCT myturbdec;
static float *alpha = myturbdec.alpha;
static float *beta = myturbdec.beta;
static float *gamma = myturbdec.gamma;
static float *temp = myturbdec.temp;
static float *temp1 = myturbdec.temp1;
static float *temp2 = myturbdec.temp2;
static float *temp3 = myturbdec.temp3;
static float *i1_in = myturbdec.i1_in;
static float *i2_in = myturbdec.i2_in;
static float *i1 = myturbdec.i1;
static float *i2 = myturbdec.i2;
static float *buf1 = myturbdec.buf1;
static float *buf2 = myturbdec.buf2;
static float *buf3 = myturbdec.buf3;

static const unsigned char nextStates[32]=
        {
                0,8,1,9,2,10,3,11,12,4,13,5,14,6,15,7,
                8,0,9,1,10,2,11,3,4,12,5,13,6,14,7,15
        };
static const unsigned char out_table_bit[32] =
        {
                0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,
                1,1,0,0,0,0,1,1,0,0,1,1,1,1,0,0
        };
static const int B[K]=
        {
#include "mpsk_dem_para/B.dat"
        };
static const int B01[K]=
        {
#include "mpsk_dem_para/B01.dat"
        };
static const unsigned char idx_1_gamma[32]		=
        {
                0, 3, 1, 2, 1, 2, 0, 3, 1, 2, 0, 3, 0, 3, 1, 2,
                0, 3, 1, 2, 1, 2, 0, 3, 1, 2, 0, 3, 0, 3, 1, 2
        };
static const unsigned char idx_2_gamma_t[32] 	=
        {
                0,0,1,1,1,1,0,0,1,1,0,0,0,0,1,1,
                3,3,2,2,2,2,3,3,2,2,3,3,3,3,2,2
        };
static const unsigned char idx_2_beta_t[32] 	=
        {
                0,8,1,9,2,10,3,11,12,4,13,5,14,6,15,7,
                8,0,9,1,10,2,11,3,4,12,5,13,6,14,7,15
        };
static const unsigned char idx_2_alpha_t[32] 	=
        {
                0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,
                0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
        };
static const unsigned char idx_2_gamma[32] 	=
        {
                0, 3, 0, 3, 1, 2, 1, 2, 1, 2, 1, 2, 0, 3, 0, 3,
                1, 2, 1, 2, 0, 3, 0, 3, 0, 3, 0, 3, 1, 2, 1, 2
        };
static const unsigned char idx_out2[32]=
        {
                0, 1, 6, 7, 10, 11, 12, 13, 18, 19, 20, 21, 24, 25, 30, 31,
                2, 3, 4, 5, 8, 9, 14, 15, 16, 17, 22, 23, 26, 27, 28, 29
        };
#pragma pack(push)//保存对齐状态
#pragma pack(8)
static float out_table[8] =  {-0.5f,-0.5f,0.5f,0.5f,-0.5f,0.5f,-0.5f,0.5f};
#pragma pack(pop)//恢复对齐状态

//多普勒补偿
static void farrow_fiter_float(float in[], int len_in, float doppler, float out[], float ptemp[]);
static void gen_carrier(float doppler, float ptemp[]);
static void doppler_compensation(float in[], int len_in, float doppler, float *ptemp, float out[]);

//turboeq
static int turboeq(float in[],char out[], int iter);
static void bsc(float in[], float out[]);// bit to symbol convertor
static void sbc(float in[], float out[]);// symbol to bit convertor

//equalizer
static void eq_FF_init(float in[], float in_d[], float out[]);
static void eq_FF(float in[], float in_d[], float out[]);
static void eq_FB_init(float in[], float in_d[], float out[]);
static void eq_FB(float in[], float in_d[], float out[]);
static void shift_U(float U[], float in[], float in_d[],paradef *para);
static float* sparse_U(float Uin[], float Uout[], paradef *para);
static void psk_demod(float in[],float out[]);
static void equalizer(float in[], float in_d[], float out[], paradef *para);
static void sort_bubble(float in[],float out[], int idx[], int n);
static void calc_sparse_idx(int idx[], float W[], paradef *para, int buf[]);

//turbocode
static void turbo_dec(float in[], float out[], int iter);
static void bit2char(unsigned char in[], char out[], int len);
static unsigned char crccheck(char in[],int len);
static void calc_alpha(float gamma[],float alpha[]);
static void calc_beta(float gamma[],float beta[]);
static void calc_L(float Lout[],float Lin[],float alpha [],float beta[], float gamma[]);
static void mat_idx(float in[],float out[], int m_in, int m_out, int n, const unsigned char idx[]);
static void siso_rsc(float Lin[], float Lout[]);
static void rsc_enc(unsigned char in[],unsigned char out[]);
static void turbo_enc(unsigned char in[],unsigned char out[]);
static void punc1110(float in[], float out[], float buf[]);
/* -------------------------------------接口函数 ------------------------------------------ */
void MPSK_DemInit(void)
{
    carrier_struct.flag_initialized = 0;
}
void mpsk_receiver(float in[], char out[], float doppler[], unsigned int frame_num)
{
    int j;
    float *p2;
    // in原始占用的是data2_buf,2*2*4*2136 = 34176 个float
    p2 				= (float*)mpsk_data1_buf; // 用于各通道多普勒补偿，需要2*2136*7 = 29904 个float
    for (j = 0;j < chnum;j ++)
    {
        doppler_compensation(in + (2 * Fs * SYMBOL_NUM) * j, SYMBOL_NUM * Fs, doppler[j], p2, in_float_sdram);
        DSPF_sp_mat_trans(in_float_sdram, SYMBOL_NUM, 2 * Fs, in + (2 * Fs * SYMBOL_NUM) * j);
    }
    DSPF_sp_mat_trans(in, 2 * Fs * chnum,SYMBOL_NUM,(float*)in_float_sdram);
    if(frame_num == 1)
    {
        bad_framenum = 0;
    }
    if(turboeq(in_float_sdram,out,((bad_framenum <= 4) ? TURBO_EQ_ITER : 0)) == 0)
    {
        bad_framenum += 1;
    }
}
/* -------------------------------------局部函数 ------------------------------------------ */
static float fclipf (float parm1, float parm2)//默认parm2是正的
{
    float out;
    if(parm1 > parm2)
    {
        out = parm2;
    }
    else if(parm1 < -parm2)
    {
        out = -parm2;
    }
    else
    {
        out = parm1;
    }
    return out;
}

/*****************************************************************************
Prototype    : farrow_fiter_float
Description  :
Input        : in[],len_in,doppler,out[],ptemp[]
Output       : NULL
Return Value : NULL
*****************************************************************************/
static void farrow_fiter_float(float in[], int len_in, float doppler, float out[], float ptemp[])
{
    int		i;
    float 	*fir_out;
    float 	sample_interval;
    float 	time;
    int 	pos;
    float 	delta1;
    float   out_i;
    float *filter_in;

    filter_in = (float*)data_a_space;//2136 * 2+ 8个float 最大16.8k
    fir_out 		= ptemp;
    memset(filter_in,0,(FARROW_K-1)*sizeof(float));
    memcpy(filter_in+(FARROW_K-1),in,len_in*sizeof(float));
    for(i = 0; i <= FARROW_L; i++)
    {
        DSPF_sp_fir_gen(filter_in,&farrow_coeffs[i][0],(fir_out + len_in*i),FARROW_K,len_in);
    }
    sample_interval	= (1 + doppler);
    memset(out,0,len_in*sizeof(float));
    for(i = 0;i < len_in;i ++)
    {
        time			= (i + FARROW_FILTERDELAY) * sample_interval;
        pos				= time;
        delta1			= time - pos;
        if(pos >= 0 && pos < len_in)
        {
            out_i		=					fir_out[4*len_in+pos];
            out_i		= delta1*out_i +	fir_out[3*len_in+pos];
            out_i		= delta1*out_i +	fir_out[2*len_in+pos];
            out_i		= delta1*out_i +	fir_out[1*len_in+pos];
            out_i		= delta1*out_i +	fir_out[		 pos];
            out[i]		= out_i;
        }
        else
        {
            out[i]		= 0;
        }
    }
    return;
}

static void mpsk_biquad(float *x, float *y, const int nx)
{
    int i;
    float w_x[3];
    float w_y[3];
    w_x[0]=w_x[1]=w_x[2]=0;
    w_y[0]=w_y[1]=w_y[2]=0;
    for (i = 0; i < nx; i++)
    {
        w_x[0] = x[i];
        w_y[0] = carrier_coeffs_b[0]*w_x[0]+carrier_coeffs_b[1]*w_x[1]+carrier_coeffs_b[2]*w_x[2] - w_y[1]*carrier_coeffs_a[1] - w_y[2]*carrier_coeffs_a[2];
        y[i] = w_y[0];//w_y[0]/carrier_coeffs_a[0],由于carrier_coeffs_a[0]=1，屏蔽
        w_x[2] = w_x[1];
        w_x[1] = w_x[0];
        w_y[2] = w_y[1];
        w_y[1] = w_y[0];
    }
}
/*****************************************************************************
 Prototype    : gen_carrier
 Description  : 产生载波信号
 Input        : doppler,ptemp[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void gen_carrier(float doppler, float ptemp[])
{
    float wc;
    float cos_wc;
    float sin_wc;
    int len;
    len = SYMBOL_NUM * Fs;
    wc					= 2.0f*PI*doppler;
    cos_wc				= cosf(wc);
    sin_wc				= sinf(wc);

    memset(carrier_struct.carrier, 0, len*sizeof(float));
    *carrier_struct.carrier = 1;
    carrier_coeffs_b[2] = 0.0f;
    carrier_coeffs_b[1] = -1.0f * cos_wc;
    carrier_coeffs_b[0] = 1.0f;
    carrier_coeffs_a[2] = 1.0f;
    carrier_coeffs_a[1] = -2.0f * cos_wc;
    carrier_coeffs_a[0] = 1.0f;

    mpsk_biquad(carrier_struct.carrier, ptemp, len);

    carrier_coeffs_b[2] = 0.0f;
    carrier_coeffs_b[1] = sin_wc;
    carrier_coeffs_b[0] = 0.0f;
    mpsk_biquad(carrier_struct.carrier, ptemp + len, len);

    DSPF_sp_mat_trans(ptemp, 2, len, carrier_struct.carrier);

    carrier_struct.flag_initialized = VALID_FLAG;
}

/*****************************************************************************
 Prototype    : doppler_compensation
 Description  : 多普勒补偿
 Input        : in[],len_in,doppler,*ptemp,out[]
 Output       : *ptemp
 Return Value : NULL
*****************************************************************************/
static void doppler_compensation(float in[], int len_in, float doppler, float *ptemp, float out[])
{
    int i;
    DSPF_sp_mat_trans(in,len_in,2,ptemp);
    farrow_fiter_float(ptemp,len_in,doppler,in,ptemp + len_in * 2);
    farrow_fiter_float(ptemp + len_in,len_in,doppler,in + len_in,ptemp + len_in * 2);
    DSPF_sp_mat_trans(in,2,len_in,ptemp);
    if (carrier_struct.flag_initialized != VALID_FLAG || (fabsf(carrier_struct.doppler-doppler)>0.0000000001))
    {
        gen_carrier(doppler,(ptemp + len_in * 2));
    }
    for(i = 0;i < len_in;i++)
    {
        cmltf(carrier_struct.carrier+i*2,ptemp+i*2,out+i*2);
    }
    return;
}

/*****************************************************************************
 Prototype    : turboeq
 Description  :
 Input        : in[],out[],iter
 Output       : NULL
 Return Value : ischeck
*****************************************************************************/
static int turboeq(float in[], char out[], int iter)
{
    int t;
    int i;
    unsigned char *encoded;
    int ischeck;
    float ber;
    ischeck = 0;

    for(t = 0;t < iter + 1;t++)
    {
        if(t == 0)
        {
            eq_FF_init (in, llr_in, d1_out); // get sparse position FF only
            eq_FF(in, llr_in, d1_out); // FF only
        }
        else
        {
            if (t == 1)
            {
                eq_FB_init(in, llr_in, d1_out); // get sparse position FF+FB
            }
            eq_FB(in, llr_in, d1_out);												// FF+FB
        }
        sbc(&d1_out[200*2], llr);															// symbol -> bit
        turbo_dec(llr, Lext, TURBO_DEC_ITER);											// decoder
        for(i = 0;i < 2 * K;i++)// Lin + Lext
        {
            Lext[i] += llr[i];
        }
        bsc(Lext, &llr_in[200*2]);													// symbol -> bit
        for(i = 0;i < K;i++)
        {
            mpsk_bit_out[B[i]] = Lext[2 * i] > 0;									// bit output
        }
        encoded = (unsigned char *)Lext;
        turbo_enc(mpsk_bit_out, encoded);													// encoder
        ber		= 0;														// equalizer's BER
        for(i = 0;i < K;i++)
        {
            ber	+= (d1_out[(200+i)*2+1]<0)!=encoded[2*i];
            ber	+= (d1_out[(200+i)*2]<0)!=encoded[2*i+1];
        }
        ber		= ber / K / 2.0f;
        bit2char(mpsk_bit_out, out, K);
        Endecode_Whiten(out, K/8, out);//去白化
        ischeck = crccheck(out, K/8);												// crc check
#ifdef DEBUG_MFSK
        ENetShell_DebugInfo("物理层：mpsk数据帧iter=%d lenW=%3d BER=%.5f CRC=%d]", t, para1.weight_idx, ber, ischeck);
#endif
        if(ischeck) break;
    }
    return ischeck;
}

/*****************************************************************************
 Prototype    : bsc
 Description  :
 Input        : in[],out[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void bsc(float in[], float out[])
{
    int i;

    for(i = 0;i < K;i++)
    {
        out[i*2] = tanhf(in[2 * i + 1] / 2) * (-0.707106781186547f);
        out[i*2+1] = tanhf(in[2 * i    ] / 2) * (-0.707106781186547f);
    }
}

/*****************************************************************************
 Prototype    : sbc
 Description  :
 Input        : in[],out[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void sbc(float in[], float out[])
{
    int i;
    for(i = 0;i < K;i++)
    {
        out[2*i+1] = -in[i*2];
        out[2*i]   = -in[i*2+1];
    }
}

/*****************************************************************************
 Prototype    : eq_FF_init
 Description  :
 Input        : in[],in_d[],out[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void eq_FF_init(float in[], float in_d[], float out[])
{
    para1.L1 			= 80;
    para1.L2 			= 0;
    para1.weight_idx 	= 0;
    para1.symbnum 		= 1936 + 200;
    para1.train_len 	= 200;
    equalizer(in, in_d, out, &para1);
}

/*****************************************************************************
 Prototype    : eq_FF
 Description  :
 Input        : in[],in_d[],out[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void eq_FF(float in[], float in_d[], float out[])
{
    equalizer(in, in_d, out, &para1);
}

/*****************************************************************************
 Prototype    : eq_FF
 Description  :
 Input        : in[],in_d[],out[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void eq_FB_init(float in[], float in_d[], float out[])
{
    para1.L1 			= 80;
    para1.L2 			= 80;
    para1.weight_idx 	= 0;
    para1.train_len 	= 200;
    equalizer(in, in_d, out, &para1);
}

/*****************************************************************************
 Prototype    : eq_FB
 Description  :
 Input        : in[],in_d[],out[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void eq_FB(float in[], float in_d[], float out[])
{
    para1.train_len = 200 + 1936;
    equalizer(in, in_d, out, &para1);
}

/*****************************************************************************
 Prototype    : eq_FB
 Description  :
 Input        : in[],in_d[],out[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void equalizer(float in[], float in_d[], float out[], paradef *para)
{
    int 			i,j;
    int 			i_U;
    int 			i_adaptive				= 0;
    int 			len_w_sparse;
    int 			len_w_sparse_quad_word;
    float 			cta;
    float 			imagp_e;
    float 			imagp_e_cumsum;
    float 	t1[2];
    float 	directed[2];
    float 	exp_cta[2];
    float 	*Upp;
    equalizer_buf 			*p_equalizer_buf;// need to be intialized to sdram_random
    float *W;
    float *U;
    float *U1;
    float *t2;
    float temp[2];

    p_equalizer_buf = (equalizer_buf *)mpsk_data2_buf;
    W 				= p_equalizer_buf->W;
    U 				= p_equalizer_buf->U;
    U1 				= p_equalizer_buf->U1;
    t2 				= p_equalizer_buf->t2;

    para->LEN_W 	= ((2 * para->L1 + 1) * SAMPLE_SIZE + 2 * para->L2);
    len_w_sparse 	= (para->weight_idx != 0) ? para->weight_idx : para->LEN_W;
    para->mu 		= 0.25f / len_w_sparse;
    memset(U, 0, MAX_LEN_W*2*sizeof(float));
    i_U				= 0;
    for(i = 0;i < para->L1 + 1;i ++)
    {
        shift_U(U, &in[i_U * SAMPLE_SIZE * 2], &in_d[i_U*2], para);
        i_U++;
    }
    Upp = sparse_U(U,U1,para);
    memset(W,0,len_w_sparse * 2 * sizeof(float));
    out[0] = 0;
    out[1] = 0;
    cta 			= 0;
    imagp_e_cumsum 	= 0;
    i_adaptive 		= (para->weight_idx == 0) ? para->train_len : para->symbnum;
    for(i = 1;i < i_adaptive;i ++)
    {
        if((i - 1) < para->train_len)
        {
            directed[0] = in_d[(i-1)*2];
            directed[1] = in_d[(i-1)*2+1];
        }
        else
        {
            psk_demod(&out[(i-1)*2], directed);
        }
        temp[0] = directed[0];
        temp[1] = -directed[1];
        cmltf(&out[(i-1)*2], temp, temp);
        imagp_e = fclipf(temp[1], 0.31415f);
        imagp_e_cumsum += imagp_e;
        cta += PLL_k1 * imagp_e + PLL_k2 * imagp_e_cumsum;
        exp_cta[0] = cosf(cta);
        exp_cta[1] = -sinf(cta);
        t1[0] = (directed[0]-out[(i-1)*2])*para->mu;
        t1[1] = (out[(i-1)*2+1]-directed[1])*para->mu;
        cmltf(t1, exp_cta, t1);
        for(j=0;j<len_w_sparse;j++)
        {
            cmltf(&Upp[j*2], t1, &t2[j*2]);
            W[j*2] += t2[j*2];
            W[j*2+1] += t2[j*2+1];
        }

        if(i_U < para->symbnum)
            shift_U(U, &in[i_U*SAMPLE_SIZE*2], &in_d[i_U*2], para);
        else
            shift_U(U, 0, 0, para);
        Upp = sparse_U(U,U1,para);
        i_U++;
        len_w_sparse_quad_word = (len_w_sparse + 3) & 0xfffc;
        for(j=0;j<len_w_sparse_quad_word;j++)
        {
            t2[j*2] = W[j*2];
            t2[j*2+1] = -W[j*2+1];
        }
        DSPF_sp_dotp_cplx(t2, Upp, len_w_sparse_quad_word, &out[i*2], &out[i*2+1]);
        cmltf(&out[i*2], exp_cta, &out[i*2]);
    }
    if(para->weight_idx == 0)
    {
        calc_sparse_idx(idx, W, para, (int*)t2);
    }
    return;
}

/*****************************************************************************
 Prototype    : calc_sparse_idx
 Description  :
 Input        : idx[],W[],*para,buf[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void calc_sparse_idx(int idx[], float W[], paradef *para, int buf[])
{
    float 	temp;
    int 	temp_int;
    int 	i;
    int 	j;
    int 	k;
    int 	ch_sum[chnum];
    int 	le_ff			= 0;
    int 	L2;
    float 	*x1;
    int 	*x;
    int 	*x2;
    int 	*x3;

    le_ff 	= para->L1 * 2 + 1;
    L2 		= para->L2;

    x1 		= (float*) buf;
    x 		= (int*)x1 + (le_ff)*Fs;
    x2 		= x + ((le_ff)*Fs*chnum+2*L2);
    x3 		= x2 + (le_ff)*Fs;

    for(j = 0;j < chnum;j++)
    {
        for(k = 0;k < le_ff;k++)
        {
            for(i = 0;i < Fs;i++)
            {
                x1[k * Fs + i] = CPOW(W[(k * Fs * chnum + j * Fs + i)*2],W[(k * Fs * chnum + j * Fs + i)*2+1]);
            }
        }
        temp = DSPF_sp_maxval(x1, Fs*le_ff);
        for(i = 0;i < Fs * le_ff;i++)
        {
            x2[i] = ((x1[i] > temp * 0.15)? 1 : 0);
        }
        for(k = 0;k < 0x6;k++)
        {
            x3[0] = x2[0] | x2[1];
            for(i = 1;i < Fs * le_ff - 1;i++)
            {
                x3[i] = x2[i-1] | x2[i] | x2[i+1];
            }
            x3[le_ff * Fs - 1] = x2[le_ff * Fs - 1] | x2[le_ff * Fs - 2];
            for(i = 0;i < Fs * le_ff;i++)
            {
                x2[i] = x3[i];
            }
        }
        for(i = 0;i < Fs * le_ff;i++)
        {
            x[j * Fs * le_ff + i] = x2[i];
        }
    }
    temp_int = MAX_LEN_W; // 求通道最小值min(x1)
    for(j = 0;j < chnum;j++)
    {
        ch_sum[j] 	= 0;
        for(i = 0;i < Fs * le_ff;i++)
        {
            ch_sum[j] += x[j * le_ff * Fs + i];
        }
        ch_sum[j] 	= (ch_sum[j] == 0) ? 10000 : ch_sum[j];
        temp_int 	= (temp_int < ch_sum[j]) ? temp_int : ch_sum[j];
    }
    if(temp_int < le_ff * Fs / 0x04) // 选择删除通道
    {
        for(j = 0;j < chnum;j++)
        {
            if(ch_sum[j] > le_ff * Fs / 0x02)
                for(i = 0;i < le_ff * Fs;i++)
                {
                    x[j * le_ff * Fs + i] = 0;
                }
        }
    }
    para->weight_idx = 0;
    for(j = 0;j < chnum;j++)
    {
        for(k = 0;k < le_ff;k++)
        {
            for(i = 0;i < Fs;i++)
            {
                idx[k * chnum * Fs + j * Fs + i] 	 = x[j * le_ff * Fs + k * Fs + i];
                para->weight_idx 				    += idx[k * chnum * Fs + j * Fs + i];
            }
        }
    }
    if(L2 > 0)
    {
        temp_int		= 0;
        for(i = 0;i < le_ff * Fs * chnum;i++)
        {
            temp_int   += idx[i];
        }

        temp_int 	= CEIL(temp_int, chnum);
        temp_int 	= ((temp_int < (2 * L2)) ? temp_int : (2 * L2));
        temp = 0;
        for(i = 0;i < 2 * L2;i++)
        {
            x1[i] = CPOW(W[(le_ff * chnum * Fs + i)*2], W[(le_ff * chnum * Fs + i)*2+1]);
            temp 	= fmax(temp,x1[i]);
        }
        sort_bubble(x1, x1, x2, 2*L2);
        for(i = 0;i < temp_int;i++)
        {
            j 							= x2[i];
            idx[le_ff * chnum * Fs + j] = ((x1[i] > (0.2 * temp)) ? 1 : 0);
        }
        for(i = 0;i < 2 * L2;i++)
        {
            para->weight_idx += idx[le_ff * chnum * Fs + i];
        }
    }
    para->weight_idx = ((para->weight_idx < MAX_SPARSE_LEN_W) ? para->weight_idx : MAX_SPARSE_LEN_W);
    j				 = 0;
    for(i = 0;i < para->LEN_W;i++)
    {
        if (idx[i] == 1)
            idx[j++] = i;
    }
}

/*****************************************************************************
 Prototype    : sort_bubble
 Description  :
 Input        : in[],out[],idx[],n
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void sort_bubble(float 	in[],
                        float 	out[],
                        int 	idx[],
                        int 	n)
{
    int 	i;
    int 	j;
    float 	temp;
    int 	temp_int;

    for(i = 0;i < n - 1;i++)
    {
        out[i]	= in[i];
        idx[i]	= i;
    }
    for(i = 0;i < n - 1;i++)
    {
        for(j = i + 1;j < n;j++)
        {
            if(out[i] < out[j])
            {
                temp		= out[i];
                out[i]		= out[j];
                out[j]		= temp;
                temp_int	= idx[i];
                idx[i]		= idx[j];
                idx[j]		= temp_int;
            }
        }
    }
}

/*****************************************************************************
 Prototype    : shift_internal
 Description  :
 Input        : *buf,*in,len,element_size
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void shift_internal(float *buf, float *in, int len, int element_size)
{
    DSPF_sp_blk_move((buf + element_size),buf,element_size*len);

    if(in != 0)
    {
        DSPF_sp_blk_move(in,(buf+element_size * (len - 1)),element_size);
    }
    else
    {
        memset((void*)(buf+element_size * (len - 1)), 0, element_size*sizeof(float));
    }
}

/*****************************************************************************
 Prototype    : shift_U
 Description  :
 Input        : *buf,*in,len,element_size
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void shift_U(float U[], float in[], float in_d[], paradef *para)
{
    int 		p_FB_1;														// first half FB
    int 		p_FB_2;														// second half FB
    int 		p_FB_3;														// FB tail

    shift_internal(U, in, 2 * para->L1 + 1, SAMPLE_SIZE * 2);
    if(para->L2)
    {
        p_FB_1 			= (2 * para->L1 + 1) * SAMPLE_SIZE;
        p_FB_2 			= p_FB_1 + para->L2;
        p_FB_3 			= p_FB_2 + para->L2;
        shift_internal(&U[p_FB_1*2], &U[p_FB_3*2], para->L2, 2);
        *((complex_float*)U+p_FB_3) = *((complex_float*)U+p_FB_2);
        shift_internal(&U[p_FB_2*2], in_d, para->L2, 2);
    }
    return;
}

/*****************************************************************************
 Prototype    : shift_U
 Description  :
 Input        : Uin[],Uout[],*para
 Output       : NULL
 Return Value : Upp
*****************************************************************************/
static float* sparse_U(float Uin[], float Uout[], paradef *para)
{
    int i;
    float *Upp;

    for(i = 0;i < para->weight_idx;i += 4)
    {
        *((complex_float*)Uout+i) = *((complex_float*)Uin+idx[i]);
        *((complex_float*)Uout+i+1) = *((complex_float*)Uin+idx[i+1]);
        *((complex_float*)Uout+i+2) = *((complex_float*)Uin+idx[i+2]);
        *((complex_float*)Uout+i+3) = *((complex_float*)Uin+idx[i+3]);
    }

    Upp		= (para->weight_idx != 0) ? Uout : Uin;
    return Upp;
}

/*****************************************************************************
 Prototype    : psk_demod
 Description  :
 Input        : in
 Output       : NULL
 Return Value : PSK_MOD[min_dist_idx]
*****************************************************************************/
static void psk_demod(float in[],float out[])
{
    int 	i;
    float 	min_dist;
    float 	temp;
    int   	min_dist_idx;

    min_dist 		= 100;
    min_dist_idx	= 0;

    for(i = 0;i < 4;i++)
    {
        temp = CPOW((in[0]-PSK_MOD[i*2]),(in[1]-PSK_MOD[i*2+1]));
        if(temp < min_dist)
        {
            min_dist		= temp;
            min_dist_idx 	= i;
        }
    }
    out[0] = PSK_MOD[min_dist_idx*2];
    out[1] = PSK_MOD[min_dist_idx*2+1];
    return;
}

/*****************************************************************************
 Prototype    : turbo_dec
 Description  :
 Input        : in[],out[],iter
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void turbo_dec(float in[], float out[], int iter)
{
    int i;
    int j;
    int t;
    int m;


    for(i = 0;i < K;i++)
    {
        *((complex_float*)buf1 + B[i]) = *((complex_float*)in + i);
    }
    punc1110(buf1, i1_in, buf3);
    for(i = 0;i < K/2;i++)
    {
        buf1[i*4+1] = 0;
    }
    for(i = 0;i < K;i++)
    {
        *((complex_float*)i2_in + i) = *((complex_float*)buf1 + B01[i]);
    }
    for(t = 0;t < iter + 1;t++)
    {
        siso_rsc(i1_in, i1);
        siso_rsc(i2_in, i2);
        for(i = 0;i < K;i++)
        {
            j = B01[i];
            *((complex_float*)buf1 + i) = *((complex_float*)i1 + j);
            *((complex_float*)buf2 + j) = *((complex_float*)i2 + i);
        }
        DSPF_sp_blk_move(buf2,i2,K * 2);
        for(i = 0;i < K;i++)
        {
            buf1[i*2+1] = 0;
            buf2[i*2+1] = 0;
        }
        for(m=0;m<K*2;m++)
        {
            i1_in[m] += buf2[m];
            i2_in[m] += buf1[m];
        }
    }
    for(i = 0;i < K/2;i++)
    {
        m = i*4;
        i1[m] += i2[m];
        i1[m+2] += i2[m+2];
        i1[m+3] = i2[m+3];
    }
    for(i = 0;i < K;i++)
    {
        *((complex_float*)out + i) = *((complex_float*)i1 + B[i]);
    }
}

/*****************************************************************************
 Prototype    : siso_rsc
 Description  :
 Input        : Lin[],Lout[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void siso_rsc(float Lin[], float Lout[])
{
    DSPF_sp_mat_mul(Lin, K, 2, out_table, 4, gamma);
    calc_alpha(gamma, alpha);
    calc_beta(gamma, beta);
    calc_L(Lout, Lin, alpha, beta, gamma);
}

/*****************************************************************************
 Prototype    : rsc_enc
 Description  :
 Input        : in[],out[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void rsc_enc(unsigned char in[], unsigned char out[])
{
    int 	i;
    unsigned char status = 0;
    unsigned char j;

    for(i = 0;i < K;i++)
    {
        j = ((in[i] & 0x1) << 4) + (status & 0xf);
        out[i] = out_table_bit[j];
        status = nextStates[j];
    }
}

/*****************************************************************************
 Prototype    : turbo_enc
 Description  :
 Input        : in[],out[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void turbo_enc(unsigned char in[], unsigned char out[])
{
    int i;
    unsigned char *buf0; // int [K]

    buf0 = (unsigned char *)mpsk_data2_buf;

    for(i = 0;i < K;i++)
    {
        out[i + K] = in[B01[i]];
    }
    rsc_enc(&out[K], out);
    for(i = 0;i < K;i++)
    {
        out[B01[i] + K] = out[i];
    }
    rsc_enc(in, out);
    for(i = 0;i < K;i += 2)
    {
        buf0[i] = out[i];
        buf0[i + 1] = out[K + i + 1];
    }
    for(i = 0;i < K;i++)
    {
        out[2 * i]  = in[B[i]];
        out[2 * i + 1] 	= buf0[B[i]];
    }
}

/*****************************************************************************
 Prototype    : punc1110
 Description  :
 Input        : in[],out[],buf[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void punc1110(float in[], float out[], float buf[])
{
    DSPF_sp_mat_trans(in, K / 2, 4, buf);
    memset(buf + K / 2 * 3, 0, K / 2 * sizeof(float));
    DSPF_sp_mat_trans(buf, 4, K / 2, out);
}

/*****************************************************************************
 Prototype    : calc_L
 Description  :
 Input        : Lin[],Lout[],alpha[],beta[],gamma[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void calc_L(float Lout[], float Lin[], float alpha[], float beta[], float gamma[])
{
    int 	i = 0;
    int 	i1;
    int 	i2;
    int 	j;
    float 	*p1;
    float 	*p2;

    for(i2 = 0;i2 < K2;i2++)
    {
        mat_idx(&alpha[i * Ms], temp1, Ms, Ms * 2, K1, idx_2_alpha_t);
        mat_idx(&beta[i * Ms], temp2, Ms, Ms * 2, K1, idx_2_beta_t);
        mat_idx(&gamma[i * Mo], temp3, Mo, Ms * 2, K1, idx_2_gamma_t);
        for(j=0;j<Ms * 2 * K1;j++)
        {
            temp1[j] += (temp2[j]+temp3[j]);
        }
        mat_idx(temp1, temp2, Ms * 2, Ms * 2, K1, idx_out2);
        p1		= temp1;
        p2		= temp2;
        for(i1 = 0;i1 < K1;i1++)
        {
            Lout[2 * i] 	= DSPF_sp_maxval(p1 + Ms, 16) - DSPF_sp_maxval(p1, 16);
            Lout[2 * i + 1] = DSPF_sp_maxval(p2 + Ms, 16) - DSPF_sp_maxval(p2, 16);
            i++;
            p1			   += Ms * 2;
            p2			   += Ms * 2;
        }
    }
    for(j=0;j<2 * K;j++)
    {
        Lout[j] -= Lin[j];
    }
}

/*****************************************************************************
 Prototype    : vec_idx_1_alpha
 Description  :
 Input        : in[],out[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void vec_idx_1_alpha(float in[], float out[])
{
    memcpy(out, in, 8*sizeof(float));
    out[8]		= in[9];
    out[9]		= in[8];
    out[10]		= in[11];
    out[11]		= in[10];
    out[12]		= in[13];
    out[13]		= in[12];
    out[14]		= in[15];
    out[15] 	= in[14];
    out[16]		= in[1];
    out[17]		= in[0];
    out[18]		= in[3];
    out[19] 	= in[2];
    out[20]		= in[5];
    out[21] 	= in[4];
    out[22]		= in[7];
    out[23] 	= in[6];
    memcpy(out + 24, in  + 8, 8*sizeof(float));
}

/*****************************************************************************
 Prototype    : vec_idx_2_beta
 Description  :
 Input        : in[],out[],
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void vec_idx_2_beta(float in[], float out[])
{
    out[0]		= in[0];
    out[1]		= in[8];
    out[2]		= in[8];
    out[3]		= in[0];
    out[4]		= in[1];
    out[5]		= in[9];
    out[6]		= in[9];
    out[7]		= in[1];
    out[8]		= in[2];
    out[9]		= in[10];
    out[10]		= in[10];
    out[11]		= in[2];
    out[12]		= in[3];
    out[13]		= in[11];
    out[14]		= in[11];
    out[15]		= in[3];
    out[16]		= in[12];
    out[17]		= in[4];
    out[18]		= in[4];
    out[19]		= in[12];
    out[20]		= in[13];
    out[21]		= in[5];
    out[22]		= in[5];
    out[23]		= in[13];
    out[24]		= in[14];
    out[25]		= in[6];
    out[26]		= in[6];
    out[27]		= in[14];
    out[28]		= in[15];
    out[29]		= in[7];
    out[30]		= in[7];
    out[31]		= in[15];
}


/*****************************************************************************
 Prototype    : acs
 Description  :
 Input        : in1[],in2[],out[],
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void acs(float in1[], float in2[], float out[])
{
    int i;
    for(i=0;i<32;i++)
    {
        in1[i] += in2[i];
    }

    out[0 ]		= fmax(in1[0 ],in1[1 ]);
    out[1 ]		= fmax(in1[2 ],in1[3 ]);
    out[2 ]		= fmax(in1[4 ],in1[5 ]);
    out[3 ]		= fmax(in1[6 ],in1[7 ]);
    out[4 ]		= fmax(in1[8 ],in1[9 ]);
    out[5 ]		= fmax(in1[10],in1[11]);
    out[6 ]		= fmax(in1[12],in1[13]);
    out[7 ]		= fmax(in1[14],in1[15]);
    out[8 ]		= fmax(in1[16],in1[17]);
    out[9 ]		= fmax(in1[18],in1[19]);
    out[10]		= fmax(in1[20],in1[21]);
    out[11]		= fmax(in1[22],in1[23]);
    out[12]		= fmax(in1[24],in1[25]);
    out[13]		= fmax(in1[26],in1[27]);
    out[14]		= fmax(in1[28],in1[29]);
    out[15]		= fmax(in1[30],in1[31]);
}

/*****************************************************************************
 Prototype    : mat_idx
 Description  :
 Input        : in[],out[],m_in,m_out,n,idx[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void mat_idx(float in[], float out[], int m_in, int m_out, int n, const unsigned char idx[])
{
    int i;
    DSPF_sp_mat_trans(in, n, m_in, out);
    for(i = 0;i < m_out;i++)
    {
        memcpy(temp+n*i, out+n*idx[i], n*sizeof(float));
    }
    DSPF_sp_mat_trans(temp, m_out, n, out);
}

/*****************************************************************************
 Prototype    : mat_idx
 Description  :
 Input        : in[],out[],m_in,m_out,n,idx[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void calc_alpha(float gamma[], float alpha[])
{
    int 	i;
    int 	i1;
    int 	i2;
    int 	j;
    float 	alpha_idxed[Ms*2];

    alpha[0]		=	0;
    for(i = 1;i < Ms;i++)
    {
        alpha[i]	=-  100;
    }
    i				= 0;
    for(i2 = 0;i2 < K2;i2++)
    {
        mat_idx(&gamma[(i - 1) * Mo], temp1, Mo, Ms * 2, K1, idx_1_gamma);
        i1			= 0;
        if(i == 0)
        {
            i1++;
            i++;
        }
        for(;i1 < K1;i1++)
        {
            vec_idx_1_alpha(&alpha[(i-1)*Ms], alpha_idxed);
            acs(alpha_idxed, &temp1[i1*Ms*2], &alpha[i*Ms]);
            for(j=(Ms-1);j>=0;j--)
            {
                alpha[i*Ms+j] -= alpha[i*Ms];
            }
            i++;
        }
    }
}

/*****************************************************************************
 Prototype    : calc_beta
 Description  :
 Input        : gamma[],beta[]
 Output       : NULL
 Return Value : NULL
*****************************************************************************/
static void calc_beta(float gamma[], float beta[])
{
    int 	i;
    int 	i1;
    int 	i2;
    int 	j;
    float 	beta_idxed[Ms*2];

    for(i = 0;i < Ms;i++)
    {
        beta[(K - 1) * Ms + i] 	= 0;
    }
    i							= K - 1;
    for(i2 = K2 - 1;i2 >= 0;i2--)
    {
        mat_idx(&gamma[(i + 2 - K1) * Mo], temp1, Mo, Ms * 2, K1, idx_2_gamma);
        i1						= K1 - 1;
        if(i == K - 1)
        {
            i1--;
            i--;
        }
        for(;i1 >= 0;i1--)
        {
            vec_idx_2_beta(&beta[(i + 1) * Ms], beta_idxed);
            acs(beta_idxed, &temp1[i1 * Ms * 2], &beta[i * Ms]);
            for(j=(Ms-1);j>=0;j--)
            {
                beta[i*Ms+j] -= beta[i*Ms];
            }
            i--;
        }
    }
}

static void bit2char(unsigned char in[], char out[], int len)
{
    int i;
    int j;
    for(i = 0;i < len / 8;i++)
    {
        out[i] = 0;
        for (j = 0;j < 8;j++)
        {
            out[i] |= in[8 * i + j] << j;
        }
    }
}

/*****************************************************************************
 Prototype    : crccheck
 Description  :
 Input        : in[],len
 Output       : NULL
 Return Value : is_check
*****************************************************************************/
static unsigned char crccheck(char in[], int len)
{
    unsigned char is_check	= 0;
    short crc16_g = 0;
    short crc16_r = 0;
    crc16_g = Endecode_Crc16(in, len - 2);
    crc16_r = in[len - 1] & 0x00ff;
    crc16_r = (crc16_r << 8) + (in[len - 2] & 0x00ff);
    if(crc16_g == crc16_r)
    {
        is_check = 1;
    }
    return is_check;
}
