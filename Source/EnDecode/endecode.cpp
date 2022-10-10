/*!
 * \file endecode.c
 * \brief 数字通信信道编解码函数实现
 *
 * 日志
 * \n 2016-04-07, 1.0.3, zly, 修改CRC校验踢狗逻辑，每校验8M踢一次狗
 * \n 2015-11-19, 1.0.2, zly, CRC校验需要时间（12M可以，15M会溢出），数据太大看门狗会溢出，校验8M左右踢一次狗
 * \n 2011-10-27, 1.0.1, xzp, 加入wyb的对偶码译码程序，未做整理
 * \n 2011-07-20, 1.0.0, xzp, 创建
 *
 */

/* --------------------------------------- 头文件 -------------------------------------------- */
#include "stdafx.h"
#include "math.h"
#include "stdio.h"
#include "EnDecode/endecode.h"
/* ------------------------------------ 局部变量定义 ----------------------------------------- */
static const unsigned char chCRCHTalbe[] =                                 // CRC 高位字节值表
        {
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
                0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
                0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
                0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
                0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
                0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
                0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
                0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
                0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
                0x00, 0xC1, 0x81, 0x40
        };

// CRC 低位字节值表
static const unsigned char chCRCLTalbe[] =
        {
                0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
                0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
                0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
                0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
                0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
                0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
                0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
                0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
                0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
                0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
                0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
                0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
                0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
                0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
                0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
                0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
                0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
                0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
                0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
                0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
                0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
                0x41, 0x81, 0x80, 0x40
        };

// 白化序列
static unsigned char endecode_whiten_seq[64] =
        {
                0x01,0x22,0x26,0xAE,0x36,0x8E,0x54,0x6C,
                0x3E,0x8F,0x76,0x4A,0x90,0xB9,0xF8,0x1E,
                0xFC,0x87,0x77,0x68,0xB6,0x17,0xCE,0x90,
                0xA8,0xEB,0x49,0xE7,0xC0,0x5D,0x5E,0x29,
                0x50,0xF5,0xB5,0x60,0xB7,0x35,0xE8,0x3E,
                0x9E,0x65,0x1D,0x8B,0xFE,0xD2,0x28,0x63,
                0xC0,0x4C,0x4D,0x7E,0x4B,0xB2,0x9F,0x56,
                0x28,0x72,0xD3,0x1B,0x56,0x39,0x61,0x04
        };

/* ------------------------------------- 接口函数实现 ----------------------------------------- */
// crc16校验
short Endecode_Crc16(char *pchMsg, unsigned int nDataLen)
{
    unsigned char chCRCHi = 0xFF; // 高CRC字节初始化
    unsigned char chCRCLo = 0xFF; // 低CRC字节初始化

    if(nDataLen == 0)
        return 0;

    unsigned char chIndex = 0; // CRC循环中的索引
    while (nDataLen--)
    {
        // 计算CRC
        chIndex = chCRCLo ^ *pchMsg++;
        chCRCLo = chCRCHi ^ chCRCHTalbe[chIndex];
        chCRCHi = chCRCLTalbe[chIndex];
    }
    return ((chCRCHi << 8) | (unsigned char)chCRCLo);
}

// 白化
void Endecode_Whiten(char *pDataSrc, unsigned int nInLen, char *pDataDes)
{
    unsigned int k = 0;

    for(k = 0; k < nInLen; k++)
    {
        *pDataDes++ = ((*pDataSrc++) ^ endecode_whiten_seq[k % 64]);
    }
}

// 解白化
void Endecode_DeWhiten(char *pDataSrc, unsigned int nInLen, char *pDataDes)
{
    /*
    int k = 0;

    for(k = 0; k < nInLen; k++)
    {
        *pDataDes++ = ((*pDataSrc++) ^ endecode_whiten_seq[k % 64]);
    }
    */
    Endecode_Whiten(pDataSrc, nInLen, pDataDes);
}


void Endecode_5bitDeIntlev2(short *pDataSrc, unsigned int nInLen, short *pDataDes)
{
    unsigned int i = 0;

    int s = 0;
    int q = 0;

    unsigned int data_dest_index = 0 ;
    unsigned int intlv[2] = {0, 0};

    // 清目的数据内存
    unsigned int *pDataTemp = (unsigned int*)pDataDes;
    for( i = 0 ; i < nInLen; i++ ) // 2012-08-17修正
    {
        *(pDataTemp++) = 0;
    }

    s = pow( 2.0, ceil( (log(nInLen*1.0) / log(2.0)) ) );
    q = s / 4 - 1;

    for( i = 0 ; i < nInLen ; i++ )
    {
        data_dest_index = intlv[0];

        //------------------------------------------
        intlv[1] = ( 21 * intlv[0] + q ) % s ;
        while ( intlv[1] >= nInLen )
        {
            intlv[1] = ( 21 * intlv[1] + q ) % s ;
        }
        intlv[0] = intlv[1] ;
        //------------------------------------------

        *(pDataDes++) = *(pDataSrc + data_dest_index);
    }
}

void Endecode_32IntDeIntlev(INT32_DATA *pDataSrc, unsigned int nInLen, INT32_DATA *pDataDes)
{
    unsigned int i = 0;

    int s = 0;
    int q = 0;

    unsigned int data_dest_index = 0 ;
    unsigned int intlv[2] = {0, 0};

    // 清目的数据内存
    unsigned int *pDataTemp = (unsigned int*)pDataDes;
    for( i = 0 ; i < nInLen * 32; i++ )
    {
        *(pDataTemp++) = 0;
    }

    s = pow( 2.0, ceil( (log(nInLen*1.0) / log(2.0)) ) );
    q = s / 4 - 1;

    for( i = 0 ; i < nInLen ; i++ )
    {
        data_dest_index = intlv[0];

        //------------------------------------------
        intlv[1] = ( 21 * intlv[0] + q ) % s ;
        while ( intlv[1] >= nInLen )
        {
            intlv[1] = ( 21 * intlv[1] + q ) % s ;
        }
        intlv[0] = intlv[1] ;
        //------------------------------------------

        *(pDataDes++) = *(pDataSrc + data_dest_index);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
#define K 5
#define N 32
#define TRACE_BACK_LEN  16
#define IsTerminatedMode 0

#define BITGET(DATA, POS)	( ( (DATA) & ((unsigned int)(0x1 << (POS)) ) ) != 0 )
#define SHIFT_IN(DATA, IN)	( ((DATA) >> 1) | (((unsigned int)IN) != 0) << (TRACE_BACK_LEN - 1) )

//static unsigned int G2_table[N]={0, 2,	4, 6, 8, 10, 12, 14, 16, 18, 20, 22,	24,	26,	28,	30,	17,	19,	21,	23,	25,	27,	29,	31,	1,	3,	5,	7,	9,	11,	13,	15};
static unsigned int G2_table[N] = {0, 17, 1, 16, 2, 19, 3, 18, 4, 21, 5, 20, 6, 23, 7, 22, 8, 25, 9, 24, 10, 27, 11, 26, 12, 29, 13, 28, 14, 31, 15, 30};
struct track_buffer
{
    unsigned int bit[K];
} ;

static  struct track_buffer state_track		[N];
static  struct track_buffer state_track_copy[N];
static  unsigned int		metric_tmp		[N];
static  unsigned int		metric_copy	[N];
static  unsigned int		metric_survived	[N];
static  unsigned int		state_survived	[N];

static void			clear_track_buffer( struct track_buffer *in);
static void			push_track_buffer( struct track_buffer *out,struct track_buffer * in, unsigned int data_in, int position);
static unsigned int	pop_track_buffer( struct track_buffer *in, int position);
static unsigned int	search_max_idx( unsigned int in[], int len_in);
static unsigned int	search_min_idx( unsigned int in[], int len_in);

// 解对偶码，by wyb
void Endecode_DeDualK5(void * decision,unsigned int decoded[], int len, int isSoftDecision)
{
    int iTime;
    int i;
    int j;
    unsigned long *pdecision;

    pdecision = (unsigned long *)decision;;

    for(i = 0; i < N; i++)
    {
        metric_survived[i] = 0;
        clear_track_buffer(&state_track[i]);
        clear_track_buffer(&state_track_copy[i]);
    }

    for (iTime = 0; iTime < len; iTime++)
    {

        // save survived metrics
        for (i=0; i<N; i++)
            metric_copy[i] = metric_survived[i];
        // search the survived state
        // i: input(next state), j:(last state)
        for (i = 0; i < N; i++)
        {
            for (j = 0; j < N; j++)
            {
                if(isSoftDecision)
                {
                    metric_tmp[j] = metric_copy[j]
                                    + pdecision[iTime*(2 * N) + (unsigned int)(i	^j)]
                                    + pdecision[iTime*(2 * N) + N + (unsigned int)(G2_table[i] ^ j)];
                }
                else
                {
                    metric_tmp[j] = metric_copy[j]
                                    + (pdecision[iTime * 2] == (unsigned int)(i	^j))
                                    + (pdecision[iTime * 2 + 1] == (unsigned int)(G2_table[i] ^ j));
                }


            }
            state_survived[i] = search_max_idx( metric_tmp, N );
            metric_survived[i] = metric_tmp[ state_survived[i] ];
        }
        // substact common value avoiding overflow
        metric_tmp[0] = metric_survived[search_min_idx( metric_survived, N )];
        for (i = 0; i < N; i++)
            metric_survived[i]-=metric_tmp[0] ;
        // save tracking buffer
        for (i = 0; i < N; i++)
            state_track_copy[i] = state_track[i];
        // push survived state to tracking buffer
        for (i = 0; i < N; i++)
        {
            push_track_buffer(&state_track[i],
                              &state_track_copy[state_survived[i]],
                              state_survived[i],
                              TRACE_BACK_LEN-1);
        }
        // pop tracking state
        if(iTime >= TRACE_BACK_LEN)
        {
            i = search_max_idx(metric_survived, N);
            decoded[iTime - TRACE_BACK_LEN] = pop_track_buffer(&state_track[i], 0);
        }
    }

    if (IsTerminatedMode)						// if encoder is ended with termination
        i = 0;									// return to 0 status
    else										// if encoder is ended with truction
        i = search_max_idx(metric_survived, N);	// find surviving status

    // pop all status in buffer
    for (iTime = 1; iTime < TRACE_BACK_LEN; iTime++)
        decoded[ len - TRACE_BACK_LEN + iTime -1 ] = pop_track_buffer(&state_track[i], iTime);
    decoded[ len - 1 ] = i;

    return;
}

static unsigned int search_max_idx(unsigned int in[], int len_in)
{
    int j;
    unsigned int max_value = in[0];
    int max_idx = 0;

    for (j = 1; j < len_in; j++)
    {
        if(max_value < in[j])
        {
            max_value =in[j];
            max_idx	=j;
        }
    }
    return max_idx;
}

static unsigned int search_min_idx(unsigned int in[], int len_in)
{
    int j;
    unsigned int min_value=in[0];
    int min_idx=0;
    for (j=1; j<len_in; j++)
    {
        if(min_value>in[j])
        {
            min_value	=in[j];
            min_idx		=j;
        }
    }
    return min_idx;
}

static void clear_track_buffer(struct track_buffer *in)
{
    int k;

    for (k = 0; k < K; k++)
    {
        in->bit[k] =0;
    }
}

static void push_track_buffer(struct track_buffer * out,struct track_buffer * in, unsigned int data_in, int position)
{
    // bit0, oldest input
    // bitx, newest input
    int k;

    for(k = 0; k < K; k++)
    {
        out->bit[k] = (in->bit[k] >> 1) | ((unsigned int )BITGET(data_in, k)) << position;
    }
}

static unsigned int pop_track_buffer(struct track_buffer *in, int position)
{
    int k;
    unsigned int data=0;

    for(k = 0; k < K; k++)
    {
        data |= BITGET(in->bit[k], position) << k ;
    }

    return data;
}
