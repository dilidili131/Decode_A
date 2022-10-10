/*!
* \file endecode.h
* \brief 数字通信信道编解码函数定义
*
*/

#ifndef _ENDECODE_H_
#define _ENDECODE_H_
#include "stdint.h"
/* ------------------------------------ 接口函数声明 ----------------------------------------- */
/*! 将输入的字节数据进行crc16校验，生成多项式为：x16 + x15 + x2 + 1
*	\param[in] pDataSrc 数据源指针，以字节寻址
*	\param[in] nInLen 数据源长度，以字节为单位
*	\return 数据源的校验值，两个校验字节组成的int型数据
*/short Endecode_Crc16(char *pDataSrc, unsigned int nInLen);

/*!	数据白化，采用m序列进行加扰，m序列周期为511，输出数据长度 = 输入数据长度
*	\param[in] pDataSrc 数据源指针，以字节寻址
*	\param[in] nInLen 数据源长度，以字节为单位
*	\param[out] pDataDes 白化后的数据地址指针，以字节寻址
*/void Endecode_Whiten(char *pDataSrc, unsigned int nInLen, char *pDataDes);

/*!	对输入的数据进行1/4卷积码编码，输出数据长度 = 输入数据长度 X 4。
*  生成矩阵为[171 133 145 133]（八进制）, 约束长度K=7
*	\param[in] pDataSrc 数据源指针，以字节寻址
*	\param[in] nInLen 数据源长度，以字节为单位
*  \param[out] pDataDes 卷积编码后的数据地址指针，以字节寻址
*/void Endecode_Conv14(char *pDataSrc, unsigned int nInLen, char *pDataDes);

/*! 对输入的数据进行非二进制对偶编码，K = 5，输出数据长度 = 输入数据长度 X 2。
*	\param[in] pDataSrc 数据源指针，以32位int寻址，低30bit有效
*	\param[in] nInLen 数据源长度，以32位int为单位
*  \param[out] pDataDes 卷积编码后的数据地址指针，以32位int寻址
*  \param[out] bIsRepeat 编码是否重复，1为重复，0不重复
*/void Endecode_DualK5(long *pDataSrc, unsigned int nInLen, long *pDataDes, unsigned char bIsRepeat);

/*!	对输入的数据进行1bit级交织，输出数据长度 = 输入数据长度。
*	\param[in] pDataSrc 数据源指针，以字节寻址
*	\param[in] nInLen 数据源长度，以字节为单位
*	\param[out] pDataDes 交织后的数据地址指针，以字节寻址
*/void Endecode_1BitIntlev(char *pDataSrc, unsigned int nInLen, char *pDataDes);

/*!	对输入的数据5bit一组进行交织，输出数据长度 = 输入数据长度。
*	\param[in] pDataSrc 数据源指针，以32位int寻址，低30bit有效
*	\param[in] nInLen 数据源长度，以32位int为单位
*	\param[out] pDataDes 交织后的数据地址指针，以32位int寻址，低30bit有效
*/void Endecode_5BitIntlev(long *pDataSrc, unsigned int nInLen, long *pDataDes);

/*!	将输入的字节数据转换为低30bit有效的unsigned int型
*	\param[in] pDataSrc 数据源指针，以字节寻址
*	\param[in] nInLen 数据源长度，以字节为单位
*	\param[out] pDataDes 转换后的数据指针，以32bit的int为单位寻址
* \param[out] pOutLen 输出数据长度
*/void Endecode_Byte2Int30b(char *pDataSrc, unsigned int nInLen, long *pDataDes, unsigned int *pOutLen);

/*! 将输入的int型数据(低30bit有效)转换为字节数据
*	\param[in] pDataSrc 数据源指针，以32位int型寻址
*  \param[in] nInlen 数据源长度，以int为单位
*	\param[out] pDataDes 转换后的数据指针，以字节寻址
*  \param[out] pOutLen 输出数据长度，以字节为单位
*/void Endecode_Int30b2Byte(int *pDataSrc, unsigned int nInlen, char *pDataDes, int *pOutLen);

/*!	对输入的数据进行解交织，输出数据长度 = 输入数据长度。
*	\param[in] pDataSrc 数据源指针，以字节寻址
*	\param[in] nInLen 数据源长度，以字节为单位
*	\param[out] pDataDes 解交织后的数据地址指针，以字节寻址
*/void Endecode_DeIntlev(char *pDataSrc, unsigned int nInLen, char *pDataDes);

//! 5bit一组做解交织,输入int型低30bit有效
void Endecode_5BitDeIntlev(long *pDataSrc, unsigned int nInLen, long *pDataDes);

long Endecode_5bit2Byte(long *pDataSrc, unsigned int nInLen, char *pDataDes);

/*!	对m序列周期为511的数据去白化，输出数据长度 = 输入数据长度
*	\param[in] pDataSrc 数据源指针，以字节寻址
*	\param[in] nInLen 数据源长度，以字节为单位
*	\param[out] pDataDes 去白化后的数据地址指针，以字节寻址
*/
void Endecode_DeWhiten(char *pDataSrc, unsigned int nInLen, char *pDataDes);


typedef struct tagINT32_DATA
{
    unsigned int data[32];
}INT32_DATA;

//! 32个int型为一组，每个组看作一个元素，对这些元素进行解交织
void Endecode_32IntDeIntlev(INT32_DATA *pDataSrc, unsigned int nInLen, INT32_DATA *pDataDes);

//! 5bit一组做解交织,输入shor型为5bit
void Endecode_5bitDeIntlev2(short *pDataSrc, unsigned int nInLen, short *pDataDes);

//! 对偶码译码
void Endecode_DeDualK5(void *decision, unsigned int decoded[], int len, int isSoftDecision);
#endif