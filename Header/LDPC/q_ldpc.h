/*
* q_ldpc.h
*
*  Created on: 2021-7-29
*      Author: ioa
*/

#ifndef Q_LDPC_H_
#define Q_LDPC_H_
#include "stdint.h"
extern unsigned char data_a_space[];//开辟出一块片内空间，类似SDRAM_BANK2的使用
/* ------------------------------------ 接口函数声明 ----------------------------------------- */
unsigned int ldpc_encoder(char *pDataSrc, unsigned int nInLen, unsigned int nBitValidNum, unsigned char syn_num, unsigned char *pDataDes, unsigned char is_DATA, unsigned int phy_mfsk_symbol_bb);
unsigned int ldpc_decoder(short l_chan[], unsigned char p_decision[], unsigned char frame_num, unsigned char syn_num, unsigned char ite_num, unsigned char is_DATA, unsigned int phy_mfsk_symbol_bb);//输入960*64，输出240字节，输出240字节,校验正确返回长度，校验错误返回0
#endif /* Q_LDPC_H_ */
