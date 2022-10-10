/*
* FH_Related_Func.h
*
*  Created on: 2021-10-26
*      Author: ioa
*/

#ifndef FH_RELATED_FUNC_H_
#define FH_RELATED_FUNC_H_
#include "stdint.h"
/* -------------------------------------接口参数声明------------------------------------------ */
extern const unsigned char 		Mod_Hop_Map[1056];
extern const uint16_t 	Interleave_Seq[1056];
/* -------------------------------------接口函数声明------------------------------------------ */
void 	FH_Cnv_Enc419(unsigned char *input,unsigned char *output,int inlen);
void 	FH_Whiten(unsigned char *pDataSrc,unsigned char *pDataDes);
#endif /* FH_RELATED_FUNC_H_ */