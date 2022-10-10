/*
* fh_dem.h
*
*  Created on: 2021-10-26
*      Author: ioa
*/

#ifndef FH_DEM_H_
#define FH_DEM_H_
#define FH_CHIRP_BASE_LEN  	                                                    	2048
void FH_DeModulate_Init(void);
//! 模式2解调译码
//! \param[in] src 输入基带数据地址，实部虚部交替出现，最大值归一到1
//! \param[in] group_num 每2048个复数点为一组，输入数据组数
//! \param[out] out 解调输出数据,长度17字节
//! return 0表示译码失败，1表示译码成功
int FH_SyncDecode(float* src, uint16_t group_num, char* out);
#endif /* FH_DEM_H_ */
