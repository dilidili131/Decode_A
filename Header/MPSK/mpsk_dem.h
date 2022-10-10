/*
* mpsk_dem.h
*
*  Created on: 2021-9-1
*      Author: ioa
*/

#ifndef MPSK_DEM_H_
#define MPSK_DEM_H_
#include "stdint.h"
/* -------------------------------------接口函数声明------------------------------------------ */
void MPSK_DemInit(void);
void mpsk_receiver(float in[], char out[], float doppler[], unsigned int frame_num);
#endif /* MPSK_DEM_H_ */
