/*
* enet_shell.h
*
*  Created on: 2021-4-23
*      Author: ioa
*/

#ifndef ENET_SHELL_H_
#define ENET_SHELL_H_
/*!
* \file enet_shell.h
* \brief 网口shell
*
*/
#define	LOG_MESSAGE		0
#define	LOG_WARNING		1
#define	LOG_EXCEPTION	2
#define	LOG_ERROR		3

#define LOG(msg)  Log(__FILE__, __LINE__, LOG_MESSAGE, msg)
#define LOGW(msg) Log(__FILE__, __LINE__, LOG_WARNING, msg)
#define LOGE(msg) Log(__FILE__, __LINE__, LOG_EXCEPTION, msg)
#define LOGX(msg) Log(__FILE__, __LINE__, LOG_ERROR, msg)

#define PRE_LOG LogN(__FILE__, __LINE__, LOG_MESSAGE,
#define LOG1(str, p1) PRE_LOG (str), (p1))
#define LOG2(str, p1, p2) PRE_LOG (str), (p1), (p2))
#define LOG3(str, p1, p2, p3) PRE_LOG (str), (p1), (p2), (p3))
#define LOG4(str, p1, p2, p3, p4) PRE_LOG (str), (p1), (p2), (p3), (p4))
#define LOG5(str, p1, p2, p3, p4, p5) PRE_LOG (str), (p1), (p2), (p3), (p4), (p5))
#define LOG6(str, p1, p2, p3, p4, p5, p6) PRE_LOG (str), (p1), (p2), (p3), (p4), (p5), (p6))
#define LOG7(str, p1, p2, p3, p4, p5, p6, p7) PRE_LOG (str), (p1), (p2), (p3), (p4), (p5), (p6), (p7))
#define LOG8(str, p1, p2, p3, p4, p5, p6, p7, p8) PRE_LOG (str), (p1), (p2), (p3), (p4), (p5), (p6), (p7), (p8))
#define LOG9(str, p1, p2, p3, p4, p5, p6, p7, p8, p9) PRE_LOG (str), (p1), (p2), (p3), (p4), (p5), (p6), (p7), (p8), (p9))

#define PRE_WARN_LOG LogN(__FILE__, __LINE__, LOG_WARNING,
#define LOGW1(str, p1) PRE_WARN_LOG (str), (p1))
#define LOGW2(str, p1, p2) PRE_WARN_LOG (str), (p1), (p2))
#define LOGW3(str, p1, p2, p3) PRE_WARN_LOG (str), (p1), (p2), (p3))
#define LOGW4(str, p1, p2, p3, p4) PRE_WARN_LOG (str), (p1), (p2), (p3), (p4))
#define LOGW5(str, p1, p2, p3, p4, p5) PRE_WARN_LOG (str), (p1), (p2), (p3), (p4), (p5))
#define LOGW6(str, p1, p2, p3, p4, p5, p6) PRE_WARN_LOG (str), (p1), (p2), (p3), (p4), (p5), (p6))
#define LOGW7(str, p1, p2, p3, p4, p5, p6, p7) PRE_WARN_LOG (str), (p1), (p2), (p3), (p4), (p5), (p6), (p7))
#define LOGW8(str, p1, p2, p3, p4, p5, p6, p7, p8) PRE_WARN_LOG (str), (p1), (p2), (p3), (p4), (p5), (p6), (p7), (p8))
#define LOGW9(str, p1, p2, p3, p4, p5, p6, p7, p8, p9) PRE_WARN_LOG (str), (p1), (p2), (p3), (p4), (p5), (p6), (p7), (p8), (p9))

#define PRE_EXCEPTION_LOG LogN(__FILE__, __LINE__, LOG_EXCEPTION,
#define LOGE1(str, p1) PRE_EXCEPTION_LOG (str), (p1))
#define LOGE2(str, p1, p2) PRE_EXCEPTION_LOG (str), (p1), (p2))
#define LOGE3(str, p1, p2, p3) PRE_EXCEPTION_LOG (str), (p1), (p2), (p3))
#define LOGE4(str, p1, p2, p3, p4) PRE_EXCEPTION_LOG (str), (p1), (p2), (p3), (p4))
#define LOGE5(str, p1, p2, p3, p4, p5) PRE_EXCEPTION_LOG (str), (p1), (p2), (p3), (p4), (p5))
#define LOGE6(str, p1, p2, p3, p4, p5, p6) PRE_EXCEPTION_LOG (str), (p1), (p2), (p3), (p4), (p5), (p6))
#define LOGE7(str, p1, p2, p3, p4, p5, p6, p7) PRE_EXCEPTION_LOG (str), (p1), (p2), (p3), (p4), (p5), (p6), (p7))
#define LOGE8(str, p1, p2, p3, p4, p5, p6, p7, p8) PRE_EXCEPTION_LOG (str), (p1), (p2), (p3), (p4), (p5), (p6), (p7), (p8))
#define LOGE9(str, p1, p2, p3, p4, p5, p6, p7, p8, p9) PRE_EXCEPTION_LOG (str), (p1), (p2), (p3), (p4), (p5), (p6), (p7), (p8), (p9))

#define PRE_ERR_LOG LogN(__FILE__, __LINE__, LOG_ERROR,
#define LOGX1(str, p1) PRE_ERR_LOG (str), (p1))
#define LOGX2(str, p1, p2) PRE_ERR_LOG (str), (p1), (p2))
#define LOGX3(str, p1, p2, p3) PRE_ERR_LOG (str), (p1), (p2), (p3))
#define LOGX4(str, p1, p2, p3, p4) PRE_ERR_LOG (str), (p1), (p2), (p3), (p4))
#define LOGX5(str, p1, p2, p3, p4, p5) PRE_ERR_LOG (str), (p1), (p2), (p3), (p4), (p5))
#define LOGX6(str, p1, p2, p3, p4, p5, p6) PRE_ERR_LOG (str), (p1), (p2), (p3), (p4), (p5), (p6))
#define LOGX7(str, p1, p2, p3, p4, p5, p6, p7) PRE_ERR_LOG (str), (p1), (p2), (p3), (p4), (p5), (p6), (p7))
#define LOGX8(str, p1, p2, p3, p4, p5, p6, p7, p8) PRE_ERR_LOG (str), (p1), (p2), (p3), (p4), (p5), (p6), (p7), (p8))
#define LOGX9(str, p1, p2, p3, p4, p5, p6, p7, p8, p9) PRE_ERR_LOG (str), (p1), (p2), (p3), (p4), (p5), (p6), (p7), (p8), (p9))
/* ------------------------------------ 接口函数声明 ----------------------------------------- */
extern void Hrt_UdpNet_Task(void);
//! for udp 广播调试信息
extern void Log(char *szSrc, unsigned long uLine, int type, char *msg);
extern void LogN(char *szSrc, unsigned long uLine, int type, char *szFormat, ...);
extern int ENetShell_DebugInfo(char *szFormat, ...);
#endif /* ENET_SHELL_H_ */
