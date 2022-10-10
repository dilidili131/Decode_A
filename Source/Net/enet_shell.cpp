/*!
 * \file enet_shell.c
 * \brief 网口shell
 *
 * 日志
 */
// 组件
#include "stdafx.h"
// C 语言函数库
#include <stdio.h>
#include<stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "Net/enet_shell.h"
#include <sys/socket.h>
#include <netinet/in.h>
//#pragma comment(lib,"ws2_32.lib")
/* --------------------------------- 常量和宏定义 ------------------------------------ */
#define UDP_PORT_BroadMSG  		8082
#define UDP_PORT_Local  		8084
/* ------------------------------------ 局部变量定义 ----------------------------------------- */
static char myIPAddr[16]={"192.168.2.222"};
static int udpsock = -1;
#pragma pack(push)//保存对齐状态
#pragma pack(2)
static unsigned char enetshell_udpbuf[1024];    	// shell界面socket接收缓存
#pragma pack(pop)//恢复对齐状态
static char shell_szBuffer_1[256] = {'\0'};//for ENetShell_DebugInfo,ENetShell_Printf
static char shell_szBuffer_2[256+64] = {'\0'};//for ENetShell_DebugInfo
#define LOG_DATABUF_SIZE (256*10)
/* ------------------------------------ 局部变量定义 ----------------------------------------- */
static char log_szBuffer[256] = { 0 };
// 注意：该函数尽量不要在中断中调用
void Log(char *szSrc, unsigned long uLine, int type, char *msg)
{
    ENetShell_DebugInfo(msg);
}

void LogN(char *szSrc, unsigned long uLine, int type, char *szFormat, ...)
{
    va_list va;
    va_start(va, szFormat);
    vsnprintf(log_szBuffer, 256 - 1, szFormat, va);
    va_end(va);

    Log(szSrc, uLine, type, log_szBuffer);
}
/**
 * 事件处理
 */
void hrt_UdpSend_Msg(void * pMem,int pLen, struct sockaddr_in * addr,unsigned short port)
{
    struct  sockaddr_in sin1;

    sin1.sin_family      = AF_INET;
    sin1.sin_addr.s_addr = addr->sin_addr.s_addr;
    sin1.sin_port        = htons(port);

    if ((sendto(udpsock, (char*)pMem, pLen, 0, (struct sockaddr * )&sin1, sizeof(sin1))) < 0)
    {
        printf("send failed\n");
    }
}
void Hrt_UdpNet_Task(void)
{
    struct  sockaddr_in sin1;
    int nNetTimeout;
    int so_broadcast = 1;
    WSADATA wsd;
    struct sockaddr_in disaddr;
    int rcvLen = 0;
    int addrLen = sizeof(struct sockaddr_in);

    if(WSAStartup(MAKEWORD(2,2),&wsd)!=0)//初始化套接字动态库
    {
        printf("WSAStartup failed!\n");
        return;
    }
    ZeroMemory(&sin1, sizeof(struct sockaddr_in));
    sin1.sin_family      = AF_INET;
    sin1.sin_addr.s_addr = inet_addr(myIPAddr);
    sin1.sin_port = htons(UDP_PORT_Local);
    // 创建套接字
    nNetTimeout = 2;
    udpsock= socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    setsockopt(udpsock,SOL_SOCKET,SO_SNDTIMEO,(char*)&nNetTimeout, sizeof(int));
    setsockopt(udpsock,SOL_SOCKET,SO_BROADCAST,(char*)&so_broadcast,sizeof(so_broadcast));

    if(udpsock == INVALID_SOCKET)
    {
        printf("udp failed\n");
//        WSACleanup();	//释放套接字资源
        return;
    }
    /* Connect socket */
    if ( bind( udpsock, (struct sockaddr*) &sin1, sizeof(sin1) ) < 0 )
    {
        printf("failed bind)\n");
        close(udpsock);	//关闭套接字
//        WSACleanup();	//释放套接字资源
        return;
    }
    while(1)
    {
        rcvLen = recvfrom(udpsock, (char*)enetshell_udpbuf, 1024, 0, (struct sockaddr *)&disaddr,
                          reinterpret_cast<socklen_t *>(&addrLen));

        if(rcvLen > 0)
        {
            //hrt_UdpSend_Msg(udpRcvData,rcvLen,(struct sockaddr_in *)&(disaddr),(unsigned short)UDP_PORT_BroadMSG);
        }
    }
}

// for udp 广播调试信息
int ENetShell_DebugInfo(char *szFormat, ...)
{
//    SYSTEMTIME st;
    time_t timeReal;
    time(&timeReal);
    timeReal = timeReal + 8*3600;
    tm* t = gmtime(&timeReal);
    unsigned int rtc_ms = 0;
    static unsigned int packet_no = 0;
    int ret;
    //char destip[16] = { "255.255.255.255" }; // udp
    struct  sockaddr_in sin1;
    sin1.sin_family = AF_INET;
    sin1.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    sin1.sin_port = htons(UDP_PORT_BroadMSG);

    va_list va;
    va_start(va, szFormat);
    vsnprintf(shell_szBuffer_1, 256 - 1, szFormat, va);
    va_end(va);
//    GetLocalTime(&st); // 获取当前时间
//    ret = sprintf_s(shell_szBuffer_2, "[%03d][%04d-%02d-%02d %02d:%02d:%02d:%03d]: %s\r\n", packet_no, st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds, shell_szBuffer_1);
    ret = sprintf(shell_szBuffer_2, "[%03d][%04d-%02d-%02d %02d:%02d:%02d]: %s\r\n", packet_no, t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec,  shell_szBuffer_1);

    ret = sendto(udpsock, shell_szBuffer_2, strlen(shell_szBuffer_2), 0, (struct sockaddr*)&sin1, sizeof(sin1));
    packet_no++;
    if (packet_no == 1000)
        packet_no = 0;
    return ret;
}