//
// Created by lch on 10/10/22.
//

#ifndef DECODE_A_DECODE_A_H
#define DECODE_A_DECODE_A_H
//! 模式1短同步信号解调译码初始化，该函数在系统初始化时调用一次即可
void Mode1_Init(void);
//! 模式1短同步信号解调译码
//! \param[in] phy_signal_system 信号中心频率
//! \param[in] phy_mfsk_symbol_bb 基带符号长度，需要与发端一致，支持256、512、1024三种
//! \param[in] phy_mfsk_symble_num 每帧符号数，需要与发端一致
//! \param[in] src 输入基带数据地址，实部虚部交替出现，最大值归一到1
//! \param[in] group_num 每256个复数点为一组，输入数据组数
//! \param[out] PHY_Out_Buf 解调输出数据包，数组长度1414字节
//! \param[out] bit_len 数据包有效比特数
//! \param[out] link_src 数据包源地址，6bit有效位
//! \param[out] link_seqo 数据包序列号，4bit有效位
//如果相邻时间收到的link_src和link_seqo都相同，可以判定是重复译码成功数据包
//! return 0表示控制帧译码失败，1表示控制帧译码成功，2表示在控制帧译码成功的基础上，数据帧也译码成功
int Mode1_Decode(unsigned int phy_signal_system, unsigned int phy_mfsk_symbol_bb, unsigned int phy_mfsk_symble_num, float* src, unsigned short group_num, unsigned char * PHY_Out_Buf, unsigned int* pbit_len, unsigned char* plink_src, unsigned char* plink_seqo);
//! 模式1长同步信号解调译码初始化，该函数在系统初始化时调用一次即可
void Mode1_Long_Init(void);
//! 模式1长同步信号解调译码
//! \param[in] phy_signal_system 信号中心频率
//! \param[in] phy_mfsk_symbol_bb 基带符号长度，需要与发端一致，支持256、512、1024三种
//! \param[in] phy_mfsk_symble_num 每帧符号数，需要与发端一致
//! \param[in] src 输入基带数据地址，实部虚部交替出现，最大值归一到1
//! \param[in] group_num 每256个复数点为一组，输入数据组数
//! \param[out] PHY_Out_Buf 解调输出数据包，数组长度1414字节
//! \param[out] bit_len 数据包有效比特数
//! \param[out] link_src 数据包源地址，6bit有效位
//! \param[out] link_seqo 数据包序列号，4bit有效位
//如果相邻时间收到的link_src和link_seqo都相同，可以判定是重复译码成功数据包
//! return 0表示控制帧译码失败，1表示控制帧译码成功，2表示在控制帧译码成功的基础上，数据帧也译码成功
int Mode1_Long_Decode(unsigned int phy_signal_system, unsigned int phy_mfsk_symbol_bb, unsigned int phy_mfsk_symble_num, float* src, unsigned short group_num, unsigned char * PHY_Out_Buf, unsigned int* pbit_len, unsigned char* plink_src, unsigned char* plink_seqo);
//! 模式2解调译码初始化
void Mode2_Init(void);
//! 模式2解调译码，该函数在系统初始化时调用一次即可
//! \param[in] src 输入基带数据地址，实部虚部交替出现，最大值归一到1
//! \param[in] group_num 每2048个复数点为一组，输入数据组数
//! \param[out] out 解调输出数据,长度17字节
//! return 0表示译码失败，1表示译码成功
int Mode2_Decode(float* src, unsigned short group_num, char* out);
//开启一个UDP线程，广播调试信息，ip192.168.2.222 目的端口8082，源端口8084
void UdpNet_Task(void);
#endif //DECODE_A_DECODE_A_H
