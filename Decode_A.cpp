#include <string.h>
#include "Decode_A.h"
#include "dsplib.h"
#include "global.h"
#include "Net/enet_shell.h"
#include "EnDecode/endecode.h"
#include "MFSK/mfsk_dem.h"
#include "MFSK/mfsk_long_dem.h"
#include "FH/fh_dem.h"
#define CONTROL_FRAME_CYCLE_PREFIX(F0,SYMBOL_BB)	(((F0==24000)?5:(F0==8000)?20:16)*SYMBOL_BB/256)	// 网络控制帧的循环前缀规定为MFSK最长循环前缀(ms)，8kHz为20ms、10kHz、20kHz、3.75k为16ms，24kHz时为10ms
// 网络层和数据链路层共享
#define FRAME_PHY_MAXLEN		(1414)				//!< 物理层缓冲区phy_frame的长度,接收时物理层最大发射1414字节数据，并非对应整数帧，前34+7帧对应8158 240*35
// data_link_layer 发送或接受数据的方式，在结构体link_send_queue的send_way和结构体link_rev_queue的recv_way中用
#define ALOHA_WAY				0x01
#define REPEAT_CODE				1			// 循环编码
#define NOT_REPEAT_CODE				0			// 循环编码

// 网络控制帧公共域中包类型定义，fc_frame_type
#define RTS_PLUS					0x03
#define CONTROL_FRAME_COMMON_LEN	sizeof(frame_common)									//网络控制帧公共部分结构体的长度,4 Bytes
#define CONTROL_FRAME_LEN			sizeof(rts_frame)										//网络控制帧结构体的长度,12 Bytes
#define CONTROL_FRAME_SPECIAL_LEN	(CONTROL_FRAME_LEN - CONTROL_FRAME_COMMON_LEN)			//网络控制帧专用域结构体的长度,8 Bytes
#define CONTROL_FRAME_PHY_LEN		9														//传给物理层的网络控制帧长度,9 Bytes
#define CONTROL_FRAME_PHY_BITVALID	69														//传给物理层的网络控制帧内有效比特数

// 定义各网络控制帧的x，x表示位域的第8个字节从第x位到第7位是空缺，字节数按第一个第二个开始数，位数按7—0数（即最低位是第0位，最高位是第7位）
#define	RTS_FRAME_NULL				(4+6+16+4-24)		//6
//通信制式和通信参数
#define COMMUNICATION_MODE_NUM1		0			// 1#通信制式 mfsk
#define COMMUNICATION_MODE_NUM5		4			// 5#通信制式 FH
#define COMMUNICATION_MODE_NUM6		5			// 6#通信制式 mt_mfsk

// 控制帧公共域
typedef struct
{
    unsigned int fc_frame_type : 5;
    unsigned int fc_src_addr : 6;
    unsigned int fc_dst_addr : 6;
    unsigned int fc_node_energy : 3;
} frame_common;

// RTS帧专用域
typedef	struct
{
    frame_common 	rf_fc;
    unsigned int 		rf_seqno : 4;		// 发送序列号
    unsigned int 		rf_data_frame_num : 6;		// 数据帧数
    unsigned int 		rf_data_bitvalid : 16;	// 数据有效比特数
    unsigned int 		rf_data_commun_mode : 4;		// 通信制式
    unsigned int 		rf_data_commun_param : 12;    // 通信参数
    unsigned int 		rf_ack_lock : 1;     //回应锁，当为1时不回应ack
    unsigned int 		rf_reserve : 6;
} rts_frame;

/* ------------------------------------ 局部函数声明 ----------------------------------------- */
static void phy_MPSK_init(unsigned int mode);
static int phy_MPSK_process(unsigned char* phy_frame);
static unsigned char is_crc16_right(unsigned char* pdata, unsigned int nlength);
static void mpsk_rsdecode(unsigned char *pdata_in, unsigned int all_frame_num, unsigned int data_frame_num, unsigned char err_frame_pos[], unsigned char err_frame_num, unsigned char *pdata_out);
static void link_phy_frame_2_control_frame(const unsigned char *phy_frame, unsigned x, unsigned char *control_frame);
static void link_phy_frame_2_frame_common(const unsigned char *phy_frame, unsigned char *conotrol_frame_commn);

/* ------------------------------------ 局部变量定义 ----------------------------------------- */
static unsigned short phy_comm_mode_mask = 0x01; // 通信制式发射屏蔽位
//MPSK解调
static char MPSK_Frame_Data[242];
static unsigned char MPSK_DemData[240 * 42];
static unsigned char MPSK_DemData1[240 * 42];
static unsigned int MPSK_DemDataIndex = 0;
static unsigned char MPSK_DemDFrameNum = 0;
static unsigned char MPSK_err_frame[42];
static unsigned char MPSK_err_index = 0;
static unsigned char MPSK_err_num = 0;

static	rts_frame			rts_plus;
static unsigned int pkt_len;			//!< 接收数据长度，以字节为单位								物理层设置
static frame_common	control_frame_common;
//! 模式1短同步信号解调译码初始化，该函数在系统初始化时调用一次即可
//! 该函数内部调用MFSK_DemReset
void Mode1_Init(void)
{
    MFSK_DemInit(0);
}
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
int Mode1_Decode(unsigned int phy_signal_system, unsigned int phy_mfsk_symbol_bb, unsigned int phy_mfsk_symble_num, float* src, unsigned short group_num, unsigned char * PHY_Out_Buf, unsigned int* pbit_len, unsigned char* plink_src, unsigned char* plink_seqo)
{
    MFSK_DEM_STATE ret;
    unsigned int temp = 0;
    unsigned char comm_mode = 0;
    unsigned char is_data = 0;
    unsigned short addata_read_index = 0; //基础接收解调读指针index
    int decode_state = 0;//控制帧译码失败

    MFSK_SetMFSKSymbolBB(phy_mfsk_symbol_bb);
    MFSK_SetMFSKSignalSystem(phy_signal_system);

    //设置控制帧解调参数
    MFSK_DemSetFrameNum(1); 		// 控制帧帧数为1
    MFSK_DemSetDecodeMode(1);	// 控制帧为重复编码
    MFSK_DemSetSymbolNum(256 * 12 / phy_mfsk_symbol_bb);
    MFSK_DemSetCyclePrefixLen((unsigned int)CONTROL_FRAME_CYCLE_PREFIX(phy_signal_system, phy_mfsk_symbol_bb)); 	// 控制帧循环前缀长度，8kHz和10kHz时为20ms，24kHz时为10ms
#ifdef LDPC_MOD 			//!< 是否支持LDPC通信制式
    if ((phy_comm_mode_mask & 0x0020) == 0x0020)//MT_MFSK通信制式打开
	{
		comm_mode = 2;
	}
#endif
    // -----------------------------------------------------------------------------------------------------------------------------
    while (addata_read_index < group_num)
    {
        if (is_data == 0)
        {
            ret = MFSK_DemRoll(comm_mode, is_data, (char*)PHY_Out_Buf, &pkt_len, src + 256 * 2 * addata_read_index);
            addata_read_index++;
            if (ret == MFSK_DEM_COMPLETE)
            {
                //解析控制帧
                if (!is_crc16_right(PHY_Out_Buf, pkt_len))//校验错误，停止译码
                {
                    break;
                }
                else//校验正确
                {
                    decode_state = 1;//控制帧译码成功
                    link_phy_frame_2_frame_common(PHY_Out_Buf, (unsigned char *)&control_frame_common);
                    if (control_frame_common.fc_frame_type != RTS_PLUS)// 继续接收RTS+后续数据
                    {
                        break;
                    }
                    link_phy_frame_2_control_frame(PHY_Out_Buf, RTS_FRAME_NULL, (unsigned char *)&rts_plus);
                    is_data = 1;
                    //设置数据帧解调参数
                    if (rts_plus.rf_data_commun_mode == 0) //  通信制式1
                    {
                        // 通信制式1
                        // 根据接收参数设定接收帧数和接收参数
                        MFSK_DemSetFrameNum(rts_plus.rf_data_frame_num);
                        MFSK_DemSetDecodeMode((rts_plus.rf_data_commun_param >> 5) & 0x000000001);
                        MFSK_DemSetSymbolNum(phy_mfsk_symble_num);
                        temp = (rts_plus.rf_data_commun_param & 0x1f) | ((rts_plus.rf_data_commun_param & 0x1c0) >> 1);
                        MFSK_DemSetCyclePrefixLen(temp); // 设置循环前缀长度,changed by zly 2017_11_08循环前缀扩展成8位
                        MFSK_DemSetRecvState(MFSK_DEM_FRAME); // 设置接收状态，继续接收数据帧
                        comm_mode = 0;
                    }
                    else if (rts_plus.rf_data_commun_mode == 1)// 通信制式2, mpsk
                    {
                        phy_MPSK_init(rts_plus.rf_data_commun_mode);
                        MFSK_DemSetFrameNum(rts_plus.rf_data_frame_num);	// 设置数据帧帧数
#ifdef MPSK_NARROW_BAND	//!< mpsk带宽减为中心频率的1/4
                        MXSK_DemSetDataSampleNum(8544);
#else
                        MXSK_DemSetDataSampleNum(4272);
#endif
                        MFSK_DemSetRecvState(MFSK_DEM_FRAME); // 设置接收状态，继续接收数据帧
                        comm_mode = 1;
                    }
#ifdef LDPC_MOD 			//!< 是否支持LDPC通信制式
                        else if (rts_plus.rf_data_commun_mode == 5)
					{
						// 通信制式6
						// 根据接收参数设定接收帧数和接收参数
						MFSK_DemSetFrameNum(rts_plus.rf_data_frame_num);
						MFSK_DemSetSymbolNum(phy_mfsk_symble_num);
						temp = (rts_plus.rf_data_commun_param & 0x1f) | ((rts_plus.rf_data_commun_param & 0x1c0) >> 1);
						MFSK_DemSetCyclePrefixLen(temp); // 设置循环前缀长度,changed by zly 2017_11_08循环前缀扩展成8位
						MFSK_DemSetRecvState(MFSK_DEM_FRAME); // 设置接收状态，继续接收数据帧
						comm_mode = 2;
					}
#endif
                    else
                    {
                        break;
                    }
                }
            }
        }
        else//is_data == 1
        {
            if (comm_mode == 1)
            {
                ret = MFSK_DemRoll(comm_mode, is_data, MPSK_Frame_Data, &pkt_len, src + 256 * 2 * addata_read_index);
            }
            else
            {
                ret = MFSK_DemRoll(comm_mode, is_data, (char*)PHY_Out_Buf, &pkt_len, src + 256 * 2 * addata_read_index);
            }

            addata_read_index++;
            if (comm_mode == 1 && ret == MXSK_FRAME_READY)
            {
                if (phy_MPSK_process(PHY_Out_Buf) == 1)
                {
                    ret = MFSK_DEM_COMPLETE;
                }
            }
            if (ret == MFSK_DEM_COMPLETE)
            {
                //校验正确
                if (is_crc16_right(PHY_Out_Buf, pkt_len))
                {
                    LOG1("链路层：收到节点%02d的ALOHA数据包，校验正确，进入空闲状态！", control_frame_common.fc_src_addr);
                    decode_state = 2;//在控制帧译码成功的基础上，数据帧也译码成功
                    *pbit_len = rts_plus.rf_data_bitvalid;
                    *plink_src = control_frame_common.fc_src_addr;
                    *plink_seqo = rts_plus.rf_seqno;
                }
                    //校验错误
                else
                {
                    LOG1("链路层：收到节点%02d的ALOHA数据包，校验错误，进入空闲状态！", control_frame_common.fc_src_addr);
                }
                break;//结束while
            }
        }
    }//while end
    MFSK_DemReset(1);
    return decode_state;
}
//! 模式1长同步信号解调译码初始化，该函数在系统初始化时调用一次即可
//! 该函数内部调用MFSK_DemReset
void Mode1_Long_Init(void)
{
    MFSK_Long_DemInit(0);
}
//! 模式1长同步信号解调译码
//! \param[in] phy_signal_system 信号中心频率
//! \param[in] phy_mfsk_symbol_bb 基带符号长度，需要与发端一致，支持256、512、1024三种
//! \param[in] phy_mfsk_symble_num 每帧符号数，需要与发端一致
//! \param[in] src 输入基带数据地址，实部虚部交替出现，最大值归一到1
//! \param[in] group_num 每512个复数点为一组，输入数据组数
//! \param[out] PHY_Out_Buf 解调输出数据包，数组长度1414字节
//! \param[out] bit_len 数据包有效比特数
//! \param[out] link_src 数据包源地址，6bit有效位
//! \param[out] link_seqo 数据包序列号，4bit有效位
//如果相邻时间收到的link_src和link_seqo都相同，可以判定是重复译码成功数据包
//! return 0表示控制帧译码失败，1表示控制帧译码成功，2表示在控制帧译码成功的基础上，数据帧也译码成功
int Mode1_Long_Decode(unsigned int phy_signal_system, unsigned int phy_mfsk_symbol_bb, unsigned int phy_mfsk_symble_num, float* src, unsigned short group_num, unsigned char * PHY_Out_Buf, unsigned int* pbit_len, unsigned char* plink_src, unsigned char* plink_seqo)
{
    MFSK_LONG_DEM_STATE ret;
    unsigned int temp = 0;
    unsigned char comm_mode = 0;
    unsigned char is_data = 0;
    unsigned short addata_read_index = 0; //基础接收解调读指针index
    int decode_state = 0;//控制帧译码失败

    MFSK_Long_SetMFSKSymbolBB(phy_mfsk_symbol_bb);
    MFSK_Long_SetMFSKSignalSystem(phy_signal_system);

    //设置控制帧解调参数
    MFSK_Long_DemSetFrameNum(1); 		// 控制帧帧数为1
    MFSK_Long_DemSetDecodeMode(1);	// 控制帧为重复编码
    MFSK_Long_DemSetSymbolNum(256 * 12 / phy_mfsk_symbol_bb);
    MFSK_Long_DemSetCyclePrefixLen((unsigned int)CONTROL_FRAME_CYCLE_PREFIX(phy_signal_system, phy_mfsk_symbol_bb)); 	// 控制帧循环前缀长度，8kHz和10kHz时为20ms，24kHz时为10ms
#ifdef LDPC_MOD 			//!< 是否支持LDPC通信制式
    if ((phy_comm_mode_mask & 0x0020) == 0x0020)//MT_MFSK通信制式打开
	{
		comm_mode = 2;
	}
#endif
    // -----------------------------------------------------------------------------------------------------------------------------
    while (addata_read_index < group_num)
    {
        if (is_data == 0)
        {
            ret = MFSK_Long_DemRoll(comm_mode, is_data, (char*)PHY_Out_Buf, &pkt_len, src + 512 * 2 * addata_read_index);
            addata_read_index++;
            if (ret == MFSK_LONG_DEM_COMPLETE)
            {
                //解析控制帧
                if (!is_crc16_right(PHY_Out_Buf, pkt_len))//校验错误，停止译码
                {
                    break;
                }
                else//校验正确
                {
                    decode_state = 1;//控制帧译码成功
                    link_phy_frame_2_frame_common(PHY_Out_Buf, (unsigned char *)&control_frame_common);
                    if (control_frame_common.fc_frame_type != RTS_PLUS)// 继续接收RTS+后续数据
                    {
                        break;
                    }
                    link_phy_frame_2_control_frame(PHY_Out_Buf, RTS_FRAME_NULL, (unsigned char *)&rts_plus);
                    is_data = 1;
                    //设置数据帧解调参数
                    if (rts_plus.rf_data_commun_mode == 0) //  通信制式1
                    {
                        // 通信制式1
                        // 根据接收参数设定接收帧数和接收参数
                        MFSK_Long_DemSetFrameNum(rts_plus.rf_data_frame_num);
                        MFSK_Long_DemSetDecodeMode((rts_plus.rf_data_commun_param >> 5) & 0x000000001);
                        MFSK_Long_DemSetSymbolNum(phy_mfsk_symble_num);
                        temp = (rts_plus.rf_data_commun_param & 0x1f) | ((rts_plus.rf_data_commun_param & 0x1c0) >> 1);
                        MFSK_Long_DemSetCyclePrefixLen(temp); // 设置循环前缀长度,changed by zly 2017_11_08循环前缀扩展成8位
                        MFSK_Long_DemSetRecvState(MFSK_LONG_DEM_FRAME); // 设置接收状态，继续接收数据帧
                        comm_mode = 0;
                    }
                    else if (rts_plus.rf_data_commun_mode == 1)// 通信制式2, mpsk
                    {
                        phy_MPSK_init(rts_plus.rf_data_commun_mode);
                        MFSK_Long_DemSetFrameNum(rts_plus.rf_data_frame_num);	// 设置数据帧帧数
#ifdef MPSK_NARROW_BAND	//!< mpsk带宽减为中心频率的1/4
                        MXSK_Long_DemSetDataSampleNum(8544);
#else
                        MXSK_Long_DemSetDataSampleNum(4272);
#endif
                        MFSK_Long_DemSetRecvState(MFSK_LONG_DEM_FRAME); // 设置接收状态，继续接收数据帧
                        comm_mode = 1;
                    }
#ifdef LDPC_MOD 			//!< 是否支持LDPC通信制式
                        else if (rts_plus.rf_data_commun_mode == 5)
					{
						// 通信制式6
						// 根据接收参数设定接收帧数和接收参数
						MFSK_Long_DemSetFrameNum(rts_plus.rf_data_frame_num);
						MFSK_Long_DemSetSymbolNum(phy_mfsk_symble_num);
						temp = (rts_plus.rf_data_commun_param & 0x1f) | ((rts_plus.rf_data_commun_param & 0x1c0) >> 1);
						MFSK_Long_DemSetCyclePrefixLen(temp); // 设置循环前缀长度,changed by zly 2017_11_08循环前缀扩展成8位
						MFSK_Long_DemSetRecvState(MFSK_LONG_DEM_FRAME); // 设置接收状态，继续接收数据帧
						comm_mode = 2;
					}
#endif
                    else
                    {
                        break;
                    }
                }
            }
        }
        else//is_data == 1
        {
            if (comm_mode == 1)
            {
                ret = MFSK_Long_DemRoll(comm_mode, is_data, MPSK_Frame_Data, &pkt_len, src + 512 * 2 * addata_read_index);
            }
            else
            {
                ret = MFSK_Long_DemRoll(comm_mode, is_data, (char*)PHY_Out_Buf, &pkt_len, src + 512 * 2 * addata_read_index);
            }

            addata_read_index++;
            if (comm_mode == 1 && ret == MXSK_LONG_FRAME_READY)
            {
                if (phy_MPSK_process(PHY_Out_Buf) == 1)
                {
                    ret = MFSK_LONG_DEM_COMPLETE;
                }
            }
            if (ret == MFSK_LONG_DEM_COMPLETE)
            {
                //校验正确
                if (is_crc16_right(PHY_Out_Buf, pkt_len))
                {
                    LOG1("链路层：收到节点%02d的ALOHA数据包，校验正确，进入空闲状态！", control_frame_common.fc_src_addr);
                    decode_state = 2;//在控制帧译码成功的基础上，数据帧也译码成功
                    *pbit_len = rts_plus.rf_data_bitvalid;
                    *plink_src = control_frame_common.fc_src_addr;
                    *plink_seqo = rts_plus.rf_seqno;
                }
                    //校验错误
                else
                {
                    LOG1("链路层：收到节点%02d的ALOHA数据包，校验错误，进入空闲状态！", control_frame_common.fc_src_addr);
                }
                break;//结束while
            }
        }
    }//while end
    MFSK_Long_DemReset(1);
    return decode_state;
}
//! 模式2解调译码初始化
void Mode2_Init(void)
{
    FH_DeModulate_Init();
}
//! 模式2解调译码，该函数在系统初始化时调用一次即可
//! \param[in] src 输入基带数据地址，实部虚部交替出现，最大值归一到1
//! \param[in] group_num 每2048个复数点为一组，输入数据组数
//! \param[out] out 解调输出数据,长度17字节
//! return 0表示译码失败，1表示译码成功
int Mode2_Decode(float* src, unsigned short group_num, char* out)
{
    int ret;
    ret = FH_SyncDecode(src, group_num, out);
    return ret;
}
void UdpNet_Task(void)
{
    Hrt_UdpNet_Task();
}
static void phy_MPSK_init(unsigned int mode)
{
    MPSK_DemDFrameNum = 0;
    MPSK_DemDataIndex = 0;

    memset(MPSK_err_frame, 0, sizeof(MPSK_err_frame));
    MPSK_err_index = 0;
    MPSK_err_num = 0;

#ifdef DEBUG_PHY
    LOG1("物理层(mpsk)：通信制式%d，MPSK解调初始化！", mode + 1);
#endif
}

static int phy_MPSK_process(unsigned char* phy_frame)
{
    unsigned int len = 0;
    short temp1;
    short temp2 = 0;

    MPSK_DemDFrameNum++;
    // 检查帧校验
    len = 240; // 实际数据长度，后两个字节为校验字节
    temp1 = MPSK_Frame_Data[len + 1] & 0x00ff;
    temp1 = (temp1 << 8) + (MPSK_Frame_Data[len] & 0x00ff);
    temp2 = Endecode_Crc16(MPSK_Frame_Data, len);
    if (temp1 == temp2) // 校验正确
    {
#ifdef DEBUG_PHY
        ENetShell_DebugInfo("物理层(mpsk)：第%02d帧！ %d", MPSK_DemDFrameNum, MPSK_Frame_Data[1]);
#endif
    }
    else
    {
#ifdef DEBUG_PHY
        ENetShell_DebugInfo("物理层(mpsk)：第%02d帧校验错误！ %d", MPSK_DemDFrameNum, MPSK_Frame_Data[1]);
#endif

        MPSK_err_frame[MPSK_err_index++] = MPSK_DemDFrameNum - 1;
        MPSK_err_num++;
    }

    // 保存帧数据
    memcpy(MPSK_DemData + MPSK_DemDataIndex, MPSK_Frame_Data, len);
    MPSK_DemDataIndex += len;
    if (MPSK_DemDFrameNum == rts_plus.rf_data_frame_num)
    {
        // rs解码
        // 冗余帧数
        if (rts_plus.rf_data_frame_num <= 8)
        {
            temp1 = rts_plus.rf_data_frame_num / 2;
        }
        else if (rts_plus.rf_data_frame_num > 8 && rts_plus.rf_data_frame_num <= 24)
        {
            temp1 = 4;
        }
        else if (rts_plus.rf_data_frame_num > 24 && rts_plus.rf_data_frame_num <= 96)
        {
            temp1 = rts_plus.rf_data_frame_num - (rts_plus.rf_data_frame_num * 10 / 12);
        }
        else
        {
            temp1 = 16;
        }

        // 解RS码
        mpsk_rsdecode(MPSK_DemData, rts_plus.rf_data_frame_num, rts_plus.rf_data_frame_num - temp1, MPSK_err_frame, MPSK_err_num, MPSK_DemData1);
        pkt_len = 240 * (rts_plus.rf_data_frame_num - temp1);
        memcpy(phy_frame, MPSK_DemData1, pkt_len);

        temp1 = MPSK_DemData1[pkt_len - 1] & 0x00ff;
        temp1 = (temp1 << 8) + (MPSK_DemData1[pkt_len - 2] & 0x00ff);
        temp2 = Endecode_Crc16((char*)MPSK_DemData1, pkt_len - 2);

        if (temp1 == temp2) // 校验正确
        {
#ifdef DEBUG_PHY
            LOG3("物理层(mpsk)：错误率%d/%d，总校验正确！ %d", MPSK_err_num, rts_plus.rf_data_frame_num, pkt_len);
#endif
        }
        else
        {
#ifdef DEBUG_PHY
            LOG3("物理层(mpsk)：错误率%d/%d，总校验错误！ %d", MPSK_err_num, rts_plus.rf_data_frame_num, pkt_len);
#endif
        }
        MPSK_DemDFrameNum = 0;
        MPSK_DemDataIndex = 0;
        return 1;

    }
    return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
// rs编解码 by wyb
// 一帧包含字节数
#define PARA_FRAMESIZE	240
// 伽罗华域阶数
#define PARA_M	8

// RS母码码字长度
#define PARA_N	255

// RS母码输入长度
#define PARA_K	239

// RS码校验码元长度最大值
#define PARA_N_K	16

// RS删除码码字长度最大值
#define PARA_N1_MAX	51//changed by zly 2015_12_30

// RS删除码信息码元长度最大值
#define PARA_K1_MAX	35//changed by zly 2015_12_30


// 用统一的母码，校验帧数最大为16，实际校验帧小于16，则删除结尾的校验帧
unsigned char g[] = { 1, 118, 52, 103, 31, 104, 126, 187, 232, 17, 56, 183, 49, 100, 81, 44, 79 };//RS(255,239)

//x=gf(1:255,8);fprintf('%d,',[0,log(x)]);
unsigned char LOG[] = { 0, 0, 1, 25, 2, 50, 26, 198, 3, 223, 51, 238, 27, 104, 199, 75, 4, 100, 224, 14, 52, 141, 239, 129, 28, 193, 105, 248, 200, 8, 76, 113, 5, 138, 101, 47, 225, 36, 15, 33, 53, 147, 142, 218, 240, 18, 130, 69, 29, 181, 194, 125, 106, 39, 249, 185, 201, 154, 9, 120, 77, 228, 114, 166, 6, 191, 139, 98, 102, 221, 48, 253, 226, 152, 37, 179, 16, 145, 34, 136, 54, 208, 148, 206, 143, 150, 219, 189, 241, 210, 19, 92, 131, 56, 70, 64, 30, 66, 182, 163, 195, 72, 126, 110, 107, 58, 40, 84, 250, 133, 186, 61, 202, 94, 155, 159, 10, 21, 121, 43, 78, 212, 229, 172, 115, 243, 167, 87, 7, 112, 192, 247, 140, 128, 99, 13, 103, 74, 222, 237, 49, 197, 254, 24, 227, 165, 153, 119, 38, 184, 180, 124, 17, 68, 146, 217, 35, 32, 137, 46, 55, 63, 209, 91, 149, 188, 207, 205, 144, 135, 151, 178, 220, 252, 190, 97, 242, 86, 211, 171, 20, 42, 93, 158, 132, 60, 57, 83, 71, 109, 65, 162, 31, 45, 67, 216, 183, 123, 164, 118, 196, 23, 73, 236, 127, 12, 111, 246, 108, 161, 59, 82, 41, 157, 85, 170, 251, 96, 134, 177, 187, 204, 62, 90, 203, 89, 95, 176, 156, 169, 160, 81, 11, 245, 22, 235, 122, 117, 44, 215, 79, 174, 213, 233, 230, 231, 173, 232, 116, 214, 244, 234, 168, 80, 88, 175 };

//x=gf(2,8).^(0:254);fprintf('%d,', x.x );
unsigned char EXP[] = { 1, 2, 4, 8, 16, 32, 64, 128, 29, 58, 116, 232, 205, 135, 19, 38, 76, 152, 45, 90, 180, 117, 234, 201, 143, 3, 6, 12, 24, 48, 96, 192, 157, 39, 78, 156, 37, 74, 148, 53, 106, 212, 181, 119, 238, 193, 159, 35, 70, 140, 5, 10, 20, 40, 80, 160, 93, 186, 105, 210, 185, 111, 222, 161, 95, 190, 97, 194, 153, 47, 94, 188, 101, 202, 137, 15, 30, 60, 120, 240, 253, 231, 211, 187, 107, 214, 177, 127, 254, 225, 223, 163, 91, 182, 113, 226, 217, 175, 67, 134, 17, 34, 68, 136, 13, 26, 52, 104, 208, 189, 103, 206, 129, 31, 62, 124, 248, 237, 199, 147, 59, 118, 236, 197, 151, 51, 102, 204, 133, 23, 46, 92, 184, 109, 218, 169, 79, 158, 33, 66, 132, 21, 42, 84, 168, 77, 154, 41, 82, 164, 85, 170, 73, 146, 57, 114, 228, 213, 183, 115, 230, 209, 191, 99, 198, 145, 63, 126, 252, 229, 215, 179, 123, 246, 241, 255, 227, 219, 171, 75, 150, 49, 98, 196, 149, 55, 110, 220, 165, 87, 174, 65, 130, 25, 50, 100, 200, 141, 7, 14, 28, 56, 112, 224, 221, 167, 83, 166, 81, 162, 89, 178, 121, 242, 249, 239, 195, 155, 43, 86, 172, 69, 138, 9, 18, 36, 72, 144, 61, 122, 244, 245, 247, 243, 251, 235, 203, 139, 11, 22, 44, 88, 176, 125, 250, 233, 207, 131, 27, 54, 108, 216, 173, 71, 142 };

unsigned char gf_mul(unsigned char in1, unsigned char in2)
{
    return (in1 == 0 || in2 == 0) ? 0 : EXP[((int)LOG[in1] + LOG[in2]) % PARA_N];
}


unsigned char a[PARA_N_K];
unsigned char a0[PARA_N_K];
unsigned char sigma[PARA_N_K];
unsigned char prod_a_ii[PARA_N_K];

void get_mat(unsigned char Ainv[], unsigned char err_pos[], int num_err, int para_n1, int para_k1)
{// 计算范德蒙矩阵（代表错误位置）的逆，目前矩阵为16x16
    int i;
    int j;
    int k;
    int temp;	int prod_a_ii;
    for (i = 0; i<num_err; i++)
    {
        temp = PARA_N_K + para_k1 - err_pos[i] - 1;
        temp = temp%PARA_N;
        a[i] = EXP[temp];
    }
    for (i = 0; i<num_err; i++)
    {
        memcpy(a0, a, num_err);
        a0[i] = 0;
        memset(sigma, 0, num_err);
        sigma[0] = 1;
        for (j = 0; j<num_err; j++)
        {
            for (k = num_err - 1; k >= 1; k--)
            {
                sigma[k] ^= gf_mul(sigma[k - 1], a0[j]);
            }

        }
        prod_a_ii = 0;
        for (j = 0; j<num_err; j++)
        {
            prod_a_ii += LOG[a[i] ^ a0[j]];
        }
        prod_a_ii %= PARA_N;

        for (j = 0; j<num_err; j++)
        {
            if (sigma[num_err - 1 - j] == 0)
            {
                Ainv[i + j*(num_err)] = 0;
            }
            else
            {
                temp = LOG[sigma[num_err - 1 - j]] - prod_a_ii;
                temp = temp<0 ? (temp + PARA_N) : temp;
                Ainv[i + j*(num_err)] = EXP[temp];
            }
        }
    }
    return;
}


unsigned char  gf_sub(unsigned char rx[], unsigned char power, int len)
{// 求解伴随式所用到的元素带入多项式过程
    int i;
    int temp;
    int y;
    y = 0;
    for (i = 0; i<len; i++)
    {
        if (rx[i] != 0)
        {
            temp = power*(len - 1 - i);
            temp = LOG[rx[i]] + temp;
            temp %= PARA_N;
            y ^= EXP[temp];
        }
    }
    return y;
}


void  gf_mat_mult(unsigned char A[], unsigned char x[], unsigned char y[], int num_err)
{//矩阵与向量的乘积
    int i; int j;

    for (i = 0; i<num_err; i++)
    {
        y[i] = 0;
        for (j = 0; j<num_err; j++)
        {
            y[i] ^= gf_mul(A[i + j*num_err], x[j]);
        }
    }
}


unsigned char Ainv[PARA_N_K*PARA_N_K] = { 0 };
unsigned char s[PARA_N_K];
unsigned char d0[PARA_N1_MAX];
unsigned char err_value[PARA_N_K];
unsigned char err_pos1[PARA_N_K] = { 0 };
int num_err1;
void rs_dec(unsigned char rx[], unsigned char decode[], unsigned char err_pos[], int num_err, int para_n1, int para_k1)
{
    int i;
    int j;
    char error[] = "too many error postions!\n";
    if (num_err>para_n1 - para_k1)
    {
        memcpy(decode, error, sizeof(error));
        return;
    }
    for (i = 0; i<num_err; i++)
    {
        err_pos1[i] = err_pos[i];
    }

    for (i = 0; i<PARA_N_K - para_n1 + para_k1; i++)
    {//把尾部发送时在母码基础上丢弃的帧，也认为是错位帧
        err_pos1[i + num_err] = para_n1 + i;
    }
    num_err1 = num_err + PARA_N_K - para_n1 + para_k1;
    get_mat(Ainv, err_pos1, num_err1, para_n1, para_k1);//得到逆矩阵
    for (i = 0; i<PARA_FRAMESIZE; i++)
    {
        for (j = 0; j<para_n1; j++)//获得接收码字
        {
            d0[j] = rx[j*PARA_FRAMESIZE + i];
        }

        for (j = 0; j<num_err1; j++)//丢失的位置填零
        {
            d0[err_pos1[j]] = 0;
        }
        for (j = 0; j<num_err1; j++)//伴随式计算
        {
            s[j] = gf_sub(d0, (unsigned char)(j + 1), para_k1 + PARA_N_K);
        }
        gf_mat_mult(Ainv, s, err_value, num_err1);//逆矩阵乘以伴随式，得到错误图案
        for (j = 0; j<num_err1; j++)
        {
            d0[err_pos1[j]] = err_value[j];//纠正错误
        }
        for (j = 0; j<para_n1; j++)
        {
            decode[j*PARA_FRAMESIZE + i] = d0[j];
        }

    }
}
//! RS译码
//!	\param[in] pdata_in 译码前字节数据指针
//!	\param[in] all_frame_num 总帧数（含冗余帧）
//!	\param[in] data_frame_num 数据帧数（不含冗余帧）
//!	\param[in] err_frame_pos 错误帧位置索引
//!	\param[in] err_frame_num 错误帧数
//!	\param[out] pdata_out 译码后字节数据指针
static void mpsk_rsdecode(unsigned char *pdata_in, unsigned int all_frame_num, unsigned int data_frame_num, unsigned char err_frame_pos[], unsigned char err_frame_num, unsigned char *pdata_out)
{
    if (err_frame_num == 0)
    {
        memcpy(pdata_out, pdata_in, data_frame_num * 240);
    }
    else
    {
        rs_dec(pdata_in, pdata_out, err_frame_pos, err_frame_num, all_frame_num, data_frame_num);
    }
}

static void link_phy_frame_2_control_frame(const unsigned char *phy_frame, unsigned x, unsigned char *control_frame)
{
    int i;
    unsigned char buff_1[CONTROL_FRAME_LEN] = { 0 };
    unsigned char buff_2[CONTROL_FRAME_LEN] = { 0 };

    memcpy((void*)buff_1, (void*)phy_frame, CONTROL_FRAME_PHY_LEN);
    memcpy((void*)buff_2, (void*)phy_frame, CONTROL_FRAME_PHY_LEN);

    buff_2[2] = buff_1[2] & 0x0f;
    buff_2[3] = 0;
    for (i = 4; i < (CONTROL_FRAME_LEN); i++)
    {
        buff_2[i] = ((buff_1[i - 2] >> 4) | (buff_1[i - 1] << 4));
    }
    memcpy((void*)buff_1, (void*)buff_2, CONTROL_FRAME_LEN);

    buff_2[7] = (buff_1[7] & (0xff >> (8 - x)));

    for (i = 8; i < (CONTROL_FRAME_LEN - 1); i++)
    {
        buff_2[i] = ((buff_1[i - 1] >> x) | (buff_1[i] << (8 - x)));
    }
    memcpy((void*)control_frame, (void*)buff_2, CONTROL_FRAME_LEN);
}

static void link_phy_frame_2_frame_common(const unsigned char *phy_frame, unsigned char *conotrol_frame_common)
{
    memcpy((void*)conotrol_frame_common, (void*)phy_frame, CONTROL_FRAME_COMMON_LEN);
}

// 检查给定长度数据的16位CRC是否正确,nlenth包括crc
static unsigned char is_crc16_right(unsigned char* pdata, unsigned int nlength)
{
    short crc16_0;
    short crc16_1;

    if (nlength <= 2)
    {
        return 0;
    }

    crc16_0 = pdata[nlength - 1] << 8 | pdata[nlength - 2];
    crc16_1 = Endecode_Crc16((char *)pdata, nlength - 2); 	// 2012-07-05修正

    //	LOG3("%x, %x %d", crc16_0, crc16_1, nlength);
    return (crc16_0 == crc16_1);
}
