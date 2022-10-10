cmake_minimum_required(VERSION 3.23)
project(Decode_A)

set(CMAKE_CXX_STANDARD 14)
# 指定lib目录
link_directories(${PROJECT_SOURCE_DIR}/Lib)
# 指定头文件搜索路径
include_directories(${PROJECT_SOURCE_DIR}/Header)

add_library(Decode_A Decode_A.cpp Header/stdafx.h Header/global.h Source/dsplib.cpp Header/dsplib.h Source/Net/enet_shell.cpp Header/Net/enet_shell.h Source/MPSK/mpsk_dem.cpp Header/MPSK/mpsk_dem.h Source/MFSK/mfsk_dem.cpp Header/MFSK/mfsk_dem.h Source/MFSK/mfsk_long_dem.cpp Header/MFSK/mfsk_long_dem.h Source/LDPC/q_ldpc.cpp Header/LDPC/q_ldpc.h Source/EnDecode/endecode.cpp Header/EnDecode/endecode.h Source/FH/fh_dem.cpp Header/FH/fh_dem.h Source/FH/FH_Related_Func.cpp Header/FH/FH_Related_Func.h Decode_A.h)