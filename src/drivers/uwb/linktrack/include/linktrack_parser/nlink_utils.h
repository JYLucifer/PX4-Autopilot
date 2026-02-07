/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef NLINK_UTILS_H
#define NLINK_UTILS_H
#ifdef __cplusplus
extern "C" {
#endif
#include "nlink_typedef.h"
/*
// 预编译指令，用于定义结构体打包对齐方式
#ifndef _MSC_VER
    // 非 Windows 环境（如 Linux/GCC）使用 __attribute__((packed)) 确保结构体紧凑排列，不对齐
    #define #pragma pack(1)
    __Declaration__) __Declaration__ __attribute__((packed))
#else
    // Windows 环境（如 MSVC）使用 #pragma pack 指令
    #define #pragma pack(1)
    __Declaration__) \
        __pragma(pack(push, 1)) __Declaration__ __pragma(pack(pop))
#endif
*/

/**
 * 计算静态数组的元素个数
 * @param X: 数组名
 * @return: 数组的元素数量
 */
#define ARRAY_LENGTH(X) (sizeof(X) / sizeof(X[0]))

/**
 * 从 Netlink 协议消息中提取长度字段
 * 假设长度信息存储在数据包的第2和第3个字节（索引2和3），并以小端序存储
 * @param X: 指向消息数据的指针
 * @return: 计算出的消息长度
 */
#define NLINK_PROTOCOL_LENGTH(X) ((size_t)(X[2] | X[3] << 8))

/**
 * 数组转换宏：将源数组 SRC 的每个元素除以 MULTIPLY 后存入目标数组 DEST
 * 用于数据标度转换，例如将原始传感器读数转换为工程单位
 */
#define NLINK_TRANSFORM_ARRAY(DEST, SRC, MULTIPLY)                             \
  for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT) {         \
    DEST[_CNT] = SRC[_CNT] / MULTIPLY;                                         \
  }

/**
 * 专门用于处理 24 位整型数组的转换宏
 * 先解析 24 位整数，再进行标度转换
 */
#define NLINK_TRANSFORM_ARRAY_INT24(DEST, SRC, MULTIPLY)                       \
  for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT) {         \
    DEST[_CNT] = NLINK_ParseInt24(SRC[_CNT]) / MULTIPLY;                       \
  }

/**
 * 安全分配新节点内存的宏
 * 如果节点指针为空，则分配内存并初始化为零
 * 如果分配失败，打印错误信息并返回0
 */
#define TRY_MALLOC_NEW_NODE(NODE_POINTER, NODE_TYPE)                           \
  if (!NODE_POINTER) {                                                         \
    void *p = malloc(sizeof(NODE_TYPE));                                       \
    if (p != NULL) {                                                           \
      NODE_POINTER = (NODE_TYPE *)p;                                           \
      memset(p, 0, sizeof(NODE_TYPE));                                         \
    } else {                                                                   \
      printf("Memory allocation failed, please increase heap size to support " \
             "protocol unpack.\r\n");                                          \
      return 0;                                                                \
    }                                                                          \
  }

// 告诉编译器按1字节对齐（紧凑排列，无填充字节），确保结构体布局与网络字节流完全一致
#pragma pack(1)

/**
 * 24位有符号整数类型
 * 用于处理嵌入式设备或网络协议中常见的24位数据格式
 */
typedef struct {
  uint8_t byteArray[3];// 3个字节存储24位数据
} nint24_t;

/**
 * 24位无符号整数类型
 */
typedef struct {
  uint8_t byteArray[3];
} nuint24_t;
#pragma pack()// 恢复默认的对齐方式

/**
 * 解析24位有符号整数
 * @param data: 包含3个字节的nint24_t结构体
 * @return: 解析后的32位有符号整数
 */
int32_t NLINK_ParseInt24(nint24_t data);

/**
 * 解析24位无符号整数
 * @param data: 包含3个字节的nuint24_t结构体
 * @return: 解析后的32位无符号整数
 */
uint32_t NLINK_ParseUint24(nuint24_t data);

/**
 * 验证数据的校验和
 * 使用简单的累加和校验算法验证数据完整性
 * @param data: 指向待验证数据的指针
 * @param data_length: 数据的总长度（包括校验和字节）
 * @return: 校验通过返回1，失败返回0
 */
uint8_t NLINK_VerifyCheckSum(const void *data, size_t data_length);

/**
 * 计算并更新数据帧的校验和
 * 计算前n-1个字节的累加和，结果存储在第n个字节
 * @param data: 指向数据缓冲区的指针
 * @param data_length: 数据的总长度（包括校验和字节）
 */
void NLink_UpdateCheckSum(uint8_t *data, size_t data_length);

/**
 * 将十六进制字符串转换为二进制数据
 * 例如："57 00 01" -> [0x57, 0x00, 0x01]
 * @param str: 输入的十六进制字符串
 * @param out: 输出缓冲区指针
 * @return: 成功转换的字节数
 */
size_t NLink_StringToHex(const char *str, uint8_t *out);

// 常用的标度转换乘数定义
#define MULTIPLY_VOLTAGE 1000.0f    // 电压转换系数（例如：mV -> V）
#define MULTIPLY_POS 1000.0f        // 位置转换系数
#define MULTIPLY_DIS 1000.0f        // 距离转换系数（例如：mm -> m）
#define MULTIPLY_VEL 10000.0f       // 速度转换系数
#define MULTIPLY_ANGLE 100.0f       // 角度转换系数
#define MULTIPLY_RSSI -2.0f         // 信号强度转换系数（负值可能用于反向指示）
#define MULTIPLY_EOP 100.0f         // 其他参数转换系数

#ifdef __cplusplus
}
#endif
#endif // NLINK_UTILS_H
