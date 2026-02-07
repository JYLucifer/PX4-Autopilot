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

#include "linktrack_parser/nlink_utils.h"

/**
 * 解析24位有符号整数。
 * 说明：TofSense等传感器协议为节省空间，常使用3字节（24位）表示数据。
 *       此函数将3字节的原始数据转换为标准的32位有符号整数。

 * @param data: 包含3个字节的nint24_t结构体，代表一个24位有符号整数。
 * @return int32_t: 解析后的32位有符号整数。
 */
int32_t NLINK_ParseInt24(nint24_t data) {
  // 获取指向data内存起始位置的字节指针，便于按字节操作
  uint8_t *byte = (uint8_t *)(&data);
  // 将3个字节重新组合成32位整数，并除以256进行缩放。
  // 操作解析：byte[0]左移8位（占据bit8~bit15），
  //          byte[1]左移16位（占据bit16~bit23），
  //          byte[2]左移24位（占据bit24~bit31，即符号位）。
  // 除以256（右移8位）的目的是将整数右移，使符号位回到正确位置（bit31），并调整数值范围。
  // 注意：此实现假设传感器和主机处理器使用相同的字节序（通常是小端模式）。
  return (int32_t)(byte[0] << 8 | byte[1] << 16 | byte[2] << 24) / 256;
}

/**
 * 解析24位无符号整数。

 * @param data: 包含3个字节的nuint24_t结构体，代表一个24位无符号整数。
 * @return uint32_t: 解析后的32位无符号整数。
 */
uint32_t NLINK_ParseUint24(nuint24_t data) {
  uint8_t *byte = (uint8_t *)(&data);
  return byte[0] | byte[1] << 8 | byte[2] << 16;
}

/**
 * 验证数据的校验和。
 * 说明：此函数使用简单的累加和校验法，用于检查数据在传输过程中是否出现错误。

 * @param data: 指向待验证数据的指针。
 * @param data_length: 数据的总长度（包括最后一个字节的校验和本身）。
 * @return uint8_t: 校验通过返回1，失败返回0。
 */
uint8_t NLINK_VerifyCheckSum(const void *data, size_t data_length) {
  // 将void*指针转换为uint8_t*指针，以便按字节遍历
  const uint8_t *byte = (uint8_t *)data;
  uint8_t sum = 0; // 初始化累加和为0

  // 遍历数据帧中除最后一个字节（即校验和字节）外的所有字节
  for (size_t i = 0; i < data_length - 1; ++i) {
    sum += byte[i];
  }

  // 比较计算得到的累加和与数据帧中最后一个字节（存储的校验和）是否相等
  return sum == byte[data_length - 1];
}

/**
 * 将十六进制字符串转换为二进制数据（字节数组）。
 * 说明：此函数常用于调试或测试，例如将协议文档中的示例字符串转换为二进制数据进行测试。

 * @param str: 输入的十六进制字符串（例如："57 00 01"或"570001"）。可包含空格。
 * @param out: 输出缓冲区指针，用于存储转换后的二进制数据。
 * @return size_t: 成功转换的字节数。
 */
size_t NLink_StringToHex(const char *str, uint8_t *out) {
  size_t outLength = 0; // 输出的字节数计数器
  size_t cnt = 0;       // 输入字符串的索引
  uint8_t high = 0, low = 0; // 临时存储一个字节的高4位和低4位
  uint8_t current = 0;   // 当前读取的字符
  uint8_t value = 0;     // 当前字符对应的十六进制数值（0-15）
  uint8_t isHighValid = 0; // 标志位，表示是否已读取到一个有效的高4位
  // 遍历输入字符串，直到遇到字符串结束符'\0'
  while ((current = str[cnt])) {
    ++cnt; // 移动到下一个字符

    // 判断当前字符属于哪个十六进制数字范围，并转换为数值
    if (current >= '0' && current <= '9') {
      value = (uint8_t)(current - '0');
    } else if (current >= 'a' && current <= 'f') {
      value = (uint8_t)(current - 'a' + 10);
    } else if (current >= 'A' && current <= 'F') {
      value = (uint8_t)(current - 'A' + 10);
    } else {
      // 如果不是十六进制字符（如空格、标点），则跳过，继续处理下一个字符
      continue;
    }

    // 处理数值：每两个十六进制数字组成一个字节（高4位 + 低4位）
    if (!isHighValid) {
      high = value;    // 第一个数字作为高4位
      isHighValid = 1; // 标记高4位已就绪
    } else {
      low = value;
      out[outLength] = (uint8_t)(high << 4 | low);
      ++outLength;
      isHighValid = 0;
    }
  }
  return outLength;
}

/**
 * 计算并更新数据帧的校验和。
 * 说明：此函数通常用于构建要发送的数据帧，计算校验和并填充到帧尾。

 * @param data: 指向数据缓冲区的指针，该缓冲区的最后一个字节应预留用于存放校验和。
 * @param data_length: 数据的总长度（包括最后一个字节的校验和位置）。
 */
void NLink_UpdateCheckSum(uint8_t *data, size_t data_length) {
  uint8_t sum = 0;
  // 计算从数据开始到倒数第二个字节的累加和
  for (size_t i = 0; i < data_length - 1; ++i) {
    sum += data[i];
  }
  // 将计算出的校验和存入数据的最后一个字节
  data[data_length - 1] = sum;
}
