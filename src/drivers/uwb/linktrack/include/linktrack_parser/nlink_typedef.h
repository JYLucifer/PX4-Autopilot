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

#ifndef NLINK_TYPEDEF_H
#define NLINK_TYPEDEF_H
#ifdef __cplusplus
// 如果是在 C++ 环境中编译，使用 extern "C" 来确保括号内的代码按 C 语言的规则进行编译和链接
// 这主要用于解决 C++ 和 C 混合编程时的名称修饰（name mangling）问题
extern "C" {
#endif
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum {
  LINKTRACK_ROLE_NODE,      // 普通节点
  LINKTRACK_ROLE_ANCHOR,   // 锚点（通常为位置固定的参考点）
  LINKTRACK_ROLE_TAG,      // 标签（通常为需要被追踪的移动目标）
  LINKTRACK_ROLE_CONSOLE,  // 控制台
  LINKTRACK_ROLE_DT_MASTER, // 主设备（在某种特定通信协议中）
  LINKTRACK_ROLE_DT_SLAVE, // 从设备
  LINKTRACK_ROLE_MONITOR,  // 监控器
} linktrack_role_e;

typedef uint32_t nlink_id_t;

// 与文件开头的 #ifdef __cplusplus 配对，结束 extern "C" 块
#define MAX_ANCHOR_COUNT 16 /**< 系统最大可支持的锚点设备数量 */
#define MAX_TAG_COUNT 16    /**< 系统最大可支持的标签设备数量 */

#ifdef __cplusplus
}
#endif
#endif // NLINK_TYPEDEF_H
