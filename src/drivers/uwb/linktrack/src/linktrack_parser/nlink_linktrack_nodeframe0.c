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

#include "linktrack_parser/nlink_linktrack_nodeframe0.h"
#include <stdio.h>
#include "linktrack_parser/nlink_utils.h"

#pragma pack(1)
typedef struct {
  uint8_t header[2];
  uint16_t frame_length;
  uint8_t role;
  uint8_t id;
  uint8_t reserved[4];
  uint8_t valid_node_count;
  // nodes...
  // uint8_t checkSum;
} nlt_nodeframe0_raw_t;
#pragma pack()

static nlt_nodeframe0_raw_t g_frame;

static uint8_t UnpackData(const uint8_t *data, size_t data_length) {
  if (data_length < g_nlt_nodeframe0.fixed_part_size ||
      data[0] != g_nlt_nodeframe0.frame_header ||
      data[1] != g_nlt_nodeframe0.function_mark)
    return 0;

  size_t frame_length = NLINK_PROTOCOL_LENGTH(data);
  if (data_length < frame_length)
    return 0;
  if (!NLINK_VerifyCheckSum(data, frame_length))
    return 0;

  static uint8_t init_needed = 1;
  if (init_needed) {
    memset(g_nlt_nodeframe0.result.nodes, 0,
           sizeof(g_nlt_nodeframe0.result.nodes));
    init_needed = 0;
  }

  memcpy(&g_frame, data, g_nlt_nodeframe0.fixed_part_size);
  g_nlt_nodeframe0.result.role = g_frame.role;
  g_nlt_nodeframe0.result.id = g_frame.id;
  g_nlt_nodeframe0.result.valid_node_count = g_frame.valid_node_count;

  for (size_t i = 0, address = g_nlt_nodeframe0.fixed_part_size;
       i < g_frame.valid_node_count; ++i) {
    const uint8_t *begin = data + address;
    // ==================【修复】重命名局部变量避免遮蔽 ==================
    size_t node_data_length = (size_t)(begin[2] | begin[3] << 8);  // 将 data_length 改为 node_data_length
    size_t current_node_size = 4 + node_data_length;  // 同步修改这里
    // ==================【修复】结束 ==================

    if (address + current_node_size > frame_length - 1) {
      printf("warning: address(%zu) + current_node_size(%zu) > "
             "frame_length(%zu)-1",
             address, current_node_size, frame_length);
      return 0;
    }

    TRY_MALLOC_NEW_NODE(g_nlt_nodeframe0.result.nodes[i], nlt_nodeframe0_node_t)
    nlt_nodeframe0_node_t *node = g_nlt_nodeframe0.result.nodes[i];
    node->role = begin[0];
    node->id = begin[1];
    node->data_length = node_data_length;  // 同步修改这里
    memcpy(node->data, begin + 4, node_data_length);  // 同步修改这里

    address += current_node_size;
  }

  return 1;
}

nlt_nodeframe0_t g_nlt_nodeframe0 = {.fixed_part_size = 11,
                                     .frame_header = 0x55,
                                     .function_mark = 0x02,
                                     .UnpackData = UnpackData};
