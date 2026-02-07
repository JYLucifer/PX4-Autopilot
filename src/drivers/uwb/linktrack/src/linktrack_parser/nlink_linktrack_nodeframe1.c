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

#include "linktrack_parser/nlink_linktrack_nodeframe1.h"

#include "linktrack_parser/nlink_utils.h"

#pragma pack(1)
typedef struct {
  uint8_t role;
  uint8_t id;
  nint24_t pos_3d[3];
  uint8_t reserved[9];
} nlt_nodeframe1_node_raw_t;

typedef struct {
  uint8_t header[2];
  uint16_t frame_length;
  uint8_t role;
  uint8_t id;
  uint32_t system_time;
  uint32_t local_time;
  uint8_t reserved0[10];
  uint16_t voltage;

  uint8_t valid_node_count;
  // nodes...
  // uint8_t checkSum;
} nlt_nodeframe1_raw_t;
#pragma pack()

static nlt_nodeframe1_raw_t g_frame;

static uint8_t UnpackData(const uint8_t *data, size_t data_length) {
  if (data_length < g_nlt_nodeframe1.fixed_part_size ||
      data[0] != g_nlt_nodeframe1.frame_header ||
      data[1] != g_nlt_nodeframe1.function_mark)
    return 0;
  size_t frame_length = NLINK_PROTOCOL_LENGTH(data);
  if (data_length < frame_length)
    return 0;
  if (!NLINK_VerifyCheckSum(data, frame_length))
    return 0;

  static uint8_t init_needed = 1;
  if (init_needed) {
    memset(g_nlt_nodeframe1.result.nodes, 0,
           sizeof(g_nlt_nodeframe1.result.nodes));
    init_needed = 0;
  }

  memcpy(&g_frame, data, g_nlt_nodeframe1.fixed_part_size);

  g_nlt_nodeframe1.result.id = g_frame.id;
  g_nlt_nodeframe1.result.local_time = g_frame.local_time;
  g_nlt_nodeframe1.result.system_time = g_frame.system_time;
  g_nlt_nodeframe1.result.voltage = g_frame.voltage / MULTIPLY_VOLTAGE;

  g_nlt_nodeframe1.result.valid_node_count = g_frame.valid_node_count;
  nlt_nodeframe1_node_raw_t raw_node;
  for (size_t i = 0; i < g_frame.valid_node_count; ++i) {
    TRY_MALLOC_NEW_NODE(g_nlt_nodeframe1.result.nodes[i], nlt_nodeframe1_node_t)

    memcpy(&raw_node,
           data + g_nlt_nodeframe1.fixed_part_size +
               i * sizeof(nlt_nodeframe1_node_raw_t),
           sizeof(nlt_nodeframe1_node_raw_t));

    nlt_nodeframe1_node_t *node = g_nlt_nodeframe1.result.nodes[i];
    node->role = raw_node.role;
    node->id = raw_node.id;
    NLINK_TRANSFORM_ARRAY_INT24(node->pos_3d, raw_node.pos_3d, MULTIPLY_POS)
  }
  return 1;
}

nlt_nodeframe1_t g_nlt_nodeframe1 = {.fixed_part_size = 27,
                                     .frame_header = 0x55,
                                     .function_mark = 0x03,
                                     .UnpackData = UnpackData};
