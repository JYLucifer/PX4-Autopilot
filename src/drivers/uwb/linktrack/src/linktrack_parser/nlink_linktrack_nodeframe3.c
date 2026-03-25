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

#include "linktrack_parser/nlink_linktrack_nodeframe3.h"

#include "nlink_utils/nlink_utils.h"

#pragma pack(1)
typedef struct {
  uint8_t role;
  uint8_t id;
  nint24_t dis;
  uint8_t fp_rssi;
  uint8_t rx_rssi;
} nlt_nodeframe3_node_raw_t;

typedef struct {
  uint8_t header[2];
  uint16_t frame_length;
  uint8_t role;
  uint8_t id;
  uint32_t local_time;
  uint32_t system_time;
  uint8_t reserved0[4];
  uint16_t voltage;
  uint8_t valid_node_count;
  // nodes...
  // uint8_t checkSum;
} nlt_nodeframe3_raw_t;
#pragma pack()

static nlt_nodeframe3_raw_t g_frame;

static uint8_t UnpackData(const uint8_t *data, size_t data_length) {
  if (data_length < g_nlt_nodeframe3.fixed_part_size ||
      data[0] != g_nlt_nodeframe3.frame_header ||
      data[1] != g_nlt_nodeframe3.function_mark)
    return 0;
  size_t frame_length = NLINK_PROTOCOL_LENGTH(data);
  if (data_length < frame_length)
    return 0;
  if (!NLINK_VerifyCheckSum(data, frame_length))
    return 0;

  static uint8_t init_needed = 1;
  if (init_needed) {
    memset(g_nlt_nodeframe3.result.nodes, 0,
           sizeof(g_nlt_nodeframe3.result.nodes));
    init_needed = 0;
  }

  memcpy(&g_frame, data, g_nlt_nodeframe3.fixed_part_size);
  g_nlt_nodeframe3.result.role = g_frame.role;
  g_nlt_nodeframe3.result.id = g_frame.id;
  g_nlt_nodeframe3.result.local_time = g_frame.local_time;
  g_nlt_nodeframe3.result.system_time = g_frame.system_time;
  g_nlt_nodeframe3.result.voltage = g_frame.voltage / MULTIPLY_VOLTAGE;

  g_nlt_nodeframe3.result.valid_node_count = g_frame.valid_node_count;
  nlt_nodeframe3_node_raw_t raw_node;
  for (size_t i = 0; i < g_frame.valid_node_count; ++i) {
    TRY_MALLOC_NEW_NODE(g_nlt_nodeframe3.result.nodes[i], nlt_nodeframe3_node_t)

    memcpy(&raw_node,
           data + g_nlt_nodeframe3.fixed_part_size +
               i * sizeof(nlt_nodeframe3_node_raw_t),
           sizeof(nlt_nodeframe3_node_raw_t));

    nlt_nodeframe3_node_t *node = g_nlt_nodeframe3.result.nodes[i];
    node->role = raw_node.role;
    node->id = raw_node.id;
    node->dis = NLINK_ParseInt24(raw_node.dis) / MULTIPLY_DIS;
    node->fp_rssi = raw_node.fp_rssi / MULTIPLY_RSSI;
    node->rx_rssi = raw_node.rx_rssi / MULTIPLY_RSSI;
  }
  return 1;
}

nlt_nodeframe3_t g_nlt_nodeframe3 = {.fixed_part_size = 21,
                                     .frame_header = 0x55,
                                     .function_mark = 0x05,
                                     .UnpackData = UnpackData};
