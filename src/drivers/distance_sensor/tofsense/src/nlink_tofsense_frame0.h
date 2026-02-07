/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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

#ifndef NLINK_TOFSENSE_FRAME0_H
#define NLINK_TOFSENSE_FRAME0_H
#ifdef __cplusplus
extern "C" {
#endif
//#include "nlink_typedef.h"

#ifndef NTS_FRAME0_T_DEFINED
#define NTS_FRAME0_T_DEFINED

typedef struct {
    uint8_t id;
    uint32_t system_time;
    float dis;
    uint8_t dis_status;
    uint16_t signal_strength;
    uint8_t range_precision; // cm, only valid in tofsense-f
} nts_frame0_result_t;

typedef struct {
    const size_t fixed_part_size;
    const uint8_t frame_header;
    const uint8_t function_mark;
    nts_frame0_result_t result;

    uint8_t (*const UnpackData)(const uint8_t *data, size_t data_length);
} nts_frame0_t;

#endif // NTS_FRAME0_T_DEFINED

extern nts_frame0_t g_nts_frame0;

#ifdef __cplusplus
}
#endif

#endif // NLINK_TOFSENSE_FRAME0_H
