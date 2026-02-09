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
#ifndef NLINK_UTILS_H
#define NLINK_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum {
    LINKTRACK_ROLE_NODE,
    LINKTRACK_ROLE_ANCHOR,
    LINKTRACK_ROLE_TAG,
    LINKTRACK_ROLE_CONSOLE,
    LINKTRACK_ROLE_DT_MASTER,
    LINKTRACK_ROLE_DT_SLAVE,
    LINKTRACK_ROLE_MONITOR,
} linktrack_role_e;

typedef uint32_t nlink_id_t;

#define MAX_ANCHOR_COUNT 16
#define MAX_TAG_COUNT    16

#define ARRAY_LENGTH(X) (sizeof(X) / sizeof((X)[0]))

#define NLINK_PROTOCOL_LENGTH(X) ((size_t)((X)[2] | ((X)[3] << 8)))

#define NLINK_TRANSFORM_ARRAY(DEST, SRC, MULTIPLY)                  \
    for (size_t _i = 0; _i < ARRAY_LENGTH(SRC); ++_i) {             \
        (DEST)[_i] = (SRC)[_i] / (MULTIPLY);                         \
    }

#define NLINK_TRANSFORM_ARRAY_INT24(DEST, SRC, MULTIPLY)            \
    for (size_t _i = 0; _i < ARRAY_LENGTH(SRC); ++_i) {             \
        (DEST)[_i] = NLINK_ParseInt24((SRC)[_i]) / (MULTIPLY);       \
    }

#define TRY_MALLOC_NEW_NODE(NODE_PTR, NODE_TYPE)                    \
    if (!(NODE_PTR)) {                                              \
        void *p = malloc(sizeof(NODE_TYPE));                        \
        if (p) {                                                    \
            (NODE_PTR) = (NODE_TYPE *)p;                             \
            memset(p, 0, sizeof(NODE_TYPE));                         \
        } else {                                                     \
            printf("Memory allocation failed\r\n");                 \
            return 0;                                                \
        }                                                            \
    }

#pragma pack(push, 1)
typedef struct {
    uint8_t byteArray[3];
} nint24_t;

typedef struct {
    uint8_t byteArray[3];
} nuint24_t;
#pragma pack(pop)

int32_t  NLINK_ParseInt24(nint24_t data);
uint32_t NLINK_ParseUint24(nuint24_t data);

uint8_t  NLINK_VerifyCheckSum(const void *data, size_t data_length);
void     NLink_UpdateCheckSum(uint8_t *data, size_t data_length);

size_t   NLink_StringToHex(const char *str, uint8_t *out);

#define MULTIPLY_VOLTAGE 1000.0f
#define MULTIPLY_POS     1000.0f
#define MULTIPLY_DIS     1000.0f
#define MULTIPLY_VEL     10000.0f
#define MULTIPLY_ANGLE   100.0f
#define MULTIPLY_RSSI   -2.0f
#define MULTIPLY_EOP     100.0f

#ifdef __cplusplus
}
#endif

#endif // NLINK_UTILS_H

