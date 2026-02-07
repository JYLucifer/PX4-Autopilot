#pragma once
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

#include <termios.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/tofsensem_scan.h>

#include "nlink_tofsensem_frame0.h"
#include "nlink_utils.h"

#define TOFSENSEM_DEFAULT_PORT	"/dev/ttyS3"

#define ARRAY_ASSIGN(DEST, SRC)                                      \
  for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT) \
  {                                                                  \
    DEST[_CNT] = SRC[_CNT];                                          \
  }

using namespace time_literals;
namespace tofsensem
{

class TofSensem : public px4::ScheduledWorkItem
{
public:
    TofSensem(const char *port);

    virtual ~TofSensem();

    int init();

    void print_status();

private:

    int collect();

    void Run() override;

    void start();

    void stop();
	bool try_parse_frame();
	char _port[32] {};
    int _fd{-1};
	void drop_one_byte();
    void consume_frame(size_t frame_len);
    void drop_frame(size_t frame_len);
	void InitFrame0();

	uint8_t _rx_buffer[1024]{};
    size_t _rx_buffer_len{0};

	hrt_abstime _last_read{0};
    hrt_abstime _last_valid_data{0};

    char _linebuf[512] {};

    unsigned int _linebuf_index{0};

	uORB::Publication<tofsensem_scan_s> _tofsensem_scan_pub{ORB_ID(tofsensem_scan)};

    perf_counter_t _loop_perf{nullptr};
    perf_counter_t _parse_errors{nullptr};
    perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
    perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

};
} // namespace tofsensem
