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

#pragma once
#include <termios.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/linktrack_nodeframe.h>
#include <uORB/topics/linktrack_node.h>
#include <uORB/topics/linktrack_dataframe.h>
#include <uORB/topics/linktrack_data.h>
#include <uORB/topics/vehicle_odometry.h>
//#include <uORB/topics/vehicle_visual_odometry.h>

#include "linktrack_parser/nlink_linktrack_anchorframe0.h"
#include "linktrack_parser/nlink_linktrack_tagframe0.h"
#include "linktrack_parser/nlink_linktrack_nodeframe0.h"
#include "linktrack_parser/nlink_linktrack_nodeframe1.h"
#include "linktrack_parser/nlink_linktrack_nodeframe2.h"
#include "linktrack_parser/nlink_linktrack_nodeframe3.h"
#include "linktrack_parser/nlink_linktrack_nodeframe4.h"
#include "linktrack_parser/nlink_linktrack_nodeframe5.h"
#include "linktrack_parser/nlink_linktrack_nodeframe6.h"
#include "linktrack_parser/nlink_linktrack_nodeframe7.h"

#include "linktrack_parser/nlink_utils.h"


#define LINKTRACK_DEFAULT_PORT	"/dev/ttyS3"

#define ARRAY_ASSIGN(DEST, SRC)                                      \
  for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT) \
  {                                                                  \
    DEST[_CNT] = SRC[_CNT];                                          \
  }

#define ARRAY_ASSIGN_NAN(DEST)                                       \
    do {                                                             \
        for (size_t _CNT = 0;                                        \
             _CNT < sizeof(DEST) / sizeof(DEST[0]); ++_CNT) {        \
            DEST[_CNT] = NAN;                                        \
        }                                                            \
    } while (0)


using namespace time_literals;
namespace linktrack
{
class Linktrack;
struct ProtocolEntry {
    uint8_t function_mark;
    size_t fixed_part_size;
    uint8_t (*unpack)(const uint8_t *data, size_t len);
    void (Linktrack::*init_and_publish)(Linktrack* drv);
};

class Linktrack : public px4::ScheduledWorkItem
{
public:
    Linktrack(const char *port);

    virtual ~Linktrack();

    int init();

    void print_status();

    void initAnchorFrame0(Linktrack* drv);
    void initTagFrame0(Linktrack* drv);
    void initNodeFrame0(Linktrack* drv);
    void initNodeFrame1(Linktrack* drv);
    void initNodeFrame2(Linktrack* drv);
    void initNodeFrame3(Linktrack* drv);
    void initNodeFrame4(Linktrack* drv);
    void initNodeFrame5(Linktrack* drv);
    void initNodeFrame6(Linktrack* drv);
    void initNodeFrame7(Linktrack* drv);
	const ProtocolEntry* find_protocol(uint8_t function_mark);
private:

    void consume_frame(size_t frame_len);

    void initDataTransmission();

    int collect();

    void Run() override;

    void start();

    void stop();
	bool try_parse_frame();
	void _print_frame_statistics(hrt_abstime current_time);
	char _port[32] {};
    int _fd{-1};

	uint8_t _rx_buffer[8192]{};
    size_t _rx_buffer_len{0};

	hrt_abstime _last_read{0};
    hrt_abstime _last_valid_data{0};

    size_t _parse_position{0};
    float _current_distance{0.0f};
    uint16_t _signal_strength{0};
    uint8_t _dis_status{0};
    uint8_t _sensor_id{0};

	uORB::Publication<linktrack_nodeframe_s> _linktrack_nodeframe_pub{ORB_ID(linktrack_nodeframe)};
	uORB::Publication<linktrack_node_s> _linktrack_node_pub{ORB_ID(linktrack_node)};
	uORB::Publication<linktrack_dataframe_s> _linktrack_dataframe_pub{ORB_ID(linktrack_dataframe)};
	uORB::Publication<linktrack_data_s> _linktrack_data_pub{ORB_ID(linktrack_data)};

    perf_counter_t _loop_perf{nullptr};
    perf_counter_t _parse_errors{nullptr};
    perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
    perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

	// uORB::Publication<vehicle_odometry_s> _vehicle_odom_pub{ORB_ID(vehicle_odometry)};
	// 在 Linktrack 类定义中
	uORB::Publication<vehicle_odometry_s> _vehicle_odom_pub{ORB_ID(vehicle_visual_odometry)};
};

} // namespace Linktrack
