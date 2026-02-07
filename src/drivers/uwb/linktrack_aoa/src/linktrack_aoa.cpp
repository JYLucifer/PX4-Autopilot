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

#include "linktrack_aoa.hpp"

#include <lib/parameters/param.h>
#include <lib/drivers/device/Device.hpp>
#include <fcntl.h>
#include <poll.h>

using namespace linktrack_aoa;

static const ProtocolEntry g_protocols[] = {
    {0x07, g_nltaoa_nodeframe0.fixed_part_size,  g_nltaoa_nodeframe0.UnpackData,  &LinktrackAoa::initAoaNodeFrame0},
    {0x02, g_nlt_nodeframe0.fixed_part_size,     g_nlt_nodeframe0.UnpackData,     &LinktrackAoa::initNodeFrame0},
};

LinktrackAoa::LinktrackAoa(const char *port) :
    ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
    _linktrack_aoa_nodeframe_pub(ORB_ID(linktrack_nodeframe)),
    _linktrack_aoa_node_pub(ORB_ID(linktrack_node)),
    _linktrack_aoa_dataframe_pub(ORB_ID(linktrack_dataframe)),
    _linktrack_aoa_data_pub(ORB_ID(linktrack_data))
{
    strncpy(_port, port, sizeof(_port) - 1);

    _port[sizeof(_port) - 1] = '\0';

    _loop_perf = perf_alloc(PC_ELAPSED, "linktrack_aoa: cycle");
    _comms_errors = perf_alloc(PC_COUNT, "linktrack_aoa: com_err");
    _parse_errors = perf_alloc(PC_COUNT, "linktrack_aoa: parse_err");

    PX4_INFO("Linktrack AOA driver created for port: %s", _port);
}

LinktrackAoa::~LinktrackAoa()
{

	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int LinktrackAoa::init()
{
	int ret = 0;

	do {
		_fd = ::open(_port, O_RDWR | O_NOCTTY);

		if (_fd < 0) {
			PX4_ERR("打开文件描述符错误");
			return -1;
		}

		unsigned speed = B921600;
		termios uart_config{};
		int termios_state{};

		tcgetattr(_fd, &uart_config);

		uart_config.c_oflag &= ~ONLCR;

		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("配置输入波特率错误: %d", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("配置输出波特率错误: %d\n", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("设置波特率属性错误: %d", termios_state);
			ret = -1;
			break;
		}

		uart_config.c_cflag |= (CLOCAL | CREAD);
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8;
		uart_config.c_cflag &= ~PARENB;
		uart_config.c_cflag &= ~CSTOPB;
		uart_config.c_cflag &= ~CRTSCTS;

		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~OPOST;

		uart_config.c_cc[VMIN] = 0;
		uart_config.c_cc[VTIME] = 0;

		if (_fd < 0) {
			PX4_ERR("失败: 激光文件描述符");
			ret = -1;
			break;
		}
	} while (0);

	::close(_fd);
	_fd = -1;

	if (ret == PX4_OK) {
		start();
	}

	return ret;
}

void LinktrackAoa::start()
{
	ScheduleOnInterval(2_ms);
}

void LinktrackAoa::stop()
{
	ScheduleClear();
}

void LinktrackAoa::Run()
{
	if (_fd < 0) {
		_fd = ::open(_port, O_RDWR | O_NOCTTY);
		if (_fd < 0) {
			PX4_ERR("open %s failed", _port);
			return;
		}
	}

	int collect_ret = collect();

	if (collect_ret == -EAGAIN) {
		ScheduleDelayed(10);

		return;
	}
}

void LinktrackAoa::print_status()
{
    PX4_INFO("=== Linktrack Driver Status ===");
    PX4_INFO("Port: %s (FD: %d)", _port, _fd);

    perf_print_counter(_sample_perf);
    perf_print_counter(_comms_errors);

}

int LinktrackAoa::collect()
{
    perf_begin(_sample_perf);

    int nread = ::read(_fd,
                       _rx_buffer + _rx_buffer_len,
                       sizeof(_rx_buffer) - _rx_buffer_len);

    if (nread <= 0) {
        perf_end(_sample_perf);
        return -EAGAIN;
    }

    _rx_buffer_len += nread;
    _last_read = hrt_absolute_time();

    if (!try_parse_frame()) {
        perf_end(_sample_perf);
        return PX4_OK;
    }

    perf_end(_sample_perf);
    return PX4_OK;
}

bool LinktrackAoa::try_parse_frame()
{
    while (_rx_buffer_len >= 2) {

        if (_rx_buffer[0] != 0x55) {
            consume_frame(1);
            continue;
        }

		uint8_t function_mark = _rx_buffer[1];
		const ProtocolEntry *proto = find_protocol(function_mark);
		if (!proto) {
            consume_frame(1);
            continue;
        }

        if (_rx_buffer_len < proto->fixed_part_size) {
            return false;
        }

        size_t frame_len = NLINK_PROTOCOL_LENGTH(_rx_buffer);

        if (frame_len == 0 || frame_len > sizeof(_rx_buffer)) {
            consume_frame(1);
            continue;
        }

        if (_rx_buffer_len < frame_len) {
            return false;
        }

        if (!proto->unpack(_rx_buffer, frame_len)) {
            consume_frame(frame_len);
            continue;
        }

		if (proto->init_and_publish) {
			(this->*(proto->init_and_publish))(this);
		}
        consume_frame(frame_len);

        _last_valid_data = hrt_absolute_time();
        return true;
    }
    return false;
}

inline void LinktrackAoa::consume_frame(size_t len)
{
    if (len >= _rx_buffer_len) {
        _rx_buffer_len = 0;
    } else {
        memmove(_rx_buffer,
                _rx_buffer + len,
                _rx_buffer_len - len);
        _rx_buffer_len -= len;
    }
}

const ProtocolEntry *LinktrackAoa::find_protocol(uint8_t function_mark)
{
    for (size_t i = 0; i < sizeof(g_protocols) / sizeof(g_protocols[0]); i++) {
        if (g_protocols[i].function_mark == function_mark) {
            return &g_protocols[i];
        }
    }
    return nullptr;
}

void LinktrackAoa::initNodeFrame0(LinktrackAoa* drv)
{
    const auto &data = g_nlt_nodeframe0.result;
    linktrack_dataframe_s msg{};

    msg.timestamp = hrt_absolute_time();
    msg.frame = 0xA2;
    msg.role = data.role;
    msg.id = data.id;

    const size_t valid_node_count = data.valid_node_count;
    const size_t max_nodes = sizeof(msg.nodes) / sizeof(msg.nodes[0]); // 获取数组实际大小

    const size_t nodes_to_process = math::min(valid_node_count, max_nodes);

    // 3. 填充节点数据
    for (size_t i = 0; i < nodes_to_process; i++) {
        if (data.nodes[i] != nullptr) {
            msg.nodes[i].timestamp = hrt_absolute_time();
            msg.nodes[i].role = data.nodes[i]->role;
            msg.nodes[i].id = data.nodes[i]->id;


            const size_t src_data_len = data.nodes[i]->data_length;
            const size_t dest_buf_size = sizeof(msg.nodes[i].data);
			const size_t bytes_to_copy = math::min(src_data_len, dest_buf_size);

			if (bytes_to_copy > 0) {
                memcpy(msg.nodes[i].data, data.nodes[i]->data, bytes_to_copy);
            }

            if (src_data_len < dest_buf_size) {
                memset(msg.nodes[i].data + src_data_len, 0, dest_buf_size - src_data_len);
            }

        } else {
            msg.nodes[i].timestamp = 0;
            msg.nodes[i].role = 0;
            msg.nodes[i].id = 0;
            memset(msg.nodes[i].data, 0, sizeof(msg.nodes[i].data));
        }
    }
	drv->_linktrack_aoa_dataframe_pub.publish(msg);
}


void LinktrackAoa::initAoaNodeFrame0(LinktrackAoa* drv) {
    const auto &data = g_nltaoa_nodeframe0.result;

    linktrack_nodeframe_s msg{};
	linktrack_node_s block_msg{};

    msg.timestamp = hrt_absolute_time();
    msg.frame = 0xA0;
    msg.role = data.role;
    msg.id = data.id;
    msg.voltage = data.voltage;
    msg.local_time = data.local_time;
    msg.system_time = data.system_time;

	ARRAY_ASSIGN_NAN(msg.pos_3d);
	ARRAY_ASSIGN_NAN(msg.eop_3d);
	ARRAY_ASSIGN_NAN(msg.vel_3d);
	ARRAY_ASSIGN_NAN(msg.dis_arr);
	ARRAY_ASSIGN_NAN(msg.imu_gyro_3d);
	ARRAY_ASSIGN_NAN(msg.imu_acc_3d);
	ARRAY_ASSIGN_NAN(msg.angle_3d);
	ARRAY_ASSIGN_NAN(msg.quaternion);

	drv->_linktrack_aoa_nodeframe_pub.publish(msg);

    constexpr size_t MAX_NODES_PER_PAGE = 32;
    const size_t total_nodes = data.valid_node_count;
    const size_t total_pages =
        (total_nodes + MAX_NODES_PER_PAGE - 1) / MAX_NODES_PER_PAGE;

    for (size_t page = 0; page < total_pages; page++) {

        memset(&block_msg, 0, sizeof(block_msg));

        block_msg.timestamp   = hrt_absolute_time();
        block_msg.page        = page;
        block_msg.total_pages = total_pages;

        const size_t start_index = page * MAX_NODES_PER_PAGE;
        const size_t end_index =
            math::min(start_index + MAX_NODES_PER_PAGE, total_nodes);

        uint8_t valid_count = 0;

        for (size_t i = start_index; i < end_index; i++) {
            const size_t page_index = i - start_index;

            if (data.nodes[i] != nullptr) {
                block_msg.id[page_index]        = data.nodes[i]->id;
                block_msg.role[page_index]      = data.nodes[i]->role;
                block_msg.dis[page_index]       = data.nodes[i]->dis;
                block_msg.angle[page_index]     = data.nodes[i]->angle;
                block_msg.fp_rssi[page_index]   = data.nodes[i]->fp_rssi;
                block_msg.rx_rssi[page_index]   = data.nodes[i]->rx_rssi;
                valid_count++;

            } else {
                block_msg.id[page_index]      = 0;
                block_msg.role[page_index]    = 0;
                block_msg.dis[page_index]     = 0.0f;
                block_msg.angle[page_index]   = 0.0f;
                block_msg.fp_rssi[page_index] = 0.0f;
                block_msg.rx_rssi[page_index] = 0.0f;
            }
        }

        block_msg.valid_count = valid_count;
        drv->_linktrack_aoa_node_pub.publish(block_msg);
    }
}
