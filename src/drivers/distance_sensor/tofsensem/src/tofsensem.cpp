/****************************************************************************
 *
 *   Copyright (c) 2017-2021 PX4 Development Team. All rights reserved.
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

#include "tofsensem.hpp"

#include <lib/parameters/param.h>
#include <lib/drivers/device/Device.hpp>
#include <fcntl.h>
#include <poll.h>

using namespace tofsensem;

TofSensem::TofSensem(const char *port) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_tofsensem_scan_pub(ORB_ID(tofsensem_scan))
{
	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_TOFSENSEM;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

		uint8_t bus_num = atoi(&_port[strlen(_port) - 1]);

		if (bus_num < 10) {
			device_id.devid_s.bus = bus_num;
		}

	_loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");
	_comms_errors = perf_alloc(PC_COUNT, MODULE_NAME": com_err");
	_parse_errors = perf_alloc(PC_COUNT, MODULE_NAME": parse_err");
	PX4_INFO("TofSensem driver created for port: %s", _port);
}

TofSensem::~TofSensem()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int TofSensem::init()
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

void TofSensem::start()
{
	ScheduleOnInterval(2_ms);
}

void TofSensem::stop()
{
	ScheduleClear();
}

void TofSensem::Run()
{
	if (_fd < 0) {
		_fd = ::open(_port, O_RDWR | O_NOCTTY);
		if (_fd < 0) {
			PX4_ERR("open %s failed", _port);
			return;
		}
	}

	if (collect() == -EAGAIN) {
		ScheduleDelayed(10);

		return;
	}
}

int TofSensem::collect()
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

bool TofSensem::try_parse_frame()
{
    while (_rx_buffer_len >= g_ntsm_frame0.fixed_part_size) {

        if (_rx_buffer[0] != g_ntsm_frame0.frame_header ||
            _rx_buffer[1] != g_ntsm_frame0.function_mark) {
            consume_frame(1);
            continue;
        }

        if (!g_ntsm_frame0.UnpackData(_rx_buffer, _rx_buffer_len)) {
            return false;
        }

        const size_t frame_len = tofm_frame0_size(_rx_buffer);

        if (frame_len == 0 || frame_len > _rx_buffer_len) {
            consume_frame(1);
            continue;
        }

        InitFrame0();
        consume_frame(frame_len);
        return true;
    }
    return false;
}

inline void TofSensem::consume_frame(size_t len)
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

void TofSensem::InitFrame0()
{
	const ntsm_frame0_t &frame = g_ntsm_frame0;
	tofsensem_scan_s data{};
	data.timestamp = hrt_absolute_time();
	data.system_time = frame.system_time;
	data.id = frame.id;
	data.pixel_count = math::min(frame.pixel_count, (uint8_t)(sizeof(data.dis) / sizeof(data.dis[0])));
    for (int i = 0; i < data.pixel_count; i++) {
        data.dis[i] = frame.pixels[i].dis;
        data.dis_status[i]    = frame.pixels[i].dis_status;
        data.signal_strength[i]  = frame.pixels[i].signal_strength;
    }
	_tofsensem_scan_pub.publish(data);
}

void TofSensem::print_status()
{
    PX4_INFO("Port: %s (FD: %d)", _port, _fd);

    if (_comms_errors != nullptr) {
        perf_print_counter(_comms_errors);
    }
    if (_parse_errors != nullptr) {
        perf_print_counter(_parse_errors);
    }
}
