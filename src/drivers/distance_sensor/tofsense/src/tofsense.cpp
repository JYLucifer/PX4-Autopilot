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

#include "tofsense.hpp"

#include <lib/parameters/param.h>
#include <lib/drivers/device/Device.hpp>
#include <fcntl.h>
#include <poll.h>
#include <mathlib/mathlib.h>

using namespace tofsense;

#define MODULE_NAME "tofsense"

static constexpr uint32_t TOFSENSE_BAUDRATE = B921600;
static constexpr float MIN_DISTANCE_M = 0.03f;
static constexpr float MAX_DISTANCE_M = 8.0f;
static constexpr float FOV_RADIANS = math::radians(27.0f);

TofSense::TofSense(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation)
{
	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_TOFSENSE;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]);
	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
	_px4_rangefinder.set_min_distance(MIN_DISTANCE_M);
	_px4_rangefinder.set_max_distance(MAX_DISTANCE_M);
	_px4_rangefinder.set_fov(FOV_RADIANS);

	_parse_errors = perf_alloc(PC_COUNT, MODULE_NAME": parse_errors");
	_loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": loop");
}

TofSense::~TofSense()
{
	stop();

	if (_comms_errors != nullptr) {
		perf_free(_comms_errors);
	}
	if (_sample_perf != nullptr) {
		perf_free(_sample_perf);
	}
	if (_parse_errors != nullptr) {
		perf_free(_parse_errors);
	}
	if (_loop_perf != nullptr) {
		perf_free(_loop_perf);
	}
}

int TofSense::init()
{
	int ret = PX4_OK;

	do {
		_fd = ::open(_port, O_RDWR | O_NOCTTY);

		if (_fd < 0) {
			PX4_ERR("Error opening fd");
			return PX4_ERROR;
		}

		unsigned speed = B921600;
		termios uart_config{};
		int termios_state{};

		tcgetattr(_fd, &uart_config);

		uart_config.c_oflag &= ~ONLCR;

		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
			ret = PX4_ERROR;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
			ret = PX4_ERROR;
			break;
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
			ret = PX4_ERROR;
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

		uart_config.c_cc[VMIN] = 1;
		uart_config.c_cc[VTIME] = 1;

		if (_fd < 0) {
			PX4_ERR("FAIL: laser fd");
			ret = PX4_ERROR;
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

void TofSense::start()
{
	ScheduleOnInterval(10_ms);
}

void TofSense::stop()
{
	ScheduleClear();
}

void TofSense::Run()
{
	if (_fd < 0) {
		_fd = ::open(_port, O_RDWR | O_NOCTTY);
	}

	if (collect() == -EAGAIN) {
		ScheduleClear();
		ScheduleOnInterval(10_ms, 174);
		return;
	}
}

int TofSense::collect()
{
	perf_begin(_sample_perf);

	float distance_m = -1.0f;
	int signal_q = 0;

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	int bytes_available = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

	if (bytes_available <= 0) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	uint8_t readbuf[64];
	bool frame_parsed = false;

	while (bytes_available > 0 && !frame_parsed) {

		const int to_read = math::min((int)sizeof(readbuf), bytes_available);
		const int ret = ::read(_fd, readbuf, to_read);

		if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return -EAGAIN;
		}

		if (ret == 0) {
		break;
		}

		bytes_available -= ret;
		_last_read = hrt_absolute_time();

		if (_rx_buffer_len + ret > sizeof(_rx_buffer)) {
		_rx_buffer_len = 0;
		perf_count(_parse_errors);
		break;
		}

		memcpy(&_rx_buffer[_rx_buffer_len], readbuf, ret);
		_rx_buffer_len += ret;

		while (_rx_buffer_len >= g_nts_frame0.fixed_part_size && !frame_parsed) {

		if (g_nts_frame0.UnpackData(_rx_buffer, _rx_buffer_len)) {

			if (g_nts_frame0.result.dis_status == 0) {
			distance_m = g_nts_frame0.result.dis;
			signal_q  = g_nts_frame0.result.signal_strength;
			_last_valid_data = hrt_absolute_time();
			} else {
			perf_count(_parse_errors);
			}

			memmove(_rx_buffer,
				_rx_buffer + g_nts_frame0.fixed_part_size,
				_rx_buffer_len - g_nts_frame0.fixed_part_size);

			_rx_buffer_len -= g_nts_frame0.fixed_part_size;

			frame_parsed = true;
		} else {
			memmove(_rx_buffer, _rx_buffer + 1, _rx_buffer_len - 1);
			_rx_buffer_len--;
			perf_count(_parse_errors);
		}
		}
	}


	_px4_rangefinder.update(timestamp_sample, distance_m, signal_q);
	perf_end(_sample_perf);
	return PX4_OK;
}

void TofSense::print_status()
{
	PX4_INFO("Port: %s (FD: %d)", _port, _fd);

	if (_comms_errors != nullptr) {
		perf_print_counter(_comms_errors);
	}
	if (_parse_errors != nullptr) {
		perf_print_counter(_parse_errors);
	}
}
