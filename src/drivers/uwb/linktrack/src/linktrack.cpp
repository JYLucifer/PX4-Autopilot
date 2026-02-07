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

#include "linktrack.hpp"

#include <lib/parameters/param.h>
#include <lib/drivers/device/Device.hpp>
#include <fcntl.h>
#include <poll.h>
#include <cmath>



using namespace linktrack;

static const ProtocolEntry g_protocols[] = {
    // {0x00, nlt_anchorframe0_.fixed_part_size,  nlt_anchorframe0_.UnpackData,  &Linktrack::initAnchorFrame0},
    {0x01, g_nlt_tagframe0.fixed_part_size,    g_nlt_tagframe0.UnpackData,    &Linktrack::initTagFrame0},
    {0x02, g_nlt_nodeframe0.fixed_part_size,   g_nlt_nodeframe0.UnpackData,   &Linktrack::initNodeFrame0},
    // {0x03, g_nlt_nodeframe1.fixed_part_size,   g_nlt_nodeframe1.UnpackData,   &Linktrack::initNodeFrame1},
    {0x04, g_nlt_nodeframe2.fixed_part_size,   g_nlt_nodeframe2.UnpackData,   &Linktrack::initNodeFrame2},
    {0x05, g_nlt_nodeframe3.fixed_part_size,   g_nlt_nodeframe3.UnpackData,   &Linktrack::initNodeFrame3},
    // {0x06, g_nlt_nodeframe4.fixed_part_size,   g_nlt_nodeframe4.UnpackData,   &Linktrack::initNodeFrame4},
    {0x08, g_nlt_nodeframe5.fixed_part_size,   g_nlt_nodeframe5.UnpackData,   &Linktrack::initNodeFrame5},
    {0x09, g_nlt_nodeframe6.fixed_part_size,   g_nlt_nodeframe6.UnpackData,   &Linktrack::initNodeFrame6},
    // {0x0b, g_nlt_nodeframe7.fixed_part_size,   g_nlt_nodeframe7.UnpackData,   &Linktrack::initNodeFrame7},
};

Linktrack::Linktrack(const char *port) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_linktrack_nodeframe_pub(ORB_ID(linktrack_nodeframe))
{
    strncpy(_port, port, sizeof(_port) - 1);

    _port[sizeof(_port) - 1] = '\0';

    _loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");
    _comms_errors = perf_alloc(PC_COUNT, MODULE_NAME": com_err");
    _parse_errors = perf_alloc(PC_COUNT, MODULE_NAME": parse_err");

    PX4_INFO("Linktrack driver created for port: %s", _port);
}

Linktrack::~Linktrack()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int Linktrack::init()
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

void Linktrack::start()
{
	ScheduleOnInterval(2_ms);
}

void Linktrack::stop()
{
	ScheduleClear();
}

void Linktrack::Run()
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

void Linktrack::print_status()
{
    PX4_INFO("=== Linktrack Driver Status ===");
    PX4_INFO("Port: %s (FD: %d)", _port, _fd);
    perf_print_counter(_sample_perf);
    perf_print_counter(_comms_errors);
}

int Linktrack::collect()
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

bool Linktrack::try_parse_frame()
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

void Linktrack::consume_frame(size_t frame_len)
{
    if (frame_len >= _rx_buffer_len) {
        _rx_buffer_len = 0;
    } else {
        memmove(_rx_buffer,
                _rx_buffer + frame_len,
                _rx_buffer_len - frame_len);
        _rx_buffer_len -= frame_len;
    }
}

const ProtocolEntry *Linktrack::find_protocol(uint8_t function_mark)
{
    for (size_t i = 0; i < sizeof(g_protocols) / sizeof(g_protocols[0]); i++) {
        if (g_protocols[i].function_mark == function_mark) {
            return &g_protocols[i];
        }
    }
    return nullptr;
}

void Linktrack::initTagFrame0(Linktrack* drv)
{
    const auto &data = g_nlt_tagframe0.result;

    linktrack_nodeframe_s msg{};

    msg.timestamp = hrt_absolute_time();
    msg.frame = 0x01;
    msg.role = data.role;
    msg.id = data.id;
	msg.voltage = data.voltage;
    msg.local_time = data.local_time;
    msg.system_time = data.system_time;

	ARRAY_ASSIGN(msg.pos_3d, data.pos_3d)
	ARRAY_ASSIGN(msg.eop_3d, data.eop_3d)
	ARRAY_ASSIGN(msg.vel_3d, data.vel_3d)
	ARRAY_ASSIGN(msg.dis_arr, data.dis_arr)
	ARRAY_ASSIGN(msg.imu_gyro_3d, data.imu_gyro_3d)
	ARRAY_ASSIGN(msg.imu_acc_3d, data.imu_acc_3d)
	ARRAY_ASSIGN(msg.angle_3d, data.angle_3d)
	ARRAY_ASSIGN(msg.quaternion, data.quaternion)

	drv->_linktrack_nodeframe_pub.publish(msg);

    /*==============================
     * 2. vehicle_odometry（EKF 核心输入）
     *    —— 等价 APM: set_vehicle_position()
     *==============================*/
    vehicle_odometry_s odom{};
    odom.timestamp        = msg.timestamp;
    odom.timestamp_sample = msg.timestamp;

    /* ---- 坐标系：本地 NED ---- */
    odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;

    /* ---- 位置（单位：米）----
     * data.pos_3d 已是 NED & 米
     */
    odom.position[0] = data.pos_3d[0];
    odom.position[1] = data.pos_3d[1];
    odom.position[2] = data.pos_3d[2];

    /* ---- APM 风格 EOP → σ → σ² ----
     * eop ≠ 标准差
     * eop ≠ 方差
     * eop = 精度因子（人为映射）
     */
    const float precision_x = data.eop_3d[0];
    const float precision_y = data.eop_3d[1];
    const float precision_z = data.eop_3d[2];

    /* 平面误差合成（完全照 APM） */
    float pos_err_xy = sqrtf(precision_x * precision_x +
                             precision_y * precision_y);

    /* 下限保护（APM 也有） */
    pos_err_xy = math::max(pos_err_xy, 0.1f);

    /* Z 轴通常更差，单独设下限 */
    float pos_err_z = math::max(precision_z, 0.2f);

    /* EKF 要的是“方差 σ²” */
    odom.position_variance[0] = pos_err_xy * pos_err_xy;
    odom.position_variance[1] = pos_err_xy * pos_err_xy;
    odom.position_variance[2] = pos_err_z  * pos_err_z;

    /* ---- EKF 稳定性保护（PX4 常规做法）---- */
    // odom.position_variance[0] *= 1.5f;
    // odom.position_variance[1] *= 1.5f;
    // odom.position_variance[2] *= 2.0f;

    /* ---- 不提供姿态 ---- */
    odom.q[0] = NAN;
    odom.q[1] = NAN;
    odom.q[2] = NAN;
    odom.q[3] = NAN;
    ARRAY_ASSIGN_NAN(odom.orientation_variance);

    /* ---- 不提供速度 ---- */
    ARRAY_ASSIGN_NAN(odom.velocity);
    odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_UNKNOWN;
    ARRAY_ASSIGN_NAN(odom.velocity_variance);

    /* ---- 不提供角速度 ---- */
    ARRAY_ASSIGN_NAN(odom.angular_velocity);

    /* ---- 质量 & reset ---- */
    odom.reset_counter = 0;
    odom.quality = 100;   // UWB 通常给高质量

    drv->_vehicle_odom_pub.publish(odom);
}

void Linktrack::initNodeFrame0(Linktrack* drv)
{
    const auto &data = g_nlt_nodeframe0.result;
    linktrack_dataframe_s msg{};

    msg.timestamp = hrt_absolute_time();
    msg.frame = 0x02;
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
	drv->_linktrack_dataframe_pub.publish(msg);
}

void Linktrack::initNodeFrame2(Linktrack* drv)
{
    const auto &data = g_nlt_nodeframe2.result;

    linktrack_nodeframe_s msg{};
	linktrack_node_s block_msg{};

    msg.timestamp = hrt_absolute_time();
    msg.frame = 0x04;
    msg.role = data.role;
    msg.id = data.id;
	msg.voltage = data.voltage;
    msg.local_time = data.local_time;
    msg.system_time = data.system_time;

	ARRAY_ASSIGN(msg.pos_3d, data.pos_3d)
	ARRAY_ASSIGN(msg.eop_3d, data.eop_3d)
	ARRAY_ASSIGN(msg.vel_3d, data.vel_3d)
	ARRAY_ASSIGN(msg.imu_gyro_3d, data.imu_gyro_3d)
	ARRAY_ASSIGN(msg.imu_acc_3d, data.imu_acc_3d)
	ARRAY_ASSIGN(msg.angle_3d, data.angle_3d)
	ARRAY_ASSIGN(msg.quaternion, data.quaternion)
	ARRAY_ASSIGN_NAN(msg.dis_arr);
	drv->_linktrack_nodeframe_pub.publish(msg);

    constexpr size_t MAX_NODES_PER_PAGE = 32;
	const size_t total_nodes = data.valid_node_count;

    size_t total_pages = (total_nodes + MAX_NODES_PER_PAGE - 1) / MAX_NODES_PER_PAGE;

    for (size_t page = 0; page < total_pages; page++) {

        memset(&block_msg, 0, sizeof(block_msg));

        block_msg.timestamp = hrt_absolute_time();
        block_msg.page = page;
        block_msg.total_pages = total_pages;

		const size_t start_index = page * MAX_NODES_PER_PAGE;
        const size_t end_index   = math::min(start_index + MAX_NODES_PER_PAGE,
                                             total_nodes);

		uint8_t valid_count = 0;

        for (size_t i = start_index; i < end_index; i++) {
            size_t page_index = i - start_index;

            if (data.nodes[i] != nullptr) {
                block_msg.id[page_index] = data.nodes[i]->id;
                block_msg.role[page_index] = data.nodes[i]->role;
                block_msg.dis[page_index] = data.nodes[i]->dis;
                block_msg.fp_rssi[page_index] = data.nodes[i]->fp_rssi;
                block_msg.rx_rssi[page_index] = data.nodes[i]->rx_rssi;
				valid_count++;

            } else {
                block_msg.id[page_index] = 0;
                block_msg.role[page_index] = 0;
                block_msg.dis[page_index] = 0.0f;
                block_msg.fp_rssi[page_index] = 0.0f;
                block_msg.rx_rssi[page_index] = 0.0f;

            }
        }
		block_msg.valid_count = valid_count;
        drv->_linktrack_node_pub.publish(block_msg);

    }
}

void Linktrack::initNodeFrame3(Linktrack* drv)
{
    const auto &data = g_nlt_nodeframe3.result;

    linktrack_nodeframe_s msg{};
	linktrack_node_s block_msg{};

    msg.timestamp = hrt_absolute_time();
    msg.frame = 0x05;
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

	drv->_linktrack_nodeframe_pub.publish(msg);

    constexpr size_t MAX_NODES_PER_PAGE = 32;
	const size_t total_nodes = data.valid_node_count;

    // 检查是否有有效的节点数据
    if (total_nodes > 0) {
        size_t total_pages = (total_nodes + MAX_NODES_PER_PAGE - 1) / MAX_NODES_PER_PAGE;

        for (size_t page = 0; page < total_pages; page++) {
            memset(&block_msg, 0, sizeof(block_msg));

            block_msg.timestamp = hrt_absolute_time();
            block_msg.page = page;
            block_msg.total_pages = total_pages;

            const size_t start_index = page * MAX_NODES_PER_PAGE;
            const size_t end_index = math::min(start_index + MAX_NODES_PER_PAGE, total_nodes);

            uint8_t valid_count = 0;

            for (size_t i = start_index; i < end_index; i++) {
                size_t page_index = i - start_index;

                if (i < total_nodes && data.nodes[i] != nullptr) {
                    // 复制节点数据
                    block_msg.id[page_index] = data.nodes[i]->id;
                    block_msg.role[page_index] = data.nodes[i]->role;
                    block_msg.dis[page_index] = data.nodes[i]->dis;
                    block_msg.fp_rssi[page_index] = data.nodes[i]->fp_rssi;
                    block_msg.rx_rssi[page_index] = data.nodes[i]->rx_rssi;
                    valid_count++;
                } else {
                    // 节点数据为空时的处理
                    block_msg.id[page_index] = 0;
                    block_msg.role[page_index] = 0;
                    block_msg.dis[page_index] = 0.0f;
                    block_msg.fp_rssi[page_index] = 0.0f;
                    block_msg.rx_rssi[page_index] = 0.0f;
                }
            }

            block_msg.valid_count = valid_count;

            // 只有当有有效数据时才发布
            if (valid_count > 0) {
                drv->_linktrack_node_pub.publish(block_msg);
            }
        }
    }
}

void Linktrack::initNodeFrame5(Linktrack* drv)
{
    const auto &data = g_nlt_nodeframe5.result;

    linktrack_nodeframe_s msg{};
	linktrack_node_s block_msg{};

    msg.timestamp = hrt_absolute_time();
    msg.frame = 0x07;
    msg.role = data.role;
    msg.id = data.id;
	msg.voltage = data.voltage;
    msg.local_time = data.local_time;
    msg.system_time = data.system_time;

	ARRAY_ASSIGN_NAN(msg.pos_3d);
	ARRAY_ASSIGN_NAN(msg.eop_3d);
	ARRAY_ASSIGN_NAN(msg.vel_3d);
	ARRAY_ASSIGN_NAN(msg.imu_gyro_3d);
	ARRAY_ASSIGN_NAN(msg.imu_acc_3d);
	ARRAY_ASSIGN_NAN(msg.angle_3d);
	ARRAY_ASSIGN_NAN(msg.quaternion);
	ARRAY_ASSIGN_NAN(msg.dis_arr);

	drv->_linktrack_nodeframe_pub.publish(msg);

    constexpr size_t MAX_NODES_PER_PAGE = 32;
	const size_t total_nodes = data.valid_node_count;

    size_t total_pages = (total_nodes + MAX_NODES_PER_PAGE - 1) / MAX_NODES_PER_PAGE;

    for (size_t page = 0; page < total_pages; page++) {

        memset(&block_msg, 0, sizeof(block_msg));

        block_msg.timestamp = hrt_absolute_time();
        block_msg.page = page;
        block_msg.total_pages = total_pages;

		const size_t start_index = page * MAX_NODES_PER_PAGE;
        const size_t end_index   = math::min(start_index + MAX_NODES_PER_PAGE,
                                             total_nodes);

		uint8_t valid_count = 0;

        for (size_t i = start_index; i < end_index; i++) {
            size_t page_index = i - start_index;

            if (data.nodes[i] != nullptr) {
                block_msg.id[page_index] = data.nodes[i]->id;
                block_msg.role[page_index] = data.nodes[i]->role;
                block_msg.dis[page_index] = data.nodes[i]->dis;
                block_msg.fp_rssi[page_index] = data.nodes[i]->fp_rssi;
                block_msg.rx_rssi[page_index] = data.nodes[i]->rx_rssi;
				valid_count++;

            } else {
                block_msg.id[page_index] = 0;
                block_msg.role[page_index] = 0;
                block_msg.dis[page_index] = 0.0f;
                block_msg.fp_rssi[page_index] = 0.0f;
                block_msg.rx_rssi[page_index] = 0.0f;

            }
        }
		block_msg.valid_count = valid_count;
        drv->_linktrack_node_pub.publish(block_msg);

    }
}

void Linktrack::initNodeFrame6(Linktrack* drv)
{
    const auto &data = g_nlt_nodeframe6.result;
    linktrack_dataframe_s msg{};

    msg.timestamp = hrt_absolute_time();
    msg.frame = 0x09;
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
	drv->_linktrack_dataframe_pub.publish(msg);
}
