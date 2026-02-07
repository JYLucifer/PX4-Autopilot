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

#include "tofsensem.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/cli.h>

namespace tofsensem
{

static TofSensem *g_dev{nullptr};

int start(const char *port);
int status();
int stop();
int usage();

int start(const char *port)
{

    if (g_dev != nullptr) {
        PX4_ERR("driver already started");
        return PX4_OK;
    }

    g_dev = new TofSensem(port);

    if (g_dev == nullptr) {
        PX4_ERR("driver start failed");
        return PX4_ERROR;
    }

    if (OK != g_dev->init()) {
        PX4_ERR("driver start failed");
        delete g_dev;
        g_dev = nullptr;
        return PX4_ERROR;
    }

    return PX4_OK;
}

int status()
{
    if (g_dev == nullptr) {
        PX4_INFO("Driver not running");
        return 1;
    }

    printf("state @ %p\n", g_dev);
    g_dev->print_status();
    return 0;
}

int stop()
{
    if (g_dev != nullptr) {
        PX4_INFO("stopping driver");

        delete g_dev;
        g_dev = nullptr;
        PX4_INFO("Driver stopped");
        return 0;
    }else {
        PX4_ERR("driver not running");
        return 1;
    }
    return PX4_OK;
}

int usage()
{
    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Driver for Nooploop TofSensem laser distance sensors.

This driver communicates with TofSensem sensors via UART serial port
using the nlink protocol library and publishes distance data through
the distance_sensor uORB topic.

### Implementation
- Uses nlink_utils protocol parsing library
- Supports multiple TofSensem sensors via sensor ID
- Provides real-time distance measurements at 30Hz
- Includes comprehensive error handling and diagnostics

### Examples
Start driver on TELEM1 (default):
$ tofsensem start

Start driver on specific serial port:
$ tofsensem start -d /dev/ttyS2

Stop driver:
$ tofsensem stop

Display driver status:
$ tofsensem status
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("tofsensem", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
    PRINT_MODULE_USAGE_PARAM_STRING('d', TOFSENSEM_DEFAULT_PORT, "<file:dev>", "Serial device", true);
    PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
    PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Driver status");

    return 0;
}

} // namespace tofsensem

extern "C" __EXPORT int tofsensem_main(int argc, char *argv[])
{
    int ch = 0;
    const char *device_path = TOFSENSEM_DEFAULT_PORT;

    int myoptind = 1;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
        case 'd':
            device_path = myoptarg;
            break;

        default:
            return tofsensem::usage();
        }
    }

    if (myoptind >= argc) {
        return tofsensem::usage();
    }

    const char *command = argv[myoptind];

    if (!strcmp(command, "start")) {
        if (strcmp(device_path, "") != 0) {
            return tofsensem::start(device_path);
        } else {
            PX4_WARN("Error: Please specify a valid serial device path!");
            return tofsensem::usage();
        }
    } else if (!strcmp(command, "stop")) {
        return tofsensem::stop();
    } else if (!strcmp(command, "status")) {
        return tofsensem::status();
    }

    PX4_ERR("Unrecognized command: %s", command);
    return tofsensem::usage();
}
