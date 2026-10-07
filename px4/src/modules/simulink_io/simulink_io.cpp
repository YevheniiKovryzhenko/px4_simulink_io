# /****************************************************************************
#  *
#  *    Copyright (C) 2025  Yevhenii Kovryzhenko. All rights reserved.
#  *
#  *    This program is free software: you can redistribute it and/or modify
#  *    it under the terms of the GNU Affero General Public License as published by
#  *    the Free Software Foundation, either version 3 of the License, or
#  *    (at your option) any later version.
#  *
#  *    This program is distributed in the hope that it will be useful,
#  *    but WITHOUT ANY WARRANTY; without even the implied warranty of
#  *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  *    GNU Affero General Public License Version 3 for more details.
#  *
#  *    You should have received a copy of the
#  *    GNU Affero General Public License Version 3
#  *    along with this program.  If not, see <https://www.gnu.org/licenses/>.
#  *
#  *    1. Redistributions of source code must retain the above copyright
#  *       notice, this list of conditions, and the following disclaimer.
#  *    2. Redistributions in binary form must reproduce the above copyright
#  *       notice, this list of conditions, and the following disclaimer in
#  *       the documentation and/or other materials provided with the
#  *       distribution.
#  *    3. No ownership or credit shall be claimed by anyone not mentioned in
#  *       the above copyright statement.
#  *    4. Any redistribution or public use of this software, in whole or in part,
#  *       whether standalone or as part of a different project, must remain
#  *       under the terms of the GNU Affero General Public License Version 3,
#  *       and all distributions in binary form must be accompanied by a copy of
#  *       the source code, as stated in the GNU Affero General Public License.
#  *
#  ****************************************************************************/

#include "simulink_io.h"
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>
#include <errno.h>

SimulinkIO::~SimulinkIO()
{
    perf_free(_cycle_perf);
    perf_free(_interval_perf);
    perf_free(_overrun_perf);
}

void SimulinkIO::run()
{
    _simulink_model.initialize();
    constexpr hrt_abstime interval_us = SimulinkWrapper::SimulinkModel::period_us;
    static_assert(interval_us > 0, "Model period must be positive");
    PX4_INFO("Running %s: period %u us", SimulinkWrapper::SimulinkModel::name(),
             (unsigned)interval_us);
    hrt_abstime deadline = hrt_absolute_time();

    while (!should_exit()) {
        deadline += interval_us;
        perf_count(_interval_perf);
        perf_begin(_cycle_perf);
        _simulink_model.step();
        perf_end(_cycle_perf);

        const hrt_abstime now = hrt_absolute_time();
        if (deadline > now) {
            // Keep stop responsive even for models with long base periods.
            while (!should_exit()) {
                const hrt_abstime sleep_start = hrt_absolute_time();
                if (sleep_start >= deadline) { break; }
                const hrt_abstime remaining = deadline - sleep_start;
                px4_usleep(remaining < 10000 ? remaining : 10000);
            }
        } else {
            perf_count(_overrun_perf);
            deadline = now;
        }
    }

    _simulink_model.terminate();
}

int SimulinkIO::print_status()
{
    PX4_INFO("Model: %s", SimulinkWrapper::SimulinkModel::name());
    PX4_INFO("Configured period: %u us", (unsigned)SimulinkWrapper::SimulinkModel::period_us);
    perf_print_counter(_cycle_perf);
    perf_print_counter(_interval_perf);
    perf_print_counter(_overrun_perf);
    return 0;
}

int SimulinkIO::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("simulink_io",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  8192,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

SimulinkIO *SimulinkIO::instantiate(int argc, char *argv[])
{
    return new SimulinkIO();
}

int SimulinkIO::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int SimulinkIO::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Runs generated Simulink code at its compiled base sample period.
The generated adapter owns model initialization, I/O and termination.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("simulink_io", "control");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int simulink_io_main(int argc, char *argv[])
{
    return SimulinkIO::main(argc, argv);
}
