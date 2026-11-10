/**************************************************************************
 *
 *   Copyright (c) 2022-2024 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/sf45_sample.h>
#include <uORB/topics/vehicle_attitude.h>
#include "../lightware_sf45_serial/SF45ObstacleMap.hpp"

class SF45Can : public ModuleBase<SF45Can>, public px4::ScheduledWorkItem
{
public:
	SF45Can() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}
	static int task_spawn(int argc, char *argv[])
	{
		auto *instance = new SF45Can();

		if (!instance) { return PX4_ERROR; }

		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (!instance->init()) { delete instance; _object.store(nullptr); _task_id = -1; return PX4_ERROR; }

		return PX4_OK;
	}
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr)
	{
		if (reason) { PX4_WARN("%s", reason); }

		PRINT_MODULE_USAGE_NAME("lightware_sf45_can", "driver");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}
	int print_status() override
	{
		PX4_INFO("node: %ld sensor: %ld samples: %lu", (long)_node, (long)_sensor, (unsigned long)_samples);
		return 0;
	}
private:
	bool init()
	{
		int32_t yaw = 0, orientation = 24, timeout = 100;
		param_get(param_find("SF45C_NODE"), &_node);
		param_get(param_find("SF45C_SENSOR"), &_sensor);
		param_get(param_find("SF45C_YAW"), &yaw);
		param_get(param_find("SF45C_ORIENT"), &orientation);
		param_get(param_find("SF45C_TIMEOUT"), &timeout);

		if (_node < 1 || _node > 125 || _sensor < 0 || _sensor > 255) {
			PX4_ERR("configure SF45C_NODE (1..125) and SF45C_SENSOR before starting"); return false;
		}

		_timeout = uint64_t(math::constrain<int32_t>(timeout, 20, 5000)) * 1000;
		_map.configure(yaw, orientation, _timeout);
		_map.reset();
		ScheduleOnInterval(10000);
		return true;
	}
	void Run() override
	{
		if (should_exit()) { ScheduleClear(); exit_and_cleanup(); return; }

		vehicle_attitude_s attitude{};

		if (_attitude_sub.update(&attitude)) { _attitude = matrix::Quatf(attitude.q); }

		uint64_t now = hrt_absolute_time();
		bool changed = _map.expire(now);
		sf45_sample_s sample{};
		unsigned processed = 0;

		while (processed++ < 32 && _sample_sub.update(&sample)) {
			now = hrt_absolute_time();

			if (sample.node_id != _node || sample.sensor_id != _sensor || sample.timestamp > now
			    || now - sample.timestamp >= _timeout) { continue; }

			const uint16_t cm = sample.distance > 50.f ? 5001 : uint16_t(roundf(sample.distance * 100.f));
			_map.update(sample.yaw, cm, sample.timestamp, _attitude);
			++_samples;
			changed = true;
		}

		if (changed || !_published) {
			const auto report = _map.report(now);
			_publication.publish(report);
			_published = true;
		}
	}
	int32_t _node{0}, _sensor{0};
	uint32_t _samples{0};
	uint64_t _timeout{100000};
	bool _published{false};
	SF45ObstacleMap _map;
	matrix::Quatf _attitude{};
	uORB::Subscription _sample_sub{ORB_ID(sf45_sample)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Publication<obstacle_distance_s> _publication{ORB_ID(obstacle_distance)};
};
extern "C" __EXPORT int lightware_sf45_can_main(int argc, char *argv[]) { return SF45Can::main(argc, argv); }
