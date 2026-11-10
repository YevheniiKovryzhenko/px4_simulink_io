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

#pragma once
#include "SF45Scan.hpp"
#include <lib/mathlib/math/Functions.hpp>
#include <lib/mathlib/math/Limits.hpp>
#include <ObstacleMath.hpp>
#include <uORB/topics/obstacle_distance.h>
#include <matrix/matrix/math.hpp>

// Transport-independent SF45 output contract. Both UART and CAN use this path.
class SF45ObstacleMap
{
public:
	void reset() { _scan.reset(); }
	void configure(int32_t yaw, int32_t orientation, uint64_t timeout)
	{
		_offset = math::degrees(ObstacleMath::sensor_orientation_to_yaw_offset(
						static_cast<ObstacleMath::SensorOrientation>(yaw)));
		_downward = orientation == 25;
		_timeout = timeout;
	}
	void update(float yaw, uint16_t distance_cm, uint64_t now, const matrix::Quatf &attitude)
	{
		if (_downward) { yaw = -yaw; }

		const int bin = ObstacleMath::get_bin_at_angle(5.f, yaw);
		uint16_t distance = 5001;

		if (distance_cm <= 5000) {
			float projected = float(distance_cm);
			ObstacleMath::project_distance_on_horizontal_plane(projected,
					math::radians(ObstacleMath::wrap_360(yaw + _offset)), attitude);
			distance = uint16_t(projected);
		}

		_scan.update(bin, distance, now, _timeout);
	}
	bool expire(uint64_t now) { return _scan.expire(now, _timeout); }
	obstacle_distance_s report(uint64_t now)
	{
		_scan.expire(now, _timeout);
		obstacle_distance_s msg{};
		msg.timestamp = now;
		msg.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
		msg.sensor_type = obstacle_distance_s::MAV_DISTANCE_SENSOR_LASER;
		msg.increment = 5;
		msg.min_distance = 20;
		msg.max_distance = 5000;
		msg.angle_offset = ObstacleMath::wrap_360(_offset);

		for (size_t i = 0; i < 72; ++i) { msg.distances[i] = _scan.distance(i); }

		return msg;
	}
private:
	SF45Scan<72> _scan;
	float _offset{0.f};
	bool _downward{false};
	uint64_t _timeout{100000};
};
