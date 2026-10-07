/****************************************************************************
 *
 *    Copyright (C) 2024  Yevhenii Kovryzhenko. All rights reserved.
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Affero General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Affero General Public License Version 3 for more details.
 *
 *    You should have received a copy of the
 *    GNU Affero General Public License Version 3
 *    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions, and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions, and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *    3. No ownership or credit shall be claimed by anyone not mentioned in
 *       the above copyright statement.
 *    4. Any redistribution or public use of this software, in whole or in part,
 *       whether standalone or as part of a different project, must remain
 *       under the terms of the GNU Affero General Public License Version 3,
 *       and all distributions in binary form must be accompanied by a copy of
 *       the source code, as stated in the GNU Affero General Public License.
 *
 ****************************************************************************/

#pragma once

#include <matrix/math.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sim_guidance_trajectory.h>
#include <uORB/topics/sim_guidance_status.h>
#include <uORB/topics/sim_guidance_request.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>

#include <px4_platform_common/module_params.h>
#include "file_loader_backend.hpp"




using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;


double get_dt_s_hrt(hrt_abstime &time_stamp);



template <typename Type>
class point
{
private:
	hrt_abstime	timestamp{0};
public:
	matrix::Vector<Type, 4> pos; // [x y z yaw]
	matrix::Vector<Type, 4> vel; // [x y z yaw]
	matrix::Vector<Type, 4> acc; // [x y z yaw]
	matrix::Vector<Type, 4> jerk; // [x y z yaw]
	matrix::Vector<Type, 4> snap; // [x y z yaw]

	void start(void);
	void reset(void);

	double get_time_s(void);

	point(/* args */);
	~point();
};

using pointf = point<float>;

class trajectory
{
private:
	static const size_t n_coeffs_max = 10;
	static const size_t n_dofs_max = 4;
	static const size_t n_int_max = 50;

	size_t n_coeffs = 0;
	size_t n_int = 0;
	size_t n_dofs = 0;

	matrix::Vector<matrix::Vector<matrix::Vector<float, n_coeffs_max>, n_dofs_max>, n_int_max> coefs;
	matrix::Vector<float, n_int_max> tof_int;

	pointf setpoint_initial{}, setpoint_current{}, vehicle_state{};


	void start(void);
	void reset(void);
	int load(void);

	int execute(void);
	int update_from_companion(void);
	int update_companion(bool request_start = false, bool request_stop = false, bool request_start_executing = false);
	int set_home();

	int update_vehicle_state(void);
	void publish_status();
	void fail(const char *reason);
	bool _home_valid{false};
	bool _local_loaded{false};
	int32_t _input_type{0};
	int32_t _output_mask{1};
	bool _use_companion{false};
	hrt_abstime _companion_timestamp{0};
	bool _reset_counters_valid{false};
	uint8_t _xy_reset{0}, _z_reset{0}, _heading_reset{0};
	static constexpr hrt_abstime input_timeout_us = 500000;
	static constexpr hrt_abstime companion_timeout_us = 1000000;

	struct LoadedTrajectory {
		size_t n_coeffs{0}, n_int{0}, n_dofs{0};
		decltype(coefs) coefficients{};
		decltype(tof_int) durations{};
	};
	int read_trajectory(file_loader_backend &loader, LoadedTrajectory &data);
	int publish_trajectory_setpoint(float time_trajectory_s);

	sim_guidance_status_s status{};
	debug_array_s sm_inbound{};
	vehicle_local_position_s vehicle_local_position{};





	// Publications
	uORB::Publication<sim_guidance_trajectory_s>	_sim_guidance_trajecotry_pub{ORB_ID(sim_guidance_trajectory)};
	uORB::Publication<sim_guidance_status_s>	_sim_guidance_status_pub{ORB_ID(sim_guidance_status)};
	uORB::Publication<debug_array_s>		_sim_guidance_pub{ORB_ID(simulink_guidance)};
	uORB::Publication<debug_array_s>		_companion_guidance_inbound_pub{ORB_ID(companion_guidance_inbound)};
	uORB::Publication<trajectory_setpoint_s>	_trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};


	// Subscriptions
	uORB::Subscription				_sim_guidance_request_sub{ORB_ID(sim_guidance_request)};
	uORB::Subscription				_sim_inbound_sub{ORB_ID(simulink_inbound)};
	uORB::Subscription				_vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription				_companion_guidance_outbound_sub{ORB_ID(companion_guidance_outbound)};

public:
	trajectory(/* args */);
	~trajectory();

	file_loader_backend file_loader{};
	int set_src(const char* _file);
	int set_src(const char* _dir, const char* _file);

	void print_status(void);
	void disable();
	void configure(int32_t input_type, int32_t output_mask);
	void update(bool use_companion = false); //main update loop
};

