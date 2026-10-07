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

#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include "trajectory.hpp"
#include "trajectory_math.hpp"

static const int XYZ_OFFSET_START_IND = 18 + 17; //control vector size + number of states before xyz
static const int XYZ_VEL_OFFSET_START_IND = 18 + 0; //control vector size + number of states before xyz velocity
static const int QUAT_OFFSET_START_IND = 18 + 6; //control vector size + number of states before quat
static const int XYZ_ACC_OFFSET_START_IND = 18 + 14; //control vector size + number of states before accel

double get_dt_s_hrt(hrt_abstime &time_stamp)
{
	return static_cast<double>(hrt_elapsed_time(&time_stamp))*1.0E-6;
}

template<typename Type>
void point<Type>::start(void)
{
	timestamp = hrt_absolute_time();
	return;
}
template<typename Type>
void point<Type>::reset(void)
{
	pos.setZero();
	vel.setZero();
	acc.setZero();
	jerk.setZero();
	snap.setZero();
	return;
}
template<typename Type>
double point<Type>::get_time_s(void)
{
	return get_dt_s_hrt(timestamp);
}
template<typename Type>
point<Type>::point(/* args */)
{
	reset();
}
template<typename Type>
point<Type>::~point()
{
}

int trajectory::set_home()
{
	// Home is a stationary hold, even if captured while the vehicle is moving.
	setpoint_current.reset();
	setpoint_current.pos = vehicle_state.pos;
	setpoint_current.start();
	setpoint_initial = setpoint_current;
	_home_valid = true;
	return publish_trajectory_setpoint(0.0f);
}

void trajectory::publish_status()
{
	status.timestamp = hrt_absolute_time();
	_sim_guidance_status_pub.publish(status);
}

void trajectory::fail(const char *reason)
{
	PX4_WARN("%s", reason);
	status.executing = false;
	status.finished = true;
	status.trajectory_valid = false;
	if (_use_companion) {
		update_companion(false, true);
	}
}

void trajectory::disable()
{
	if (_use_companion && status.started) {
		update_companion(false, true);
	}
	reset();
	status.loaded = _use_companion ? false : _local_loaded;
	_companion_timestamp = 0;
	_reset_counters_valid = false;
	// Requests issued while disabled must not start an old trajectory on re-enable.
	sim_guidance_request_s ignored{};
	while (_sim_guidance_request_sub.update(&ignored)) {}
	publish_status();
}

void trajectory::configure(int32_t input_type, int32_t output_mask)
{
	if (input_type != _input_type || output_mask != _output_mask) {
		disable();
	}
	_input_type = input_type;
	_output_mask = output_mask;
}

void trajectory::update(bool use_companion)
{
	if (_use_companion != use_companion) {
		disable();
		_use_companion = use_companion;
		status.loaded = use_companion ? false : _local_loaded;
	}
	const bool state_valid = update_vehicle_state() == 0;
	if (status.executing && !state_valid) {
		fail("Vehicle state unavailable; stopping trajectory");
	}

	sim_guidance_request_s request{};
	// Acknowledgment is the status topic; never overwrite the command topic.
	while (_sim_guidance_request_sub.update(&request)) {
		if (request.stop) {
			status.executing = false;
			status.finished = true;
			status.trajectory_valid = false;
			if (use_companion) { update_companion(false, true); }
			continue;
		}
		if (request.reset) {
			reset();
			if (use_companion) { update_companion(false, true); }
		}
		if (request.start && (!status.started || status.finished)) {
			if (!state_valid) {
				fail("Cannot start without valid vehicle state");
				continue;
			}
			start();
			if (set_home() < 0) { fail("Cannot publish home"); continue; }
			if (use_companion) {
				status.loaded = false;
				update_companion(true);
			} else if (!_local_loaded && load() < 0) {
				fail("Failed to load trajectory");
			}
		}
		if (request.set_home && !request.start) {
			if (status.executing) {
				PX4_WARN("Stop execution before setting home");
			} else if (!state_valid) {
				fail("Cannot set home without valid vehicle state");
			} else if (set_home() < 0) {
				fail("Cannot publish home");
			}
		}
		if (request.start_execution && !request.start && !request.reset
		    && status.started && status.loaded && !status.finished && !status.executing && _home_valid) {
			if (!state_valid) { fail("Cannot execute without valid vehicle state"); continue; }
			status.executing = true;
			setpoint_initial.start();
			if (use_companion) { update_companion(false, false, true); }
			PX4_INFO("Started trajectory execution");
		}
	}

	if (use_companion) {
		if (update_from_companion() < 0) { fail("Invalid companion trajectory"); }
		if (_companion_timestamp == 0 || hrt_elapsed_time(&_companion_timestamp) > companion_timeout_us) {
			status.loaded = false;
			if (status.executing) { fail("Companion guidance timed out"); }
		}
		if (status.started && !status.finished) { update_companion(); }
	} else if (status.executing && execute() < 0) {
		fail("Failed to evaluate or publish trajectory");
	}
	status.trajectory_valid = status.started && status.loaded && _home_valid
				  && !status.finished && state_valid;
	publish_status();
}

int trajectory::read_trajectory(file_loader_backend &loader, LoadedTrajectory &data)
{
	traj_file_header_t header{};
	if (loader.read_header(header) < 0) { return -1; }
	if (header.n_coeffs == 0 || header.n_coeffs > n_coeffs_max
	    || header.n_int == 0 || header.n_int > n_int_max
	    || header.n_dofs == 0 || header.n_dofs > n_dofs_max) {
		PX4_WARN("Invalid trajectory dimensions");
		return -1;
	}
	data.n_coeffs = header.n_coeffs;
	data.n_int = header.n_int;
	data.n_dofs = header.n_dofs;
	float total = 0.0f;
	for (size_t segment = 0; segment < data.n_int; ++segment) {
		for (size_t axis = 0; axis < data.n_dofs; ++axis) {
			traj_file_data_t row{};
			if (loader.read_data(row) < 0) { return -1; }
			if (row.i_int != segment || row.i_dof != axis
			    || !PX4_ISFINITE(row.t_int) || row.t_int <= 0.0f) {
				PX4_WARN("Invalid trajectory segment %u axis %u", (unsigned)segment, (unsigned)axis);
				return -1;
			}
			if (axis == 0) { data.durations(segment) = row.t_int; }
			else if (fabsf(row.t_int - data.durations(segment)) > 1.e-5f) {
				PX4_WARN("Inconsistent segment duration");
				return -1;
			}
			for (size_t coefficient = 0; coefficient < data.n_coeffs; ++coefficient) {
				if (!PX4_ISFINITE(row.coefs[coefficient])) {
					PX4_WARN("Nonfinite trajectory coefficient");
					return -1;
				}
				data.coefficients(segment)(axis)(coefficient) = row.coefs[coefficient];
			}
		}
		const float next = total + data.durations(segment);
		if (!PX4_ISFINITE(next) || next <= total) { return -1; }
		total = next;
	}
	return 0;
}

int trajectory::load()
{
	return set_src(file_loader.get_dir(), file_loader.get_file());
}

// #define DEBUG
int trajectory::update_vehicle_state()
{
	vehicle_state.reset();
	vehicle_state.start();
	if (_input_type == 1) {
		_sim_inbound_sub.update(&sm_inbound);
		if (sm_inbound.timestamp == 0 || hrt_elapsed_time(&sm_inbound.timestamp) > input_timeout_us) { return -1; }
		for (int i = 0; i < 3; ++i) {
			vehicle_state.pos(i) = sm_inbound.data[XYZ_OFFSET_START_IND + i];
			vehicle_state.vel(i) = sm_inbound.data[XYZ_VEL_OFFSET_START_IND + i];
			vehicle_state.acc(i) = sm_inbound.data[XYZ_ACC_OFFSET_START_IND + i];
		}
		matrix::Quatf attitude(&sm_inbound.data[QUAT_OFFSET_START_IND]);
		if (!attitude.isAllFinite() || attitude.norm() < 0.5f) { return -1; }
		attitude.normalize();
		vehicle_state.pos(3) = matrix::Eulerf(attitude).psi();
	} else {
		_vehicle_local_position_sub.update(&vehicle_local_position);
		const auto &position = vehicle_local_position;
		// heading_good_for_control requires in-flight magnetic alignment in EKF2.
		// Ground-start guidance needs the current finite yaw, not that post-takeoff flag.
		if (position.timestamp == 0 || hrt_elapsed_time(&vehicle_local_position.timestamp) > input_timeout_us
		    || !position.xy_valid || !position.z_valid) { return -1; }
		if (_reset_counters_valid && _home_valid) {
			const bool xy_reset = position.xy_reset_counter != _xy_reset;
			const bool z_reset = position.z_reset_counter != _z_reset;
			const bool yaw_reset = position.heading_reset_counter != _heading_reset;
			if (xy_reset || z_reset || yaw_reset) {
				// A missed reset delta cannot reconstruct the old coordinate frame.
				if (_use_companion
				    || (xy_reset && uint8_t(position.xy_reset_counter - _xy_reset) != 1)
				    || (z_reset && uint8_t(position.z_reset_counter - _z_reset) != 1)
				    || (yaw_reset && uint8_t(position.heading_reset_counter - _heading_reset) != 1)) {
					_home_valid = false;
					fail("Estimator frame changed; restart guidance");
				} else {
					matrix::Vector<float, 4> delta{};
					if (xy_reset) { delta(0) = position.delta_xy[0]; delta(1) = position.delta_xy[1]; }
					if (z_reset) { delta(2) = position.delta_z; }
					if (yaw_reset) { delta(3) = position.delta_heading; }
					if (!delta.isAllFinite()) {
						_home_valid = false;
						fail("Invalid estimator reset delta");
					} else {
						setpoint_initial.pos += delta;
						setpoint_current.pos += delta;
					}
				}
			}
		}
		_xy_reset = position.xy_reset_counter;
		_z_reset = position.z_reset_counter;
		_heading_reset = position.heading_reset_counter;
		_reset_counters_valid = true;
		const float pos[4] = {position.x, position.y, position.z, position.heading};
		vehicle_state.pos = matrix::Vector<float, 4>(pos);
		const float vel[4] = {position.vx, position.vy, position.vz, 0.0f};
		vehicle_state.vel = matrix::Vector<float, 4>(vel);
		const float acc[4] = {position.ax, position.ay, position.az, 0.0f};
		vehicle_state.acc = matrix::Vector<float, 4>(acc);
	}
	// Angular derivatives are not needed for a stationary home; body r is not Euler yaw rate.
	return vehicle_state.pos.isAllFinite() ? 0 : -1;
}

int trajectory::publish_trajectory_setpoint(float time_trajectory_s)
{
	const int32_t output_type_mask = _output_mask;
	const size_t output_dofs = _use_companion ? 3 : n_dofs;
	if (!_home_valid || !setpoint_current.pos.isAllFinite() || !setpoint_current.vel.isAllFinite()
	    || !setpoint_current.acc.isAllFinite() || !setpoint_current.jerk.isAllFinite()
	    || !setpoint_current.snap.isAllFinite() || !PX4_ISFINITE(time_trajectory_s)
	    || (output_type_mask & 7) == 0) { return -1; }

	// ==========================================
	// TRAJECTORY_SETPOINT (Standard PX4 3D/4D)
	// ==========================================
	if (output_type_mask & (1 << 0)) {
		trajectory_setpoint_s tr_sp{};

		// Loop through standard 3D elements (X=0, Y=1, Z=2)
		for (size_t i = 0; i < 3; i++)
		{
			if (i < output_dofs) {
				// Dof is valid: Publish initial point baseline + trajectory offset
				tr_sp.position[i]     = static_cast<float>(setpoint_current.pos(i));
				tr_sp.velocity[i]     = static_cast<float>(setpoint_current.vel(i));
				tr_sp.acceleration[i] = static_cast<float>(setpoint_current.acc(i));
				tr_sp.jerk[i]         = static_cast<float>(setpoint_current.jerk(i));
			} else {
				// Dof exceeds trajectory allocation: Fall back strictly to initial coordinates
				tr_sp.position[i]     = static_cast<float>(setpoint_initial.pos(i));
				tr_sp.velocity[i]     = static_cast<float>(setpoint_initial.vel(i));
				tr_sp.acceleration[i] = static_cast<float>(setpoint_initial.acc(i));
				tr_sp.jerk[i]         = static_cast<float>(setpoint_initial.jerk(i));
			}
		}

		// Yaw Guidance Check (Index 3 represents Yaw in a 4-DOF vector map)
		if (output_dofs > 3) {
			// execute() already adds the initial yaw; derivatives are trajectory rates.
			tr_sp.yaw      = matrix::wrap_pi(static_cast<float>(setpoint_current.pos(3)));
			tr_sp.yawspeed = static_cast<float>(setpoint_current.vel(3));
		} else {
			tr_sp.yaw      = matrix::wrap_pi(static_cast<float>(setpoint_initial.pos(3)));
			tr_sp.yawspeed = static_cast<float>(setpoint_initial.vel(3));
		}

		tr_sp.timestamp = hrt_absolute_time();
		_trajectory_setpoint_pub.publish(tr_sp);
	}

	// ==========================================
	// SIM_GUIDANCE_TRAJECTORY (Custom uORB)
	// ==========================================
	if (output_type_mask & (1 << 1)) {
		sim_guidance_trajectory_s smg_traj{};
		smg_traj.time_s = static_cast<float>(time_trajectory_s);
		smg_traj.n_dofs = static_cast<uint8_t>(output_dofs);

		// Fill array boundaries completely up to maximum capacity
		for (size_t i = 0; i < n_dofs_max; i++)
		{
			if (i < output_dofs) {
				smg_traj.position[i]     = static_cast<float>(setpoint_current.pos(i));
				smg_traj.velocity[i]     = static_cast<float>(setpoint_current.vel(i));
				smg_traj.acceleration[i] = static_cast<float>(setpoint_current.acc(i));
				smg_traj.jerk[i]         = static_cast<float>(setpoint_current.jerk(i));
				smg_traj.snap[i]         = static_cast<float>(setpoint_current.snap(i));
			} else {
				smg_traj.position[i]     = static_cast<float>(setpoint_initial.pos(i));
				smg_traj.velocity[i]     = static_cast<float>(setpoint_initial.vel(i));
				smg_traj.acceleration[i] = static_cast<float>(setpoint_initial.acc(i));
				smg_traj.jerk[i]         = static_cast<float>(setpoint_initial.jerk(i));
				smg_traj.snap[i]         = static_cast<float>(setpoint_initial.snap(i));
			}
		}

		#ifdef DEBUG
		// Print header timestamp row safely under PX4 character buffer limits
		PX4_INFO("--- TRAJECTORY SMG TIMESTEP t = %9.6f ---", (double)smg_traj.time_s);

		// Axis-by-axis clean row outputs [Pos, Vel, Acc, Jerk, Snap]
		PX4_INFO("  X-AXIS : pos=%7.3f | vel=%7.3f | acc=%7.3f | jerk=%7.3f | snap=%7.3f",
			(double)smg_traj.position[0], (double)smg_traj.velocity[0], (double)smg_traj.acceleration[0], (double)smg_traj.jerk[0], (double)smg_traj.snap[0]);

		PX4_INFO("  Y-AXIS : pos=%7.3f | vel=%7.3f | acc=%7.3f | jerk=%7.3f | snap=%7.3f",
			(double)smg_traj.position[1], (double)smg_traj.velocity[1], (double)smg_traj.acceleration[1], (double)smg_traj.jerk[1], (double)smg_traj.snap[1]);

		PX4_INFO("  Z-AXIS : pos=%7.3f | vel=%7.3f | acc=%7.3f | jerk=%7.3f | snap=%7.3f",
			(double)smg_traj.position[2], (double)smg_traj.velocity[2], (double)smg_traj.acceleration[2], (double)smg_traj.jerk[2], (double)smg_traj.snap[2]);

		PX4_INFO("  YAW-AXIS: pos=%7.3f | vel=%7.3f | acc=%7.3f | jerk=%7.3f | snap=%7.3f",
			(double)smg_traj.position[3], (double)smg_traj.velocity[3], (double)smg_traj.acceleration[3], (double)smg_traj.jerk[3], (double)smg_traj.snap[3]);
		#endif


		smg_traj.timestamp = hrt_absolute_time();
		_sim_guidance_trajecotry_pub.publish(smg_traj);
	}

	// ==========================================
	// SIMULINK_GUIDANCE
	// ==========================================
	if (output_type_mask & (1 << 2)) {
		debug_array_s smg{};
		smg.id = debug_array_s::SIMULINK_GUIDANCE_ID;
		char message_name[10] = "guidance";
		memcpy(smg.name, message_name, sizeof(message_name));
		smg.name[sizeof(smg.name) - 1] = '\0'; // enforce null termination

		smg.timestamp = hrt_absolute_time();
		size_t tmp_ind = 0;

		smg.data[tmp_ind] = static_cast<float>(status.finished);
		tmp_ind++;
		for (size_t i = 0; i < n_dofs_max; i++)
		{
			smg.data[tmp_ind] = static_cast<float>(setpoint_current.pos(i));
			tmp_ind++;
		}
		for (size_t i = 0; i < n_dofs_max; i++)
		{
			smg.data[tmp_ind] = static_cast<float>(setpoint_current.vel(i));
			tmp_ind++;
		}
		for (size_t i = 0; i < n_dofs_max; i++)
		{
			smg.data[tmp_ind] = static_cast<float>(setpoint_current.acc(i));
			tmp_ind++;
		}
		for (size_t i = 0; i < n_dofs_max; i++)
		{
			smg.data[tmp_ind] = static_cast<float>(setpoint_current.jerk(i));
			tmp_ind++;
		}
		for (size_t i = 0; i < n_dofs_max; i++)
		{
			smg.data[tmp_ind] = static_cast<float>(setpoint_current.snap(i));
			tmp_ind++;
		}
		_sim_guidance_pub.publish(smg);
	}
	return 0;
}


int trajectory::execute(void)
{
	double time_trajectory_s = setpoint_initial.get_time_s();
	setpoint_current.start();

	matrix::Vector<float, n_dofs_max> derivatives[5];
	const int res = guidance::evaluate(coefs, tof_int, n_coeffs, n_dofs, n_int, time_trajectory_s, derivatives);
	if (res < 0) return -1;
	else if (res == 1)
	{
		status.executing = false;
		status.finished = true;
		PX4_INFO("Completed trajectory execution");
	}

	setpoint_current.start();
	setpoint_current.pos = setpoint_initial.pos + derivatives[0];
	setpoint_current.vel = derivatives[1];
	setpoint_current.acc = derivatives[2];
	setpoint_current.jerk = derivatives[3];
	setpoint_current.snap = derivatives[4];

	#ifdef DEBUG
	// Print header timestamp row safely under PX4 character buffer limits
	PX4_INFO("--- TRAJECTORY SETPOINT TIMESTEP t = %9.6f ---", time_trajectory_s);

	// Axis-by-axis clean row outputs [Pos, Vel, Acc, Jerk, Snap]
	PX4_INFO("  X-AXIS : pos=%7.3f | vel=%7.3f | acc=%7.3f | jerk=%7.3f | snap=%7.3f",
		(double)setpoint_current.pos(0), (double)setpoint_current.vel(0), (double)setpoint_current.acc(0), (double)setpoint_current.jerk(0), (double)setpoint_current.snap(0));

	PX4_INFO("  Y-AXIS : pos=%7.3f | vel=%7.3f | acc=%7.3f | jerk=%7.3f | snap=%7.3f",
		(double)setpoint_current.pos(1), (double)setpoint_current.vel(1), (double)setpoint_current.acc(1), (double)setpoint_current.jerk(1), (double)setpoint_current.snap(1));

	PX4_INFO("  Z-AXIS : pos=%7.3f | vel=%7.3f | acc=%7.3f | jerk=%7.3f | snap=%7.3f",
		(double)setpoint_current.pos(2), (double)setpoint_current.vel(2), (double)setpoint_current.acc(2), (double)setpoint_current.jerk(2), (double)setpoint_current.snap(2));

	PX4_INFO("  YAW-AXIS: pos=%7.3f | vel=%7.3f | acc=%7.3f | jerk=%7.3f | snap=%7.3f",
		(double)setpoint_current.pos(3), (double)setpoint_current.vel(3), (double)setpoint_current.acc(3), (double)setpoint_current.jerk(3), (double)setpoint_current.snap(3));
	#endif



	return publish_trajectory_setpoint(time_trajectory_s);
}

int trajectory::update_from_companion()
{
	debug_array_s message{};
	if (!_companion_guidance_outbound_sub.update(&message)) { return 0; }
	if (message.timestamp == 0 || hrt_elapsed_time(&message.timestamp) > companion_timeout_us) { return -1; }
	_companion_timestamp = message.timestamp;
	const bool alive = PX4_ISFINITE(message.data[0]) && message.data[0] > 0.1f;
	status.loaded = alive;
	if (!alive) { return status.executing ? -1 : 0; }
	if (!status.executing) { return 0; }

	// The companion wire format has exactly three axes and absolute NED positions.
	pointf next{};
	next.pos(3) = setpoint_initial.pos(3);
	size_t index = 3;
	matrix::Vector<float, 4> *vectors[] = {&next.pos, &next.vel, &next.acc, &next.jerk, &next.snap};
	for (auto *vector : vectors) {
		for (int axis = 0; axis < 3; ++axis) { (*vector)(axis) = message.data[index++]; }
		if (!vector->isAllFinite()) { return -1; }
	}
	if (!PX4_ISFINITE(message.data[1]) || !PX4_ISFINITE(message.data[2]) || message.data[2] < 0.0f) { return -1; }
	setpoint_current = next;
	// Consume the endpoint even when the same packet marks execution complete.
	const bool finished = message.data[1] > 0.1f;
	if (finished) { status.executing = false; status.finished = true; }
	return publish_trajectory_setpoint(message.data[2]);
}

int trajectory::update_companion(bool request_start, bool request_stop, bool request_start_executing)
{
	debug_array_s _companion_guidance_inbound{};
	_companion_guidance_inbound.timestamp = hrt_absolute_time();
	_companion_guidance_inbound.id = debug_array_s::COMPANION_GUIDANCE_INBOUND_ID;
	char message_name[10] = "compg_in";
	memcpy(_companion_guidance_inbound.name, message_name, sizeof(message_name));
	_companion_guidance_inbound.name[sizeof(_companion_guidance_inbound.name) - 1] = '\0'; // enforce null termination

	int tmp_ind = 0;
	static const size_t companion_max_dof = 3;

	if (request_stop) PX4_INFO("Sending stop request to the companion...");
	if (request_start)
	{
		PX4_INFO("Sending start request to the companion...");
		PX4_INFO("Position = [%f, %f, %f, %f], Reference = [%f, %f, %f, %f]",\
		 (double)vehicle_state.pos(0), (double)vehicle_state.pos(1), (double)vehicle_state.pos(2), (double)vehicle_state.pos(3),\
		 (double)setpoint_current.pos(0), (double)setpoint_current.pos(1), (double)setpoint_current.pos(2), (double)setpoint_current.pos(3));
	}
	if (request_start_executing) PX4_INFO("Sending start execution request to the companion...");

	_companion_guidance_inbound.data[tmp_ind] = static_cast<float>(request_start);
	tmp_ind++;
	_companion_guidance_inbound.data[tmp_ind] = static_cast<float>(request_start_executing);
	tmp_ind++;
	_companion_guidance_inbound.data[tmp_ind] = static_cast<float>(request_stop);
	tmp_ind++;

	for (size_t i = 0; i < companion_max_dof; i++)
	{
		_companion_guidance_inbound.data[tmp_ind] = setpoint_current.pos(i);
		tmp_ind++;
	}
	for (size_t i = 0; i < companion_max_dof; i++)
	{
		_companion_guidance_inbound.data[tmp_ind] = setpoint_current.vel(i);
		tmp_ind++;
	}
	for (size_t i = 0; i < companion_max_dof; i++)
	{
		_companion_guidance_inbound.data[tmp_ind] = setpoint_current.acc(i);
		tmp_ind++;
	}
	for (size_t i = 0; i < companion_max_dof; i++)
	{
		_companion_guidance_inbound.data[tmp_ind] = setpoint_current.jerk(i);
		tmp_ind++;
	}
	for (size_t i = 0; i < companion_max_dof; i++)
	{
		_companion_guidance_inbound.data[tmp_ind] = setpoint_current.snap(i);
		tmp_ind++;
	}

	for (size_t i = 0; i < companion_max_dof; i++)
	{
		_companion_guidance_inbound.data[tmp_ind] = static_cast<float>(vehicle_state.pos(i));
		tmp_ind++;
	}
	_companion_guidance_inbound_pub.publish(_companion_guidance_inbound);
	return 0;
}


void trajectory::start(void)
{
	if (!status.started || status.finished)
	{
		reset();
		PX4_INFO("Initializing trajectory...");
		status.started = true;
	}

	return;
}
void trajectory::reset(void)
{
	setpoint_initial.reset();
	setpoint_current.reset();
	_home_valid = false;
	status.started = false;
	status.executing = false;
	status.finished = false;
	status.trajectory_valid = false;
	return;
}

int trajectory::set_src(const char* _file)
{
	return set_src(file_loader.get_dir(), _file);
}

int trajectory::set_src(const char* directory, const char* filename)
{
	if (status.executing) {
		PX4_WARN("Stop trajectory execution before changing source");
		return -1;
	}
	// Loading is serialized by the module mutex. Stage off-stack; retain the last
	// valid trajectory and its source if any part of this replacement fails.
	file_loader_backend candidate;
	if (candidate.set_src(filename, directory) < 0) { return -1; }
	LoadedTrajectory *data = new LoadedTrajectory{};
	if (!data) { return -1; }
	if (read_trajectory(candidate, *data) < 0) { delete data; return -1; }
	candidate.close_file();
	if (file_loader.set_src(candidate.get_file(), candidate.get_dir()) < 0) { delete data; return -1; }
	coefs = data->coefficients;
	tof_int = data->durations;
	n_coeffs = data->n_coeffs;
	n_int = data->n_int;
	n_dofs = data->n_dofs;
	delete data;
	_local_loaded = true;
	if (!_use_companion) { status.loaded = true; }
	PX4_INFO("Trajectory successfully loaded");
	return 0;
}

void trajectory::print_status(void)
{
	PX4_INFO("Latched on to %s trajectory file in %s", file_loader.get_file(), file_loader.get_dir());

	PX4_INFO("Guidance Internal Status Report:");
	if (status.started) 	PX4_INFO("%-20s%10s", "Started:", "true");
	else 			PX4_INFO("%-20s%10s", "Started:", "false");

	if (status.loaded) 	PX4_INFO("%-20s%10s", "Loaded:", "true");
	else 			PX4_INFO("%-20s%10s", "Loaded:", "false");

	if (status.executing) 	PX4_INFO("%-20s%10s", "Executing:", "true");
	else 			PX4_INFO("%-20s%10s", "Executing:", "false");

	if (status.trajectory_valid) \
				PX4_INFO("%-20s%10s", "Trajectory valid:", "true");
	else 			PX4_INFO("%-20s%10s", "Trajectory valid:", "false");

	if (status.finished) 	PX4_INFO("%-20s%10s", "Finished:", "true");
	else 			PX4_INFO("%-20s%10s", "Finished:", "false");


	PX4_INFO("Latest Trajectory Parameters:");
	PX4_INFO("%-45s %u / %u", "Number of coefficients for each segment:", static_cast<uint16_t>(n_coeffs), static_cast<uint16_t>(n_coeffs_max));
	PX4_INFO("%-45s %u / %u", "Number of segments:", static_cast<uint16_t>(n_int), static_cast<uint16_t>(n_int_max));
	PX4_INFO("%-45s %u / %u", "Number of active degrees of freedom:", static_cast<uint16_t>(n_dofs), static_cast<uint16_t>(n_dofs_max));
	return;
}

trajectory::trajectory(/* args */)
{
}

trajectory::~trajectory()
{

}
