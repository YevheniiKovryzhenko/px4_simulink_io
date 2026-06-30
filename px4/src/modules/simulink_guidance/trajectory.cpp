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

static const int XYZ_OFFSET_START_IND = 18 + 17; //control vector size + number of states before xyz
static const int XYZ_VEL_OFFSET_START_IND = 18 + 0; //control vector size + number of states before xyz velocity
static const int QUAT_OFFSET_START_IND = 18 + 6; //control vector size + number of states before quat
static const int XYZ_ACC_OFFSET_START_IND = 18 + 14; //control vector size + number of states before accel
static const int PQR_OFFSET_START_IND = 18 + 3; //control vector size + number of states before pqr

double get_dt_s_hrt(hrt_abstime &time_stamp)
{
	return static_cast<double>(hrt_elapsed_time(&time_stamp))*1.0E-6;
}

template <typename Type, size_t n_coeffs, size_t n_dofs, size_t n_int>
void assign_coefs2matrix(matrix::Vector<matrix::Vector<matrix::Vector<Type, n_coeffs>, n_dofs>, n_int> &coeffs,\
size_t i_int, size_t i_dof, Type* input_1Darray, size_t n_coeffs_in)
{
	for (size_t i_coeff = 0; i_coeff < n_coeffs_in && i_coeff < n_coeffs; i_coeff++) coeffs(i_int)(i_dof)(i_coeff) = input_1Darray[i_coeff];
	return;
}

template <typename Type, size_t n_coeffs, size_t n_dofs, size_t n_int>
void assign_coefs2matrix(matrix::Vector<matrix::Vector<matrix::Vector<Type, n_coeffs>, n_dofs>, n_int> &coeffs,\
traj_file_data_t& data_in, size_t n_coeffs_in)
{
	for (size_t i_coeff = 0; i_coeff < n_coeffs_in && i_coeff < n_coeffs; i_coeff++) coeffs(data_in.i_int)(data_in.i_dof)(i_coeff) = data_in.coefs[i_coeff];
	return;
}

template <typename Type, size_t n_coeffs>
Type poly_val(matrix::Vector<Type, n_coeffs> &coeffs, Type time_int_s, Type tof_int_s, uint8_t deriv_order, size_t n_coeffs_in)
{
	Type out = static_cast<Type>(0.0);
	double tau = static_cast<double>(time_int_s) / static_cast<double>(tof_int_s);

	Type scaling = static_cast<Type>(pow(1.0 / static_cast<double>(tof_int_s), static_cast<double>(deriv_order)));
	if (n_coeffs_in > n_coeffs) n_coeffs_in = n_coeffs;

	if (deriv_order == 0) for (size_t i = 0; i < n_coeffs_in; i++) out += coeffs(i)*static_cast<Type>(pow(tau,static_cast<double>(i)));
	else
	{
		for (size_t i = static_cast<size_t>(deriv_order); i < n_coeffs_in; i++)
		{
			Type prod__ = static_cast<Type>(1.0);
			for (size_t ii = i - static_cast<size_t>(deriv_order) + 1; ii < i + 1; ii++) prod__ *= static_cast<Type>(ii);

			out += coeffs(i)*prod__*static_cast<Type>(pow(tau,static_cast<double>(i - static_cast<size_t>(deriv_order))));
		}
	}
	out *= scaling;
	return out;
}

template <typename Type, size_t n_coefs, size_t n_dofs, size_t n_int>
int eval_traj(matrix::Vector<Type, n_dofs> &eval_vec, Type time_trajectory_s, matrix::Vector<matrix::Vector<matrix::Vector<Type, n_coefs>, n_dofs>, n_int> &coeffs,\
 		matrix::Vector<Type, n_int> &tof_int_s, uint8_t deriv_order,\
		size_t n_coeffs_in, size_t n_dofs_in, size_t n_int_in)
{

	//locate segment number and tof for this segment:
	size_t i_int;
	if (n_int_in > n_int) n_int_in = n_int;
	Type TOF_max = static_cast<Type>(0.0);
	Type time_int_s = static_cast<Type>(0.0);
	for (size_t i = 0; i < n_int_in; i++) TOF_max += tof_int_s(i);

	int res = 0;

	if (time_trajectory_s < static_cast<Type>(0.0))
	{
		i_int = 0;
		//time_int_s = static_cast<Type>(0.0);
	}
	else if (time_trajectory_s >= TOF_max)
	{
		i_int = n_int_in - 1;
		time_int_s = tof_int_s(i_int);
		res = 1;
	}
	else
	{
		Type tof_sum = static_cast<Type>(0.0);
		i_int = 0;
		for (size_t i = 0; i < n_int_in; i++)
		{
			tof_sum += tof_int_s(i);
			if (time_trajectory_s > tof_sum) i_int++;
			else break;
		}

		if (i_int > n_int_in - 1)
		{
			i_int = n_int_in - 1;
			time_int_s = tof_int_s(i_int);
		}
		else if (i_int > 0)
		{
			time_int_s = time_trajectory_s;
			for (size_t i = 0; i < i_int; i++) time_int_s -= tof_int_s(i);
		}
		else time_int_s = time_trajectory_s;
	}

	if (n_dofs_in > n_dofs) n_dofs_in = n_dofs;

	for (size_t i_dof = 0; i_dof < n_dofs_in; i_dof++)
	{
		eval_vec(i_dof) = poly_val<Type,n_coefs>(coeffs(i_int)(i_dof), time_int_s, tof_int_s(i_int), deriv_order, n_coeffs_in);
	}

	return res;
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
	setpoint_current.reset();
	setpoint_current = vehicle_state;
	setpoint_current.start();

	setpoint_initial = setpoint_current;
	return publish_trajectory_setpoint(0.0);
}

void trajectory::update(bool use_companion)
{
	//if using companion, we need to process it first
	bool already_sent_request = false;
	if (use_companion) update_from_companion(); //also updates status flags
	update_vehicle_state();
	_vehicle_local_position_sub.update(&vehicle_local_position);

	//check the requests:
	sim_guidance_request_s smg_request{};
	if (_sim_guidance_request_sub.update(&smg_request))//new request has been published, need to process
	{
		if (smg_request.reset || smg_request.start || smg_request.stop || smg_request.start_execution) //ignore if all is false (no real requests)
		{
			if (smg_request.stop)
			{
				status.finished = true; //this this as early termination
				if (use_companion)
				{
					update_companion(false, true);
					status.loaded = false;
				}
				already_sent_request = true;
			}
			if (smg_request.reset)
			{
				reset(); //check reset flag first
				if (use_companion)
				{
					update_companion(false, true);
					status.loaded = false;
				}
				already_sent_request = true;
			}
			if (smg_request.set_home) //process manually-requested set_home (even if disabled)
			{
				set_home(); //make sure we have the most recent state first, also sets home
				if (use_companion)
				{
					update_companion(false, true);
					status.loaded = false;
				}
				//this will only publish once, if trajectory execution is not enabled
				//won't do much, but is usefull if trying to capture current state
				already_sent_request = true;
			}
			if (smg_request.start) //this is when we have received the first request (assume pos_hold is not yet enabled, but will be as we send back ack)
			{
				start(); //start if not started yet
				set_home(); //make sure we have the most recent state first, also sets home
				if (use_companion) update_companion(true);
				//assume pos_hold will latch on this point, so we won't update the ref untill trajectory is executed or reset
				already_sent_request = true;
			}


			if (!smg_request.start && !smg_request.reset && !smg_request.stop && smg_request.start_execution \
				&& status.loaded && status.started && !status.finished)
			{
				if(use_companion) update_companion(true, false, true);
				status.executing = true; //trajectory is fully loaded and ready, so start evaluation
				PX4_INFO("Started trajectory execution");
				setpoint_initial.start(); //starts the timer for trajectory execuition
				already_sent_request = true;
			}

			sim_guidance_request_s smg_request_ack{};
			smg_request_ack.reset = false;
			smg_request_ack.start = false;
			smg_request_ack.stop = false;
			smg_request_ack.start_execution = false;
			smg_request_ack.set_home = false;
			smg_request_ack.timestamp = hrt_absolute_time();

			_sim_guidance_request_pub.publish(smg_request_ack);
		}

	}//otherwise keep doing stuff


	if (status.started && !status.finished)
	{
		if (use_companion)
		{
			status.trajectory_valid = true; //this should be handled on the other side
			if (!already_sent_request) update_companion();
		}
		else
		{
			if (!status.loaded)
			{
				status.trajectory_valid = false;
				if (load() < 0)
				//if (load_dummy_data() < 0)
				{
					PX4_INFO("Failed to load trajectory, disengaging guidance...");
					status.finished = true;
				}
			}
			else
			{
				if (status.executing && execute() < 0)
				{
					PX4_INFO("Failed to execute trajectory, disengaging guidance...");
					status.finished = true;
					status.executing = false;
				}
				else status.trajectory_valid = true;
			}
		}
	}
	else
	{
		status.executing = false;
		status.trajectory_valid = false;
		//don't update the reference since it has home + ref until guidance is properly reset
	}

	//publish guidance status:
	sim_guidance_status_s smg_status{};
	smg_status.started = status.started;
	smg_status.loaded = status.loaded;
	smg_status.executing = status.executing;
	smg_status.finished = status.finished;
	smg_status.trajectory_valid = status.trajectory_valid;
	smg_status.timestamp = hrt_absolute_time();

	_sim_guidance_status_pub.publish(smg_status);


	return;
}

int trajectory::load_dummy_data(void)
{
	file_loader.close_file(); //do this no matter what
	PX4_INFO("Begin trajectory loading sequence...");
	//read first row to get the settings of the trajectory:
	traj_file_header_t traj_header{};
	if (file_loader.read_dummy_header(traj_header) < 0) return -1;
	n_coeffs = static_cast<size_t>(traj_header.n_coeffs);
	n_int = static_cast<size_t>(traj_header.n_int);
	n_dofs = static_cast<size_t>(traj_header.n_dofs);

	//check if all good:
	if (n_coeffs > n_coeffs_max)
	{
		PX4_WARN("Too many coefficients");
		status.loaded = false;
		file_loader.close_file();
		return -1;
	}
	if (n_int > n_int_max)
	{
		PX4_WARN("Too many segments");
		status.loaded = false;
		file_loader.close_file();
		return -1;
	}
	if (n_dofs > n_dofs_max)
	{
		PX4_WARN("Too many dofs");
		status.loaded = false;
		file_loader.close_file();
		return -1;
	}
	if (n_coeffs == 0 || n_int == 0 || n_dofs == 0)
	{
		PX4_WARN("Invalid trajectory (zeros in the settings)");
		status.loaded = false;
		file_loader.close_file();
		return -1;
	}

	//load the trajectory data (we need coefficients and time allocated for each interval)
	tof_int.setZero();
	coefs.setZero();
	matrix::Vector<DATATYPE_TRAJ, n_dofs_max> tof_int_i;
	for (size_t i_int = 0; i_int < n_int; i_int++)
	{
		tof_int_i.setZero();
		for (size_t i_dof = 0; i_dof < n_dofs; i_dof++)
		{
			traj_file_data_t traj_data{};
			if (file_loader.read_dummy_data(traj_data, i_int, i_dof) < 0) return -1;

			//perform additional checks:
			if (static_cast<size_t>(traj_data.i_dof) != i_dof)
			{
				PX4_ERR("Error in the trajectory loading: i_dof for i_int=%u, i_dof=%u does not match the file.",static_cast<uint16_t>(i_int), static_cast<uint16_t>(i_dof));
				status.loaded = false;
				file_loader.close_file();
				return -1;
			}
			if (static_cast<size_t>(traj_data.i_int) != i_int)
			{
				PX4_ERR("Error in the trajectory loading: i_int for i_int=%u, i_dof=%u does not match the file.", static_cast<uint16_t>(i_int), static_cast<uint16_t>(i_dof));
				status.loaded = false;
				file_loader.close_file();
				return -1;
			}
			//that's all we can do for the data (as of right now)
			assign_coefs2matrix<DATATYPE_TRAJ, n_coeffs_max, n_dofs_max, n_int_max>(coefs, traj_data, n_coeffs);
			tof_int_i(i_dof) = traj_data.t_int;
		}
		if (n_dofs > 1)
		{
			for (size_t i_dof = 0; i_dof < n_dofs-1; i_dof++)
			{
				if (fabsf(static_cast<float>(tof_int_i(i_dof) - tof_int_i(i_dof+1))) > 1.0E-5f)
				{
					PX4_ERR("Error in the trajectory loading: i_int=%u, t_int does not match accross all dofs.", static_cast<uint16_t>(i_int));
					status.loaded = false;
					file_loader.close_file();
					return -1;
				}
			}
		}
		tof_int(i_int) = tof_int_i(0);


	}

	status.loaded = true;
	file_loader.close_file();
	PX4_INFO("Trajectory successfully loaded!");
	return 0;
}

int trajectory::load(void)
{
	file_loader.close_file(); //do this no matter what
	PX4_INFO("Begin trajectory loading sequence...");
	//read first row to get the settings of the trajectory:
	traj_file_header_t traj_header{};
	if (file_loader.read_header(traj_header) < 0) return -1;
	n_coeffs = static_cast<size_t>(traj_header.n_coeffs);
	n_int = static_cast<size_t>(traj_header.n_int);
	n_dofs = static_cast<size_t>(traj_header.n_dofs);

	//check if all good:
	if (n_coeffs > n_coeffs_max)
	{
		PX4_WARN("Too many coefficients");
		status.loaded = false;
		file_loader.close_file();
		return -1;
	}
	if (n_int > n_int_max)
	{
		PX4_WARN("Too many segments");
		status.loaded = false;
		file_loader.close_file();
		return -1;
	}
	if (n_dofs > n_dofs_max)
	{
		PX4_WARN("Too many dofs");
		status.loaded = false;
		file_loader.close_file();
		return -1;
	}
	if (n_coeffs == 0 || n_int == 0 || n_dofs == 0)
	{
		PX4_WARN("Invalid trajectory (zeros in the settings)");
		status.loaded = false;
		file_loader.close_file();
		return -1;
	}

	//load the trajectory data (we need coefficients and time allocated for each interval)
	tof_int.setZero();
	coefs.setZero();
	matrix::Vector<DATATYPE_TRAJ, n_dofs_max> tof_int_i;
	for (size_t i_int = 0; i_int < n_int; i_int++)
	{
		tof_int_i.setZero();
		for (size_t i_dof = 0; i_dof < n_dofs; i_dof++)
		{
			traj_file_data_t traj_data{};
			if (file_loader.read_data(traj_data) < 0) return -1;

			//perform additional checks:
			if (static_cast<size_t>(traj_data.i_dof) != i_dof)
			{
				PX4_ERR("Error in the trajectory loading: i_dof for i_int=%u, i_dof=%u does not match the file.",static_cast<uint16_t>(i_int), static_cast<uint16_t>(i_dof));
				status.loaded = false;
				file_loader.close_file();
				return -1;
			}
			if (static_cast<size_t>(traj_data.i_int) != i_int)
			{
				PX4_ERR("Error in the trajectory loading: i_int for i_int=%u, i_dof=%u does not match the file.", static_cast<uint16_t>(i_int), static_cast<uint16_t>(i_dof));
				status.loaded = false;
				file_loader.close_file();
				return -1;
			}
			//that's all we can do for the data (as of right now)
			assign_coefs2matrix<DATATYPE_TRAJ, n_coeffs_max, n_dofs_max, n_int_max>(coefs, traj_data, n_coeffs);
			tof_int_i(i_dof) = traj_data.t_int;
		}
		if (n_dofs > 1)
		{
			for (size_t i_dof = 0; i_dof < n_dofs-1; i_dof++)
			{
				if (fabsf(static_cast<float>(tof_int_i(i_dof) - tof_int_i(i_dof+1))) > 1.0E-5f)
				{
					PX4_ERR("Error in the trajectory loading: i_int=%u, t_int does not match accross all dofs.", static_cast<uint16_t>(i_int));
					status.loaded = false;
					file_loader.close_file();
					return -1;
				}
			}
		}
		tof_int(i_int) = tof_int_i(0);


	}

	status.loaded = true;
	file_loader.close_file();
	PX4_INFO("Trajectory successfully loaded!");
	return 0;
}

// #define DEBUG
int trajectory::update_vehicle_state(void)
{
	static param_t smg_in_type_handle = param_find("SMG_IN_TYPE");
	int32_t input_type = 0;
	if (smg_in_type_handle == PARAM_INVALID || param_get(smg_in_type_handle, &input_type) != OK) {
		return -1;
	}

	vehicle_state.reset();
	vehicle_state.start();

	switch (input_type)
	{
	case 1: //DEBUG_FLOAT_ARRAY
	{
		_sim_inbound_sub.update(&sm_inbound);

		for (int i = 0; i < math::min(static_cast<int>(n_dofs), 2); i++)
		{
			vehicle_state.pos(i) = sm_inbound.data[XYZ_OFFSET_START_IND + i]; //position [x y z]
			vehicle_state.vel(i) = sm_inbound.data[XYZ_VEL_OFFSET_START_IND + i]; //velocity [x y z]
			vehicle_state.acc(i) = sm_inbound.data[XYZ_ACC_OFFSET_START_IND + i]; //acceleration [x y z]
		}
		if (n_dofs > 2)
		{
			vehicle_state.vel(3) = sm_inbound.data[PQR_OFFSET_START_IND + 2]; //yaw rate

			matrix::Quaternion<float> vehicle_attitude_quat(\
			sm_inbound.data[QUAT_OFFSET_START_IND],\
			sm_inbound.data[QUAT_OFFSET_START_IND+1],\
			sm_inbound.data[QUAT_OFFSET_START_IND+2],\
			sm_inbound.data[QUAT_OFFSET_START_IND+3]);

			matrix::Euler<float> vehicle_attitude_eul(vehicle_attitude_quat);
			vehicle_state.pos(3) = vehicle_attitude_eul.psi();		//yaw
		}
		break;
	}
	default: //VEHICLE_LOCAL_POSITION
	{
		_vehicle_local_position_sub.update(&vehicle_local_position);
		_vehicle_angular_velocity_sub.update(&vehicle_angular_velocity);

		vehicle_state.pos(0) = vehicle_local_position.x;
		vehicle_state.pos(1) = vehicle_local_position.y;
		vehicle_state.pos(2) = vehicle_local_position.z;

		vehicle_state.vel(0) = vehicle_local_position.vx;
		vehicle_state.vel(1) = vehicle_local_position.vy;
		vehicle_state.vel(2) = vehicle_local_position.vz;

		vehicle_state.acc(0) = vehicle_local_position.ax;
		vehicle_state.acc(1) = vehicle_local_position.ay;
		vehicle_state.acc(2) = vehicle_local_position.az;

		vehicle_state.pos(3) = vehicle_local_position.heading;
		vehicle_state.vel(3) = vehicle_angular_velocity.xyz[2];
		vehicle_state.acc(3) = vehicle_angular_velocity.xyz_derivative[2];
		break;
	}}

	return 0;
}

int trajectory::publish_trajectory_setpoint(float time_trajectory_s)
{
	static param_t smg_out_type_handle = param_find("SMG_OUT_TYPE");
	int32_t output_type_mask = 0;
	if (smg_out_type_handle == PARAM_INVALID || param_get(smg_out_type_handle, &output_type_mask) != OK) {
		return -1;
	}

	// ==========================================
	// TRAJECTORY_SETPOINT (Standard PX4 3D/4D)
	// ==========================================
	if (output_type_mask & (1 << 0)) {
		trajectory_setpoint_s tr_sp{};

		// Loop through standard 3D elements (X=0, Y=1, Z=2)
		for (size_t i = 0; i < 3; i++)
		{
			if (i < n_dofs) {
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
		if (n_dofs > 3) {
			tr_sp.yaw      = static_cast<float>(setpoint_current.pos(3)) + static_cast<float>(setpoint_initial.pos(3));
			tr_sp.yawspeed = static_cast<float>(setpoint_current.vel(3)) + static_cast<float>(setpoint_initial.vel(3));
		} else {
			tr_sp.yaw      = static_cast<float>(setpoint_initial.pos(3));
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
		smg_traj.n_dofs = static_cast<uint8_t>(n_dofs);

		// Fill array boundaries completely up to maximum capacity
		for (size_t i = 0; i < n_dofs_max; i++)
		{
			if (i < n_dofs) {
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

	pointf offset{};
	offset.reset();

	int res = eval_traj<DATATYPE_TRAJ,n_coeffs_max,n_dofs_max,n_int_max>(offset.pos, time_trajectory_s, coefs, tof_int, 0, n_coeffs, n_dofs, n_int);
	if (eval_traj<DATATYPE_TRAJ,n_coeffs_max,n_dofs_max,n_int_max>(offset.vel, time_trajectory_s, coefs, tof_int, 1, n_coeffs, n_dofs, n_int) < 0) return -1;
	if (eval_traj<DATATYPE_TRAJ,n_coeffs_max,n_dofs_max,n_int_max>(offset.acc, time_trajectory_s, coefs, tof_int, 2, n_coeffs, n_dofs, n_int) < 0) return -1;
	if (eval_traj<DATATYPE_TRAJ,n_coeffs_max,n_dofs_max,n_int_max>(offset.jerk, time_trajectory_s, coefs, tof_int, 3, n_coeffs, n_dofs, n_int) < 0) return -1;
	if (eval_traj<DATATYPE_TRAJ,n_coeffs_max,n_dofs_max,n_int_max>(offset.snap, time_trajectory_s, coefs, tof_int, 4, n_coeffs, n_dofs, n_int) < 0) return -1;
	if (res < 0) return -1;
	else if (res == 1)
	{
		status.executing = false;
		status.finished = true;
		PX4_INFO("Completed trajectory execution");
	}

	setpoint_current.start();
	setpoint_current.pos = setpoint_initial.pos + offset.pos;
	setpoint_current.vel = offset.vel;
	setpoint_current.acc = offset.acc;
	setpoint_current.jerk = offset.jerk;
	setpoint_current.snap = offset.snap;

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

int trajectory::update_from_companion(void)
{

	debug_array_s comp_outbound{};
	if (!_companion_guidance_outbound_sub.update(&comp_outbound)) return 0;

	setpoint_current.reset();
	setpoint_current.start();

	size_t tmp_ind = 0;
	static const int companion_max_dof = 3;

	bool is_alive = comp_outbound.data[tmp_ind] > 0.1f;
	tmp_ind++;
	bool finished = comp_outbound.data[tmp_ind] > 0.1f;
	tmp_ind++;


	if (finished)
	{
		if (status.executing) PX4_INFO("Completed trajectory execution");
		status.executing = false;
		status.finished = true;
	}


	if (status.executing)
	{
		float time_trajectory_s = comp_outbound.data[tmp_ind];
		tmp_ind++;
		for (int i = 0; i < companion_max_dof; i++)
		{
			setpoint_current.pos(i) = comp_outbound.data[tmp_ind];
			tmp_ind++;
		}
		for (int i = 0; i < companion_max_dof; i++)
		{
			setpoint_current.vel(i) = comp_outbound.data[tmp_ind];
			tmp_ind++;
		}
		for (int i = 0; i < companion_max_dof; i++)
		{
			setpoint_current.acc(i) = comp_outbound.data[tmp_ind];
			tmp_ind++;
		}
		for (int i = 0; i < companion_max_dof; i++)
		{
			setpoint_current.jerk(i) = comp_outbound.data[tmp_ind];
			tmp_ind++;
		}
		for (int i = 0; i < companion_max_dof; i++)
		{
			setpoint_current.snap(i) = comp_outbound.data[tmp_ind];
			tmp_ind++;
		}

		publish_trajectory_setpoint(time_trajectory_s);
	}

	status.loaded = is_alive;
	return 0;
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
	if (!status.started)
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

int trajectory::set_src(const char* _dir, const char* _file)
{
	if (file_loader.set_src(_file, _dir))
	{
		PX4_INFO("Failed to set source location at %s for file %s.", _dir, _file);
		return -1;
	}
	else
	{
		//much more efficient to just load file here
		status.loaded = false;
		load();
	}
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
