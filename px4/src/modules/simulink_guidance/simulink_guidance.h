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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
//#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sim_guidance_request.h>
#include "trajectory.hpp"

extern "C" __EXPORT int simulink_guidance_main(int argc, char *argv[]);


class SimulinkGuidance : public ModuleBase<SimulinkGuidance>, public ModuleParams
{
public:

	SimulinkGuidance(int example_param);

	virtual ~SimulinkGuidance() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static SimulinkGuidance *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;


private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);

	void update_guidance(void);


	void load_trajectory_from_params(void);

	trajectory traj{};

	hrt_abstime	_boot_timestamp{0};


	// Subscriptions
	uORB::Subscription		_parameter_update_sub{ORB_ID(parameter_update)};

	// Publications
	uORB::Publication<sim_guidance_request_s>	_sim_guidance_request_pub{ORB_ID(sim_guidance_request)};

	/**
	 * THIS IS WHERE YOU DEFINE NEW PARAMETRS
	 * example:
	 * DEFINE_PARAMETERS(
	 *	(ParamFloat<px4::params::SM_LQI_LG_SAT>) _param_sm_lqi_lg_sat)
	 *
	 * where:
	 *	DEFINE_PARAMETERS is macro (short function)
	 * 	ParamFloat defines the data type of the parameter (float in this case)
	 * 	SM_LQI_LG_SAT is the global name that other app will see (e.g. simulink or QGC). Make sure this is under 14 characters
	 *	_param_sm_lqi_lg_sat is the local variable (used in this app only).
	 *	They MUST BE UPPERCASE AND lowercase respectively (don't mix)
	 */
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SMG_EN>) _param_smg_en,
		(ParamInt<px4::params::SMG_OUT_TYPE>) _param_smg_out_type,
		(ParamInt<px4::params::SMG_TRAJ_DIR>) _params_smg_traj_dir,
		(ParamInt<px4::params::SMG_TRAJ_ID>) _params_smg_traj_id
	)//MAKE SURE EVERY PARAMETER IS FOLLOWED BY "," AND LAST ONE DOES NOT HAVE ANYTHING
};

