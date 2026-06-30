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

#include "simulink_guidance.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <math.h>
#include <uORB/topics/parameter_update.h>
//#include "waypoints.hpp"

int SimulinkGuidance::print_status()
{
	PX4_INFO("Running");
	traj.print_status();

	return 0;
}

int SimulinkGuidance::custom_command(int argc, char *argv[])
{

	if (!is_running()) {
		print_usage("not running");
		return 1;
	}



	// additional custom commands can be handled like this:
	for (int i = 0; i < argc; i++)
	{
		if (!strcmp(argv[i], "set_src")) {
			// Check argument count: need at least 1 argument (filename)
			if (argc < i + 2) {
				PX4_WARN("Usage: set_src <file_name> or set_src <directory> <file_name>");
				print_usage();
				return 0;
			}

			const char *dir_arg = nullptr;
			const char *file_arg = nullptr;

			// Parse arguments: 1 arg = filename only, 2 args = directory + filename
			if (argc >= i + 3) {
				// Two arguments: directory and filename
				dir_arg = argv[i + 1];
				file_arg = argv[i + 2];
			} else {
				// One argument: filename only, use current directory
				dir_arg = get_instance()->traj.file_loader.get_dir();
				file_arg = argv[i + 1];
			}

			// Normalize paths (handles .traj extension, trailing slashes, etc.)
			char normalized_file[256];
			char normalized_dir[256];
			if (get_instance()->traj.file_loader.normalize_paths(
				normalized_file, normalized_dir, file_arg, dir_arg) < 0) {
				PX4_WARN("Failed to normalize file paths");
				return 0;
			}

			// Set source with normalized paths
			if (get_instance()->traj.set_src(normalized_dir, normalized_file) < 0) {
				PX4_WARN("Failed to set trajectory file");
				return 0;
			}

			return 0;
		}
		else if(!strcmp(argv[i], "trajectory"))
		{
			if (argc < i + 2) {
				PX4_WARN("Usage: trajectory <start|stop|reset|execute>");
				return 0;
			}

			const char *cmd = argv[i + 1];
			sim_guidance_request_s request{};
			request.timestamp = hrt_absolute_time();

			if (!strcmp(cmd, "start")) {
				request.start = true;
				PX4_INFO("Starting trajectory guidance");
			}
			else if (!strcmp(cmd, "stop")) {
				request.stop = true;
				PX4_INFO("Stopping trajectory guidance");
			}
			else if (!strcmp(cmd, "reset")) {
				request.reset = true;
				PX4_INFO("Resetting trajectory guidance");
			}
			else if (!strcmp(cmd, "execute")) {
				request.start_execution = true;
				PX4_INFO("Executing trajectory");
			}
			else if (!strcmp(cmd, "set_home")) {
				request.set_home = true;
				PX4_INFO("Setting home position");
			}
			else {
				PX4_WARN("Unknown trajectory command: %s", cmd);
				PX4_WARN("Available commands: start, stop, reset, execute, set_home");
				return 0;
			}

			// Publish the request
			get_instance()->_sim_guidance_request_pub.publish(request);
			return 0;
		}
		else if(!strcmp(argv[i], "ls"))
		{
			if (argc < i+2)
			{
				const char* directory_ = get_instance()->traj.file_loader.get_dir();
				//PX4_WARN("Please specify a directory");
				if (get_instance()->traj.file_loader.list_dirs(directory_) < 0)
				{
					PX4_WARN("Failed to list directories");
					return 0;
				}
				if (get_instance()->traj.file_loader.list_files(directory_) < 0)
				{
					PX4_WARN("Failed to list files");
					return 0;
				}
				return 0;
			}
			else
			{
				const char *directory_ = nullptr;
				directory_ = argv[i+1];
				if (get_instance()->traj.file_loader.list_dirs(directory_) < 0)
				{
					PX4_WARN("Failed to list directories");
					return 0;
				}
				if (get_instance()->traj.file_loader.list_files(directory_) < 0)
				{
					PX4_WARN("Failed to list files");
					return 0;
				}
				return 0;
			}

		}
		else if(!strcmp(argv[i], "test"))
		{
			if (argc < i+2)
			{
				const char* directory_ = get_instance()->traj.file_loader.get_dir();
				//PX4_WARN("Please specify a directory");
				if (get_instance()->traj.file_loader.list_dirs(directory_) < 0)
				{
					PX4_WARN("Failed to list directories");
					return 0;
				}
				if (get_instance()->traj.file_loader.list_files(directory_) < 0)
				{
					PX4_WARN("Failed to list files");
					return 0;
				}
				return 0;
			}
			else if (argc - 1 > i)
			{
				if (!strcmp(argv[i+1], "solver"))
				{
					//test_solver_codegen();
					return 0;
				}
				else
				{
					PX4_WARN("Uknown test routine. \n\
						Please specify test routine from the list:\n\
						solver");
					return 0;
				}
			}
			else
			{
				const char *directory_ = nullptr;
				directory_ = argv[i+1];
				if (get_instance()->traj.file_loader.list_dirs(directory_) < 0)
				{
					PX4_WARN("Failed to list directories");
					return 0;
				}
				if (get_instance()->traj.file_loader.list_files(directory_) < 0)
				{
					PX4_WARN("Failed to list files");
					return 0;
				}
				return 0;
			}
		}
		else continue;
	}


	return print_usage("unknown command");
}


int SimulinkGuidance::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("simulink_guidance",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT - 5,
				      1800,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

SimulinkGuidance *SimulinkGuidance::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	const char *file_string = nullptr;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			PX4_INFO("p=%i",example_param);
			break;

		case 'f':
			file_string = myoptarg;

			PX4_INFO("f=%s",file_string);
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	SimulinkGuidance *instance = new SimulinkGuidance(example_param);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

SimulinkGuidance::SimulinkGuidance(int example_param)
	: ModuleParams(nullptr)
{
}

//#define DEBUG



void SimulinkGuidance::run()
{
	// initialize parameters
	parameters_update(true);

	_boot_timestamp = hrt_absolute_time();
	while (!should_exit()) {
		parameters_update(); // update parameters
		update_guidance(); //update everything related to simulink

		px4_usleep(5000);// don't update too frequenty
	}
}

template <typename Type, size_t M>
void assign_1Darray2Vector(matrix::Vector<Type, M> *output_Vec, Type input_1Darray[M])
{
	for (int i = 0; i < M; i++) output_Vec(i) = input_1Darray[i];
	return;
}

void SimulinkGuidance::update_guidance(void)
{
	int32_t enable_fl = _param_smg_en.get();


	if (enable_fl > 0)
	{
		#ifdef DEBUG
		PX4_INFO("Updating main loop");
		#endif
		traj.update(enable_fl == 2);

	}


}



void SimulinkGuidance::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int SimulinkGuidance::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Simulink Guidance module for autonomous trajectory tracking control.

Loads polynomial trajectory files and executes them by publishing setpoint commands.
Supports multi-DOF trajectories (x, y, z, yaw) with configurable polynomial orders.

### Implementation
Runs as a background task with real-time trajectory execution.
File I/O uses standard POSIX operations for robust filesystem access.
Integrates with PX4 uORB messaging for position/velocity/acceleration setpoints.

### Examples
Load and execute a trajectory:
$ simulink_guidance start
$ simulink_guidance set_src ./trajectories fig_8_1
$ simulink_guidance trajectory start
$ simulink_guidance status

Trajectory control commands:
$ simulink_guidance trajectory start     # Engage guidance module
$ simulink_guidance trajectory stop      # Stop trajectory execution
$ simulink_guidance trajectory reset     # Reset trajectory state
$ simulink_guidance trajectory execute   # Begin trajectory evaluation
$ simulink_guidance trajectory set_home  # Set home at current position

Test trajectory solver:
$ simulink_guidance test solver

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("simulink_guidance", "simulink");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("set_src");
	PRINT_MODULE_USAGE_ARG("<file_name>", "Load trajectory file from current directory (.traj extension optional)", false);
	PRINT_MODULE_USAGE_ARG("<directory> <file_name>", "Load trajectory file from specified directory", false);
	PRINT_MODULE_USAGE_COMMAND("trajectory");
	PRINT_MODULE_USAGE_ARG("start", "Engage guidance module and activate trajectory tracking", false);
	PRINT_MODULE_USAGE_ARG("stop", "Stop trajectory execution and halt guidance", false);
	PRINT_MODULE_USAGE_ARG("reset", "Reset trajectory state to initial conditions", false);
	PRINT_MODULE_USAGE_ARG("execute", "Begin trajectory evaluation and tracking", false);
	PRINT_MODULE_USAGE_ARG("set_home", "Set home position at current vehicle location", false);
	PRINT_MODULE_USAGE_COMMAND("test");
	PRINT_MODULE_USAGE_ARG("solver", "Test solver code generation", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int simulink_guidance_main(int argc, char *argv[])
{
	return SimulinkGuidance::main(argc, argv);
}
