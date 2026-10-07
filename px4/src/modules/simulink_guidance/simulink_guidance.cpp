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
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <math.h>
#include <uORB/topics/parameter_update.h>
#include <dirent.h>
#include <sys/stat.h>


int SimulinkGuidance::print_status()
{
	LockGuard lock{_trajectory_mutex};
	PX4_INFO("Running");
	traj.print_status();

	return 0;
}

int SimulinkGuidance::custom_command(int argc, char *argv[])
{
	if (!is_running()) { return print_usage("not running"); }
	LockGuard lock{get_instance()->_trajectory_mutex};
	auto &instance = *get_instance();
	if (argc == 0) { return print_usage("missing command"); }
	if (!strcmp(argv[0], "set_src")) {
		if (argc != 2 && argc != 3) { return print_usage("set_src <file> or <directory> <file>"); }
		const char *directory = argc == 3 ? argv[1] : instance.traj.file_loader.get_dir();
		const char *filename = argv[argc - 1];
		char normalized_file[256], normalized_directory[256];
		if (instance.traj.file_loader.normalize_paths(normalized_file, normalized_directory, filename, directory) < 0) {
			return PX4_ERROR;
		}
		return instance.traj.set_src(normalized_directory, normalized_file);
	}
	if (!strcmp(argv[0], "trajectory")) {
		if (argc != 2) { return print_usage("trajectory <start|stop|reset|execute|set_home>"); }
		if (instance._param_smg_en.get() == 0) {
			PX4_WARN("Enable SMG_EN before issuing trajectory commands");
			return PX4_ERROR;
		}
		sim_guidance_request_s request{};
		request.timestamp = hrt_absolute_time();
		if (!strcmp(argv[1], "start")) { request.start = true; }
		else if (!strcmp(argv[1], "stop")) { request.stop = true; }
		else if (!strcmp(argv[1], "reset")) { request.reset = true; }
		else if (!strcmp(argv[1], "execute")) { request.start_execution = true; }
		else if (!strcmp(argv[1], "set_home")) { request.set_home = true; }
		else { return print_usage("unknown trajectory command"); }
		return instance._sim_guidance_request_pub.publish(request) ? PX4_OK : PX4_ERROR;
	}
	if (!strcmp(argv[0], "ls")) {
		if (argc > 2) { return print_usage("ls [directory]"); }
		const char *directory = argc == 2 ? argv[1] : instance.traj.file_loader.get_dir();
		if (instance.traj.file_loader.list_dirs(directory) < 0) { return PX4_ERROR; }
		return instance.traj.file_loader.list_files(directory);
	}
	if (!strcmp(argv[0], "test")) {
		PX4_WARN("The onboard solver is not built; run the host guidance regression tests");
		return PX4_ERROR;
	}
	return print_usage("unknown command");
}

void SimulinkGuidance::load_trajectory_from_params()
{
	int32_t traj_dir_select = _params_smg_traj_dir.get();
	int32_t traj_id         = _params_smg_traj_id.get();

	if (traj_dir_select == _last_traj_dir && traj_id == _last_traj_id) {
		return;
	}

	// Resolve directory string based on the enum parameter selection
	const char *dir_arg;
	switch (traj_dir_select)
	{
	case 1:
		dir_arg = "/fs/microsd/Trajectories/";
	break;

	default:
		dir_arg = "./Trajectories/";
	break;
	}

	// Open directory and locate the unique target matching prefix string (e.g., "ID0001_")
	DIR *dir_handle = opendir(dir_arg);
	if (dir_handle == nullptr) {
		PX4_ERR("Auto-load failed: cannot open directory %s", dir_arg);
		return;
	}

	char prefix_token[32];
	snprintf(prefix_token, sizeof(prefix_token), "ID%04d_", (int)traj_id);

	struct dirent *entry;
	char discovered_filename[256] = "";

	bool duplicate = false;
	while ((entry = readdir(dir_handle)) != nullptr) {
		const size_t length = strlen(entry->d_name);
		if (strncmp(entry->d_name, prefix_token, strlen(prefix_token)) != 0
		    || length < 5 || strcasecmp(entry->d_name + length - 5, ".traj") != 0) {
			continue;
		}
		char path[512];
		const int written = snprintf(path, sizeof(path), "%s%s", dir_arg, entry->d_name);
		struct stat info{};
		if (written < 0 || static_cast<size_t>(written) >= sizeof(path)
		    || stat(path, &info) != 0 || !S_ISREG(info.st_mode)) {
			continue;
		}
		if (discovered_filename[0] != '\0') {
			duplicate = true;
			break;
		}
		strncpy(discovered_filename, entry->d_name, sizeof(discovered_filename) - 1);
	}
	closedir(dir_handle);

	if (duplicate) {
		PX4_ERR("Multiple trajectory files match %s", prefix_token);
		return;
	}

	// Fallback safely if no matching prefix asset was found inside the folder
	if (strlen(discovered_filename) == 0) {
		PX4_WARN("Auto-load aborted: no file matching prefix '%s' in %s", prefix_token, dir_arg);
		return;
	}

	// Pass directly to your backend matrix file loader
	if (get_instance()->traj.set_src(dir_arg, discovered_filename) < 0) {
		PX4_ERR("Auto-load: Backend rejected source files configuration");
	} else {
		// Update the tracking cache ONLY after a successful registration setup
		_last_traj_dir = traj_dir_select;
		_last_traj_id = traj_id;

		PX4_INFO("Successfully loaded trajectory asset: %s", discovered_filename);
	}
}




int SimulinkGuidance::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("simulink_guidance",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT - 5,
				      4096,
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
	if (argc > 1) {
		print_usage("Use set_src after starting the module");
		return nullptr;
	}
	return new SimulinkGuidance();
}

SimulinkGuidance::SimulinkGuidance() : ModuleParams(nullptr)
{
	_sim_guidance_request_pub.advertise();
}

//#define DEBUG



void SimulinkGuidance::run()
{
	{
		LockGuard lock{_trajectory_mutex};
		parameters_update(true);
	}
	while (!should_exit()) {
		{
			LockGuard lock{_trajectory_mutex};
			parameters_update();
			update_guidance();
		}
		px4_usleep(5000);
	}
	LockGuard lock{_trajectory_mutex};
	traj.disable();
}

void SimulinkGuidance::update_guidance()
{
	const int32_t enable = _param_smg_en.get();
	if (enable != _last_enable) {
		traj.disable();
		_last_enable = enable;
	}
	if (enable > 0) {
		traj.update(enable == 2);
	}
}


void SimulinkGuidance::parameters_update(bool force)
{
	// Check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// Clear update notification from the uORB bus
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// Synchronize the macro values from the central storage backend
		updateParams();
		traj.configure(_param_smg_in_type.get(), _param_smg_out_type.get());

		// Check if trajectory needs to be loaded
		load_trajectory_from_params();
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
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int simulink_guidance_main(int argc, char *argv[])
{
	return SimulinkGuidance::main(argc, argv);
}
