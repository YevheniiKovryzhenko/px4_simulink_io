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

/**
 * @file file_loader_backend.cpp
 *
 * Trajectory file loader backend for simulink guidance module.
 *
 * This module provides safe file I/O operations for loading trajectory files
 * from the filesystem. It uses standard POSIX I/O functions (open/close/read/write)
 * rather than PX4's px4_open/px4_close which are only for virtual devices.
 *
 * Safety features:
 * - All input parameters are validated (null checks, length limits)
 * - Buffer overflow protection on all string operations
 * - All file operations check return values and errno
 * - Graceful error handling - no fatal crashes on file errors
 * - File descriptors properly managed (closed on error paths)
 *
 * @author Yevhenii Kovryzhenko
 */

#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <unistd.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <string.h>
#include <fcntl.h>

#include "file_loader_backend.hpp"

#define MODULE_NAME "simulink_guidance"
#define PATH_BUFFER_SIZE 256

// #define DEBUG

file_loader_backend::file_loader_backend()
{

}

file_loader_backend::~file_loader_backend()
{
}

int file_loader_backend::list_dirs(const char* location)
{
	if (!location) {
		PX4_ERR("Invalid location parameter");
		return -1;
	}

	DIR* dir = opendir(location);

	if (dir == NULL) {
		PX4_WARN("Failed to open directory %s: %s", location, strerror(errno));
		return -1;
	}

	PX4_INFO("Directories in %s:", location);
	struct dirent* entry;
	int count = 0;

	while ((entry = readdir(dir)) != NULL) {
		if (entry->d_type == DT_DIR) {
			if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0) {
				PX4_INFO("  %s/", entry->d_name);
				count++;
			}
		}
	}

	closedir(dir);
	PX4_INFO("Total directories: %d", count);

	return 0;
}

int file_loader_backend::list_files(const char* location)
{
	if (!location) {
		PX4_ERR("Invalid location parameter");
		return -1;
	}

	DIR* dir = opendir(location);

	if (dir == NULL) {
		PX4_WARN("Failed to open directory %s: %s", location, strerror(errno));
		return -1;
	}

	PX4_INFO("Files in %s:", location);
	struct dirent* entry;
	int count = 0;

	while ((entry = readdir(dir)) != NULL) {
		if (entry->d_type == DT_REG) {
			PX4_INFO("  %s", entry->d_name);
			count++;
		}
	}

	closedir(dir);
	PX4_INFO("Total files: %d", count);

	return 0;
}

/**
 * @brief Safely concatenate directory and filename paths
 *
 * @param result Output buffer for combined path
 * @param result_size Size of result buffer
 * @param directory Directory path (may or may not end with '/')
 * @param filename Filename to append
 * @return 0 on success, -1 on error
 */
static int concatenatePaths(char* result, size_t result_size, const char* directory, const char* filename) {
    if (!result || !directory || !filename) {
        PX4_ERR("NULL parameter in concatenatePaths");
        return -1;
    }

    if (result_size == 0) {
        PX4_ERR("Zero-size result buffer");
        return -1;
    }

    size_t dir_length = strlen(directory);
    size_t file_length = strlen(filename);

    // Check if combined path would fit in buffer (including null terminator and possible '/')
    if (dir_length + file_length + 2 > result_size) {
        PX4_ERR("Path too long: %zu + %zu + 2 > %zu", dir_length, file_length, result_size);
        return -1;
    }

    // Use snprintf for safety (automatically null-terminates)
    int written;
    if (dir_length > 0 && directory[dir_length - 1] != '/') {
        written = snprintf(result, result_size, "%s/%s", directory, filename);
    } else {
        written = snprintf(result, result_size, "%s%s", directory, filename);
    }

    // Verify snprintf succeeded
    if (written < 0 || (size_t)written >= result_size) {
        PX4_ERR("snprintf failed or truncated in concatenatePaths");
        return -1;
    }

    return 0;
}

int file_loader_backend::list_abs_path(const char* location)
{
	char resolved_path[256];
	resolve_abs_path(resolved_path, location);
	PX4_INFO("Absolute path: %s", resolved_path);
	return 0;
}

/**
 * @brief Resolve relative path to absolute path (mainly for debugging)
 *
 * Note: This function uses realpath() which operates on the host filesystem.
 * For PX4 SITL virtual filesystem, paths should NOT be resolved to host absolute paths.
 *
 * @param abspath Output buffer for absolute path
 * @param relpath Input relative path
 * @return 0 on success, -1 on error
 */
int file_loader_backend::resolve_abs_path(char* abspath, const char* relpath)
{
	if (!abspath || !relpath) {
		PX4_ERR("NULL parameter in resolve_abs_path");
		return -1;
	}

	if (strlen(relpath) == 0) {
		PX4_ERR("Empty path provided");
		return -1;
	}

	// Check if path is already absolute
	if (relpath[0] == '/') {
		size_t len = strlen(relpath);
		if (len >= PATH_BUFFER_SIZE) {
			PX4_ERR("Path too long: %zu >= %d", len, PATH_BUFFER_SIZE);
			return -1;
		}
		strncpy(abspath, relpath, PATH_BUFFER_SIZE - 1);
		abspath[PATH_BUFFER_SIZE - 1] = '\0';
		return 0;
	}

	char* resolved_path = realpath(relpath, NULL);
	if (resolved_path == NULL) {
		PX4_WARN("Failed to get the absolute path for %s: %s", relpath, strerror(errno));
		return -1;
	}

	size_t resolved_len = strlen(resolved_path);
	if (resolved_len >= PATH_BUFFER_SIZE) {
		PX4_ERR("Resolved path too long: %zu >= %d", resolved_len, PATH_BUFFER_SIZE);
		free(resolved_path);
		return -1;
	}

	PX4_INFO("Resolved: %s", resolved_path);
	strncpy(abspath, resolved_path, PATH_BUFFER_SIZE - 1);
	abspath[PATH_BUFFER_SIZE - 1] = '\0';
	free(resolved_path);
	return 0;
}


int file_loader_backend::set_src(const char* _file, const char* _dir)
{
	if (!_file || !_dir) {
		PX4_ERR("Invalid file or directory parameters");
		return -1;
	}

	// Validate filename length
	if (strlen(_file) >= sizeof(file_name)) {
		PX4_ERR("Filename too long: %s", _file);
		return -1;
	}

	// For PX4 SITL, we need to use paths as-is (relative to virtual filesystem root)
	// Don't resolve to host absolute paths as px4_access/px4_open use virtual filesystem
	char normalized_dir[PATH_BUFFER_SIZE];

	// If directory is relative, keep it relative
	// If it's absolute, use as-is (assuming it's already in virtual filesystem space)
	strncpy(normalized_dir, _dir, sizeof(normalized_dir) - 1);
	normalized_dir[sizeof(normalized_dir) - 1] = '\0';

	// Validate directory path length
	if (strlen(normalized_dir) >= sizeof(directory)) {
		PX4_ERR("Directory path too long: %s", normalized_dir);
		return -1;
	}

	// Check if the directory exists using opendir (works with relative paths in PX4 SITL)
	DIR* test_dir = opendir(normalized_dir);
	if (test_dir == NULL) {
		PX4_ERR("Directory does not exist or not accessible: %s", normalized_dir);
		PX4_ERR("  (errno=%d: %s)", errno, strerror(errno));
		return -1;
	}
	closedir(test_dir);
	#ifdef DEBUG
		PX4_INFO("Directory validated: %s", normalized_dir);
	#endif

	// Construct full file path for validation
	char full_path[PATH_BUFFER_SIZE];
	if (concatenatePaths(full_path, sizeof(full_path), normalized_dir, _file) < 0) {
		PX4_ERR("Failed to construct file path");
		return -1;
	}

	// Validate full path length
	if (strlen(full_path) >= PATH_BUFFER_SIZE) {
		PX4_ERR("Full file path too long (%d chars)", (int)strlen(full_path));
		return -1;
	}

	#ifdef DEBUG
		PX4_INFO("Validating file path: %s", full_path);
	#endif

	// Use standard POSIX open() for regular files (px4_open is only for virtual devices)
	int test_fd = open(full_path, O_RDONLY);
	if (test_fd < 0) {
		int err = errno;
		PX4_ERR("Cannot open file (errno=%d): %s", err, strerror(err));
		PX4_ERR("  File: %s", _file);
		PX4_ERR("  Directory: %s", normalized_dir);
		PX4_ERR("  Full path: %s", full_path);
		return -1;
	}
	#ifdef DEBUG
		PX4_INFO("  File opened successfully (fd=%d)", test_fd);
	#endif
	close(test_fd);

	// Close any currently open file
	close_file();

	// Store the filename (without path) and directory separately
	strncpy(file_name, _file, sizeof(file_name) - 1);
	strncpy(directory, normalized_dir, sizeof(directory) - 1);

	// Enforce null termination
	file_name[sizeof(file_name) - 1] = '\0';
	directory[sizeof(directory) - 1] = '\0';

	// Log the configuration
	PX4_INFO("Trajectory file set: %s", full_path);

	return 0;
}

//#define DEBUG

/**
 * @brief Normalize and validate file/directory paths
 *
 * Handles smart defaults:
 * - Auto-appends .traj extension if not present
 * - Normalizes directory paths (consistent trailing slash handling)
 * - Validates all inputs
 *
 * @param file_out Output buffer for normalized filename (min 256 bytes)
 * @param dir_out Output buffer for normalized directory (min 256 bytes)
 * @param file_in Input filename (with or without .traj extension)
 * @param dir_in Input directory path (with or without trailing /)
 * @return 0 on success, -1 on error
 */
int file_loader_backend::normalize_paths(char* file_out, char* dir_out, const char* file_in, const char* dir_in)
{
	// Validate inputs
	if (!file_out || !dir_out || !file_in || !dir_in) {
		PX4_ERR("NULL parameter in normalize_paths");
		return -1;
	}

	if (strlen(file_in) == 0) {
		PX4_ERR("Empty filename provided");
		return -1;
	}

	if (strlen(dir_in) == 0) {
		PX4_ERR("Empty directory provided");
		return -1;
	}

	// Normalize directory: copy as-is, trailing slash is optional and handled by concatenatePaths
	size_t dir_len = strlen(dir_in);
	if (dir_len >= 256) {
		PX4_ERR("Directory path too long: %zu chars", dir_len);
		return -1;
	}
	strncpy(dir_out, dir_in, 255);
	dir_out[255] = '\0';

	// Normalize filename: check for .traj extension and append if missing
	size_t file_len = strlen(file_in);
	if (file_len >= 256) {
		PX4_ERR("Filename too long: %zu chars", file_len);
		return -1;
	}

	// Check if filename already ends with .traj (case-insensitive)
	bool has_extension = false;
	if (file_len >= 5) {
		const char* ext = file_in + file_len - 5;
		if (strcasecmp(ext, ".traj") == 0) {
			has_extension = true;
		}
	}

	if (has_extension) {
		// Already has extension, use as-is
		strncpy(file_out, file_in, 255);
		file_out[255] = '\0';
	} else {
		// Append .traj extension
		if (file_len + 5 >= 256) {
			PX4_ERR("Filename too long after adding .traj extension: %zu chars", file_len + 5);
			return -1;
		}
		snprintf(file_out, 256, "%s.traj", file_in);
	}

	#ifdef DEBUG
		PX4_INFO("Normalized paths:");
		PX4_INFO("  Input:  %s / %s", dir_in, file_in);
		PX4_INFO("  Output: %s / %s", dir_out, file_out);
	#endif

	return 0;
}

int file_loader_backend::read_dummy_header(traj_file_header_t& header)
{
	header.n_coeffs = 10;
	header.n_int = 2;
	header.n_dofs = 4;

	return 0;
}

int file_loader_backend::read_dummy_data(traj_file_data_t& data, int i_int, int i_dof)
{

        static const float tof_int_raw[] = {2.107510627081439, 2.892489372918561};
	static const float x0_coefs_raw[] = {0.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};
	static const float y0_coefs_raw[] = {0.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};
	static const float z0_coefs_raw[] = {0.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};
	static const float yaw0_coefs_raw[] = {0.0, 1.223059991942257e-15,-4.932113332000828e-14,1.161170181981698e-14,-2.593478958556478e-15,10.901912753415060,-24.562419897244904,24.024854141204592,-11.641988801638679,2.277641804263970};

	static const float x1_coefs_raw[] = {1,3.667080492742769,2.117588692858374,-5.141681182811569,-3.594994733263419,16.288952666450880,-37.844351520577504,53.726887291011200,-36.411234714857710,9.191753008446979};
	static const float y1_coefs_raw[] = {1,3.667080492742769,2.117588692858374,-5.141681182811569,-3.594994733263419,16.288952666450880,-37.844351520577504,53.726887291011200,-36.411234714857710,9.191753008446979};
	static const float z1_coefs_raw[] = {1,3.667080492742769,2.117588692858374,-5.141681182811569,-3.594994733263419,16.288952666450880,-37.844351520577504,53.726887291011200,-36.411234714857710,9.191753008446979};
	static const float yaw1_coefs_raw[] = {1,3.667080492742769,2.117588692858374,-5.141681182811569,-3.594994733263419,16.288952666450880,-37.844351520577504,53.726887291011200,-36.411234714857710,9.191753008446979};

	data.i_dof = i_dof;
	data.i_int = i_int;
	data.t_int = tof_int_raw[i_int];
	switch (i_int)
	{
	case 0:
		switch (i_dof)
		{
		case 0:
			for (int i = 0; i < 10; i++) data.coefs[i] = x0_coefs_raw[i];
			break;
		case 1:
			for (int i = 0; i < 10; i++) data.coefs[i] = y0_coefs_raw[i];
			break;
		case 2:
			for (int i = 0; i < 10; i++) data.coefs[i] = z0_coefs_raw[i];
			break;

		default:
			for (int i = 0; i < 10; i++) data.coefs[i] = yaw0_coefs_raw[i];
			break;
		}
		break;

	default:
		switch (i_dof)
		{
		case 0:
			for (int i = 0; i < 10; i++) data.coefs[i] = x1_coefs_raw[i];
			break;
		case 1:
			for (int i = 0; i < 10; i++) data.coefs[i] = y1_coefs_raw[i];
			break;
		case 2:
			for (int i = 0; i < 10; i++) data.coefs[i] = z1_coefs_raw[i];
			break;

		default:
			for (int i = 0; i < 10; i++) data.coefs[i] = yaw1_coefs_raw[i];
			break;
		}
		break;
	}



	return 0;
}

int file_loader_backend::read_header(traj_file_header_t& header)
{
	if (open_file() < 0)
	{
		PX4_ERR("Failed to open file");
		return -1;
	}

	ssize_t bytes_read = read(_fd, &header, sizeof(traj_file_header_t));
	if (bytes_read < 0)
	{
		PX4_ERR("Failed to read header from fd %d: %s", _fd, strerror(errno));
		return -1;
	}
	if (bytes_read != sizeof(traj_file_header_t))
	{
		PX4_ERR("Incomplete header read: %ld of %zu bytes", (long)bytes_read, sizeof(traj_file_header_t));
		return -1;
	}

	#ifdef DEBUG
		PX4_INFO("Read header:");
		printf("n_int=%u, n_dofs=%u, n_coeffs=%u\n",header.n_int, header.n_dofs, header.n_coeffs);
	#endif

	return 0;
}

int file_loader_backend::write_header(traj_file_header_t& header)
{
	// Close if open in read mode, then open for writing
	if (_fd >= 0)
	{
		close_file();
	}

	// Construct full path from directory + filename
	char full_path[PATH_BUFFER_SIZE];
	if (concatenatePaths(full_path, sizeof(full_path), directory, file_name) < 0) {
		PX4_ERR("Failed to construct full file path for writing");
		return -1;
	}

	_fd = open(full_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (_fd < 0)
	{
		PX4_ERR("Can't open file for writing %s: %s", full_path, strerror(errno));
		return -1;
	}

	ssize_t bytes_written = write(_fd, &header, sizeof(traj_file_header_t));
	if (bytes_written < 0)
	{
		PX4_ERR("Failed to write header to fd %d: %s", _fd, strerror(errno));
		close_file();
		return -1;
	}
	if (bytes_written != sizeof(traj_file_header_t))
	{
		PX4_ERR("Incomplete header write: %ld of %zu bytes", (long)bytes_written, sizeof(traj_file_header_t));
		close_file();
		return -1;
	}
	return 0;
}



int file_loader_backend::read_data(traj_file_data_t& data)
{
	if (open_file() < 0)
	{
		PX4_ERR("Failed to open file");
		return -1;
	}

	ssize_t bytes_read = read(_fd, &data, sizeof(traj_file_data_t));
	if (bytes_read < 0)
	{
		PX4_ERR("Failed to read data from fd %d: %s", _fd, strerror(errno));
		return -1;
	}
	if (bytes_read == 0)
	{
		// End of file reached - not necessarily an error
		return -1;
	}
	if (bytes_read != sizeof(traj_file_data_t))
	{
		PX4_ERR("Incomplete data read: %ld of %zu bytes", (long)bytes_read, sizeof(traj_file_data_t));
		return -1;
	}

	#ifdef DEBUG
		PX4_INFO("Read data:");
		PX4_INFO("i_int=%u, i_dof=%u, t_int=%f",data.i_int, data.i_dof, (double)data.t_int);
		for(int i = 0; i < 10; i++) printf(", coeff[%i]=%f", i, (double)data.coefs[i]);
		printf("\n");
	#endif
	return 0;
}

int file_loader_backend::write_data(traj_file_data_t& data)
{
	// File should already be open from write_header, but check anyway
	if (_fd < 0)
	{
		// Construct full path from directory + filename
		char full_path[PATH_BUFFER_SIZE];
		if (concatenatePaths(full_path, sizeof(full_path), directory, file_name) < 0) {
			PX4_ERR("Failed to construct full file path for writing");
			return -1;
		}

		_fd = open(full_path, O_WRONLY | O_APPEND, 0644);
		if (_fd < 0)
		{
			PX4_ERR("Can't open file for writing %s: %s", full_path, strerror(errno));
			return -1;
		}
	}

	ssize_t bytes_written = write(_fd, &data, sizeof(traj_file_data_t));
	if (bytes_written < 0)
	{
		PX4_ERR("Failed to write data to fd %d: %s", _fd, strerror(errno));
		return -1;
	}
	if (bytes_written != sizeof(traj_file_data_t))
	{
		PX4_ERR("Incomplete data write: %ld of %zu bytes", (long)bytes_written, sizeof(traj_file_data_t));
		return -1;
	}
	return 0;
}

int file_loader_backend::open_file(void)
{
	if (_fd >= 0)
	{
		// File already open
		return 0;
	}

	// Construct full path from directory + filename
	char full_path[PATH_BUFFER_SIZE];
	if (concatenatePaths(full_path, sizeof(full_path), directory, file_name) < 0) {
		PX4_ERR("Failed to construct full file path");
		return -1;
	}

	// Open in read-only mode by default (trajectory files are typically read-only)
	_fd = open(full_path, O_RDONLY);
	if (_fd < 0)
	{
		PX4_ERR("Can't open file: %s", strerror(errno));
		PX4_ERR("  File: %s", file_name);
		PX4_ERR("  Directory: %s", directory);
		PX4_ERR("  Full path: %s", full_path);
		return -1;
	}
	#ifdef DEBUG
		PX4_INFO("Opened file (fd=%d): %s/%s", _fd, directory, file_name);
	#endif

	return 0;
}

const char* file_loader_backend::get_dir(void)
{
	return directory;
}
const char* file_loader_backend::get_file(void)
{
	return file_name;
}

/**
 * @brief Close currently open file (safe to call multiple times)
 *
 * @return 0 on success (or if no file was open)
 */
int file_loader_backend::close_file(void)
{
	if (_fd >= 0)
	{
		int fd_to_close = _fd;
		_fd = -1; // Mark as closed immediately to prevent double-close

		if (close(fd_to_close) < 0)
		{
			PX4_WARN("Error closing file descriptor %d: %s", fd_to_close, strerror(errno));
			// Continue anyway - file descriptor is invalidated
		}
		#ifdef DEBUG
		else
		{
			PX4_INFO("Closed file descriptor %d", fd_to_close);
		}
		#endif
	}
	return 0;
}


