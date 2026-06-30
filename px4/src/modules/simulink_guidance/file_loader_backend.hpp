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
 * @file file_loader_backend.hpp
 *
 * Trajectory file loader backend for simulink guidance module.
 *
 * Provides safe file I/O operations for loading trajectory data from disk.
 * Supports both real trajectory files and dummy/test trajectories.
 *
 * Thread safety: This class is NOT thread-safe. Each instance should be
 * used from a single thread only.
 *
 * @author Yevhenii Kovryzhenko
 */

#pragma once

#include <matrix/math.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <matrix/math.hpp>
#include <stdint.h>

/**
 * @brief Trajectory file header structure
 *
 * Describes the structure of trajectory data in the file.
 * Packed to ensure consistent binary layout across platforms.
 */
typedef struct traj_file_header_t
{
	uint8_t n_coeffs;  ///< Number of polynomial coefficients per trajectory segment
	uint8_t n_int;     ///< Number of trajectory intervals/segments
	uint8_t n_dofs;    ///< Number of degrees of freedom (typically x, y, z, yaw)
}__attribute__((packed)) traj_file_header_t;

/**
 * @brief Trajectory file data structure
 *
 * Contains polynomial coefficients for one trajectory segment and DOF.
 * Packed to ensure consistent binary layout across platforms.
 */
typedef struct traj_file_data_t
{
	uint8_t i_int;     ///< Interval/segment index
	uint8_t i_dof;     ///< Degree of freedom index (0=x, 1=y, 2=z, 3=yaw)
	float t_int;       ///< Time duration of this interval
	float coefs[10];   ///< Polynomial coefficients (up to 10th order)
} __attribute__((packed)) traj_file_data_t;

/**
 * @brief File loader backend class
 *
 * Manages trajectory file I/O operations with robust error handling.
 * All methods return 0 on success, -1 on error (never throws exceptions).
 */
class file_loader_backend
{
private:
	char file_name[256] {};                         ///< Filename (without path)
	char directory[256] = "/fs/microsd/trajectories/"; ///< Directory path
	int _fd = -1;                                   ///< File descriptor (-1 when closed)

	/**
	 * @brief Open file for reading (internal use)
	 * @return 0 on success, -1 on error
	 */
	int open_file(void);

public:
	/**
	 * @brief Set source file location and validate accessibility
	 *
	 * Validates that the directory exists and the file can be opened.
	 * Closes any currently open file.
	 *
	 * @param _file Filename (without directory path)
	 * @param _dir Directory path (relative or absolute)
	 * @return 0 on success, -1 on error
	 */
	int set_src(const char* _file, const char* _dir);

	/**
	 * @brief Normalize and validate file/directory paths with smart defaults
	 *
	 * Handles:
	 * - Appends .traj extension if not present
	 * - Normalizes directory paths (adds/removes trailing slash as needed)
	 * - Validates inputs are not NULL or empty
	 *
	 * @param file_out Output buffer for normalized filename (min 256 bytes)
	 * @param dir_out Output buffer for normalized directory (min 256 bytes)
	 * @param file_in Input filename (may or may not have .traj extension)
	 * @param dir_in Input directory path (may or may not have trailing /)
	 * @return 0 on success, -1 on error
	 */
	int normalize_paths(char* file_out, char* dir_out, const char* file_in, const char* dir_in);	/**
	 * @brief Read trajectory header from file
	 * @param traj_header Output: header structure to fill
	 * @return 0 on success, -1 on error
	 */
	int read_header(traj_file_header_t& traj_header);

	/**
	 * @brief Read trajectory data block from file
	 * @param traj_data Output: data structure to fill
	 * @return 0 on success, -1 on error (including EOF)
	 */
	int read_data(traj_file_data_t& traj_data);

	/**
	 * @brief Read dummy header for testing (no file I/O)
	 * @param traj_header Output: header structure to fill
	 * @return 0 (always succeeds)
	 */
	int read_dummy_header(traj_file_header_t& traj_header);

	/**
	 * @brief Read dummy data for testing (no file I/O)
	 * @param traj_data Output: data structure to fill
	 * @param i_int Interval index
	 * @param i_dof DOF index
	 * @return 0 (always succeeds)
	 */
	int read_dummy_data(traj_file_data_t& traj_data, int i_int, int i_dof);

	/**
	 * @brief Write trajectory header to file
	 * Opens file in write mode (truncates if exists).
	 * @param traj_header Header structure to write
	 * @return 0 on success, -1 on error
	 */
	int write_header(traj_file_header_t& traj_header);

	/**
	 * @brief Write trajectory data block to file
	 * @param traj_data Data structure to write
	 * @return 0 on success, -1 on error
	 */
	int write_data(traj_file_data_t& traj_data);

	/**
	 * @brief Close currently open file (safe to call multiple times)
	 * @return 0 on success
	 */
	int close_file(void);

	/**
	 * @brief List directories in specified location
	 * @param location Directory path
	 * @return 0 on success, -1 on error
	 */
	int list_dirs(const char* location);

	/**
	 * @brief List files in specified location
	 * @param location Directory path
	 * @return 0 on success, -1 on error
	 */
	int list_files(const char* location);

	/**
	 * @brief Display absolute path of given location
	 * @param location Path to resolve
	 * @return 0 on success, -1 on error
	 */
	int list_abs_path(const char* location);

	/**
	 * @brief Resolve relative path to absolute path
	 * @param abspath Output buffer (must be at least PATH_BUFFER_SIZE)
	 * @param relpath Input relative path
	 * @return 0 on success, -1 on error
	 */
	int resolve_abs_path(char* abspath, const char* relpath);

	/**
	 * @brief Get current directory path
	 * @return Pointer to internal directory string
	 */
	const char* get_dir(void);

	/**
	 * @brief Get current filename
	 * @return Pointer to internal filename string
	 */
	const char* get_file(void);

	file_loader_backend();
	~file_loader_backend();
};
