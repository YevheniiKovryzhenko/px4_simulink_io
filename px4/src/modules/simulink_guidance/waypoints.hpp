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

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "rt_nonfinite.h"
#include "solver_codegen.h"
#include "solver_codegen_emxAPI.h"
#include "solver_codegen_types.h"

typedef enum
{
        basic_1,
        line_x_1,
        line_xy_1,
        line_xyz_1,
        rrt_test_1,
        circle_1,
        circle_1_slow,
        square_1,
        square_1_slow,
        fig_8_1,
        fig_8_1_slow,
        fig_8_yaw_1,
        fig_8_yaw_1_slow
} trajectory_type_t;

bool solve_preset_traj(trajectory_type_t traj_type,\
        double** wpts, uint16_t wpts_size[2], double* Tf, bool use_time_allocation, bool show_details,\
        double** pp, double** T_out, double* offsets, uint16_t* N_dim, uint16_t* N_segments);

bool solve_trajectory_min_snap(double* wpts_data, uint16_t size[2], double Tf, bool use_time_allocation, bool show_details,\
        double** pp_out, double* T_out, double* offset_out, uint16_t* N_dim_out, uint16_t* N_segments_out);

void test_solver_codegen(void);
