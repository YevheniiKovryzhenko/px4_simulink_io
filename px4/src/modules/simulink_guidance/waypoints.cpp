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


#include <px4_platform_common/module.h>
#include "waypoints.hpp"

#define MAX_WPTS 200
#define MAX_DIM 4
#define N_COEFFS 10

const char* get_trajectory_type_string(trajectory_type_t traj_type)
{

  switch (traj_type)
  {
  case basic_1:
    return "basic_1";
  case line_x_1:
    return "line_x_1";
  case line_xy_1:
    return "line_xy_1";
  case line_xyz_1:
    return "line_xyz_1";
  case rrt_test_1:
    return "rrt_test_1";
  case circle_1:
   return "circle_1";
  case circle_1_slow:
    return "circle_1_slow";
  case square_1:
    return "square_1";
  case square_1_slow:
    return "square_1_slow";
  case fig_8_1:
    return "fig_8_1";
  case fig_8_1_slow:
   return "fig_8_1_slow";
  case fig_8_yaw_1:
   return "fig_8_yaw_1";
  case fig_8_yaw_1_slow:
    return "fig_8_yaw_1_slow";
  default:
    return "N/A";
  }
}

void reshape2D_into_1D(double* in_2D, double* out_1D, uint16_t size[2])
{
  int id = 0;
  for (uint16_t i = 0; i < size[0]; i++)
  {
    double* tmp = &in_2D[i];
    for (uint16_t ii = 0; ii < size[1]; ii++)
    {
        out_1D[id] = tmp[ii];
        id++;
    }
  }
}

void reshape1D_into_2D(double* in_1D, double* out_2D, int size[2])
{
  int id = 0;
  for (int i = 0; i < size[0]; i++)
  {
    double* tmp = &out_2D[i];
    for (int ii = 0; ii < size[1]; ii++)
    {
        tmp[ii] = in_1D[id];
        id++;
    }
  }
}

void printf_emxArray_real_T(emxArray_real_T* in)
{
  /*
    struct emxArray_real_T {
    double *data;
    int *size;
    int allocatedSize;
    int numDimensions;
    boolean_T canFreeData;
    };
  */
  for(int i = 0; i < in->allocatedSize; i++)
  {
    printf("data[%i] = %f\n", i, in->data[i]);
  }
  for(int i = 0; i < in->allocatedSize; i++)
  {
    printf("size[%i] = %i\n", i, in->size[i]);
  }

  printf("allocatedSize: %i\n", in->allocatedSize);
  printf("numDimensions: %i\n", in->numDimensions);
  printf("canFreeData: %i\n", in->canFreeData);
}

bool preallocate_array(double** array, size_t size)
{
  if (*array != NULL) free(*array);
  *array = (double*)malloc(size);

  // Checking for memory allocation
  if (*array == NULL)
  {
      printf("Memory not allocated.\n");
      return false;
  }

  return true; //success!
}

bool reset_wpts(double** wpts_1D, double* wpts_2D, uint16_t wpts_size[2], double** T_out)
{
  if (preallocate_array(wpts_1D, sizeof(double) * (size_t)(wpts_size[0]*wpts_size[1]))\
    && preallocate_array(T_out, sizeof(double) * (size_t)(wpts_size[1]+1)))
  {
    reshape2D_into_1D(wpts_2D, *wpts_1D, wpts_size);
    return true;
  }
  else
  {
    printf("ERROR: failed to preallocate array\n");
    return false;
  }
}

bool traj_basic_1(double** wpts, uint16_t wpts_size[2], double* Tf, double** T_out)
{

  static const uint16_t n_dim = 1;
  static const uint16_t n_wpts = 2;

  double wpts_[n_dim][n_wpts];

  wpts_[0][0] = 0.0;
  wpts_[0][1] = 1.0;

  wpts_size[0] = n_dim;
  wpts_size[1] = n_wpts;

  *Tf = 5;
  return reset_wpts(wpts, &wpts_[0][0], wpts_size, T_out);
}

bool traj_line_x_1(double** wpts, uint16_t wpts_size[2], double* Tf, double** T_out)
{

  static const uint16_t n_dim = 1;
  static const uint16_t n_wpts = 3;

  double wpts_[n_dim][n_wpts];

  wpts_[0][0] = 0.0;
  wpts_[0][1] = 1.0;
  wpts_[0][2] = 3.0;

  wpts_size[0] = n_dim;
  wpts_size[1] = n_wpts;

  *Tf = 5;
  return reset_wpts(wpts, &wpts_[0][0], wpts_size, T_out);
}


bool traj_line_xy_1(double** wpts, uint16_t wpts_size[2], double* Tf, double** T_out)
{

  static const uint16_t n_dim = 2;
  static const uint16_t n_wpts = 3;

  double wpts_[2][3] = {
    {0,   1,   3},
    {0,   1,   3}
    };

  wpts_size[0] = n_dim;
  wpts_size[1] = n_wpts;

  *Tf = 5;

  return reset_wpts(wpts, &wpts_[0][0], wpts_size, T_out);
}

bool traj_line_xyz_1(double** wpts, uint16_t wpts_size[2], double* Tf, double** T_out)
{
  static const uint16_t n_dim = 3;
  static const uint16_t n_wpts = 3;

  double wpts_[3][3] = {
    {0,   1,   3},
    {0,   1,   3},
    {0,   1,   3}
    };

  wpts_size[0] = n_dim;
  wpts_size[1] = n_wpts;

  *Tf = 5;

  return reset_wpts(wpts, &wpts_[0][0], wpts_size, T_out);
}

bool traj_rrt_test_1(double** wpts, uint16_t wpts_size[2], double* Tf, double** T_out)
{
  static const uint16_t n_dim = 4;
  static const uint16_t n_wpts = 5;

  double wpts_[4][5] = {
    {5.4,   4.2,  2.8,  0.5,  0.4},
    {3,     1.6,  1.6,  1.8,  0.8},
    {-0.4,  -1.4, -1.4, -0.9, -0.4},
    {M_PI,  M_PI, M_PI, M_PI, M_PI}
    };

  wpts_size[0] = n_dim;
  wpts_size[1] = n_wpts;

  *Tf = 10;

  return reset_wpts(wpts, &wpts_[0][0], wpts_size, T_out);
}

bool traj_circle_1(double** wpts, uint16_t wpts_size[2], double* Tf, double** T_out)
{
  static const uint16_t n_dim = 3;
  static const uint16_t n_wpts = 8;
  *Tf = 10;

  double wpts_[3][8];

  for (uint16_t i_wpt = 1; i_wpt < n_wpts; i_wpt++)
  {
    wpts_[0][i_wpt] = 2*sin(2*M_PI * (*Tf) / (n_wpts - 1));
    wpts_[1][i_wpt] = 2*cos(2*M_PI * (*Tf) / (n_wpts - 1));
    wpts_[2][i_wpt] = -1;
  }

  wpts_size[0] = n_dim;
  wpts_size[1] = n_wpts;

  return reset_wpts(wpts, &wpts_[0][0], wpts_size, T_out);
}

bool traj_circle_1_slow(double** wpts, uint16_t wpts_size[2], double* Tf, double** T_out)
{
  bool res = traj_circle_1(wpts, wpts_size, Tf, T_out);
  *Tf = 20;
  return res;
}



bool traj_square_1(double** wpts, uint16_t wpts_size[2], double* Tf, double** T_out)
{
  static const uint16_t n_dim = 3;
  static const uint16_t n_wpts = 9;

  double wpts_[3][9] = {
    {1,   0,     -1,     -1,     -1,     0,      1,   1,   1},
    {1,   1,      1,     0,      -1,     -1,     -1,  0,   1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1}
    };

  wpts_size[0] = n_dim;
  wpts_size[1] = n_wpts;
  *Tf = 10;

  return reset_wpts(wpts, &wpts_[0][0], wpts_size, T_out);
}

bool traj_8_1(double** wpts, uint16_t wpts_size[2], double* Tf, double** T_out)
{
  static const uint16_t n_dim = 3;
  static const uint16_t n_wpts = 9;

  double wpts_[4][9] = {
    {0,       0.5,    0.0,    -0.5,   0.0,    0.5,    0.0,    -0.5,   0.0},
    {0,       1.0,    2.0,    1.0,    0.0,    -1.0,   -2.0,   -1.0,   0.0},
    {0,       0.25,   0.5,    0.25,   0.0,    -0.25,  -0.5,   -0.25,  0.0}
  };

  wpts_size[0] = n_dim;
  wpts_size[1] = n_wpts;
  *Tf = 15;

  return reset_wpts(wpts, &wpts_[0][0], wpts_size, T_out);
}

bool traj_8_1_slow(double** wpts, uint16_t wpts_size[2], double* Tf, double** T_out)
{
  bool res = traj_8_1(wpts, wpts_size, Tf, T_out);
  *Tf = 20;
  return res;
}

bool traj_8_yaw_1(double** wpts, uint16_t wpts_size[2], double* Tf, double** T_out)
{
  static const uint16_t n_dim = 4;
  static const uint16_t n_wpts = 9;

  double wpts_[4][9] = {
    {0,       0.5,    0.0,    -0.5,   0.0,    0.5,    0.0,    -0.5,   0.0},
    {0,       1.0,    2.0,    1.0,    0.0,    -1.0,   -2.0,   -1.0,   0.0},
    {0,       0.25,   0.5,    0.25,   0.0,    -0.25,  -0.5,   -0.25,  0.0},
    {45,      90.0,   180.0,  270.0,  315.0,  270.0,  180.0,  90.0,   45.0}
  };

  for (uint16_t i_wpt = 0; i_wpt < n_wpts; i_wpt++)
  {
    wpts_[3][i_wpt] = (wpts_[3][i_wpt] - 45.0) * M_PI / 180.0;
  }

  wpts_size[0] = n_dim;
  wpts_size[1] = n_wpts;
  *Tf = 15;

  return reset_wpts(wpts, &wpts_[0][0], wpts_size, T_out);
}

bool traj_8_yaw_1_slow(double** wpts, uint16_t wpts_size[2], double* Tf, double** T_out)
{
  bool res = traj_square_1(wpts, wpts_size, Tf, T_out);
  *Tf = 20;
  return res;
}

bool traj_square_1_slow(double** wpts, uint16_t wpts_size[2], double* Tf, double** T_out)
{
  bool res = traj_8_yaw_1(wpts, wpts_size, Tf, T_out);
  *Tf = 20;
  return res;
}

bool solve_trajectory_min_snap(double* wpts_data, uint16_t size[2], double Tf, bool use_time_allocation, bool show_details,\
        double** pp_out, double* T_out, double* offset_out, uint16_t* N_dim_out, uint16_t* N_segments_out)
{
  bool all_good = false;
  if (Tf > 0.0)
  {
    if (size[0] > MAX_DIM || size[0] < 1 || size[1] > MAX_WPTS || size[1] < 1)
    {
      PX4_WARN("ERROR: data size must be between [1][1] and [%i][%i]\n", MAX_DIM, MAX_WPTS);
    }
    else
    {
        emxArray_real_T *T = NULL;
        emxArray_real_T *n_dim_ids = NULL;
        emxArray_real_T *pp = NULL;
        //double wpts_data[800];
        emxArray_real_T *offset = NULL;
        double Cost;
        double ExitFlag;
        double Iterations;
        double N_segments;
        double n_dim;


        int wpts_size[2] = {(int)size[0], (int)size[1]};
        //reshape2D_into_1D(wpts_data_org_2D, wpts_data, wpts_size);


        emxInitArray_real_T(&T, 2);
        emxInitArray_real_T(&pp, 3);
        emxInitArray_real_T(&n_dim_ids, 1);
        emxInitArray_real_T(&offset, 1);


        //printf("Initilized, starting execution!\n");
        //uint64_t time_stamp = get_time_usec();

        solver_codegen(wpts_data, wpts_size, Tf,
                      use_time_allocation, show_details, T, pp,
                      &n_dim, n_dim_ids, &N_segments, offset, &Cost, &ExitFlag,
                      &Iterations);

        //printf("Time elapsed: %f s\n", get_dt_s(time_stamp));

        all_good = (int)ExitFlag == 0 || (int)ExitFlag == 2;

        //printf("Results out:\n");
        *N_segments_out = (uint16_t)N_segments;
        *N_dim_out = (uint16_t)n_dim;

        //printf("Cost = %f, n_dim = %i, N_segments = %i\n", Cost, (int)*N_dim_out, (int)*N_segments_out);
        //printf("offset = [ ");
        for (int i = 0; i < MAX_DIM; i++)
        {
          offset_out[i] = offset->data[i];
          //printf("%f ", offset_out[i]);
        }
        //printf("]\n");
        //printf("T = [ ");
        for (int i = 0; i < N_segments+1; i++)
        {
          T_out[i] = T->data[i];
          //printf("%f ", T_out[i]);
        }
        //printf("]\n");
        //printf("n_dim_ids = [ ");
        //for (int i = 0; i < n_dim; i++) printf("%i ", (int) n_dim_ids->data[i]);
        //printf("]\n");

        if (preallocate_array(pp_out, sizeof(double) * (size_t)(N_segments*N_COEFFS*n_dim)))
        {
          double* pp_out_tmp = *pp_out;
          for (int i_seg = 0; i_seg < N_segments; i_seg++)
          {
            for (int i_dim = 0; i_dim < n_dim; i_dim++)
            //for (int i_coefs = 0; i_coefs < N_COEFFS; i_coefs++)
            {
              //printf("pp[seg=%i,dim=%i] = [ ", i_seg, i_dim);
              for (int i_coefs = 0; i_coefs < 10; i_coefs++)
              //for (int i_dim = 0; i_dim < n_dim; i_dim++)
              {

                pp_out_tmp[i_seg * (int)(N_segments * n_dim) + i_coefs * (int)n_dim + i_dim] = pp->data[(i_dim + pp->size[2] * i_coefs) +\
                                                pp->size[2] * N_COEFFS * i_seg];
                //test_pt[i * y_size * z_size + ii * z_size + iii] = test[i][ii][iii];
                //printf("%f ", pp_out[i_seg][i_coefs][i_dim]);
                //pp_out[i_seg * (int)(N_segments * n_dim) + i_coefs * (int)n_dim + i_dim] = 0.0;
              }
              //printf("]\n");
            }
          }
        }
        else
        {
            PX4_WARN("ERROR: can't transfer results\n");
            all_good = false;
        }




        emxDestroyArray_real_T(pp);
        emxDestroyArray_real_T(n_dim_ids);
        emxDestroyArray_real_T(offset);
        emxDestroyArray_real_T(T);

        //printf("Done!\n");
    }
  }
  else
  {
    PX4_WARN("ERROR: Total time must me positive\n");
  }

  return all_good;
}

bool solve_preset_traj(trajectory_type_t traj_type,\
        double** wpts, uint16_t wpts_size[2], double* Tf, bool use_time_allocation, bool show_details,\
        double** pp, double** T_out, double* offsets, uint16_t* N_dim, uint16_t* N_segments)
{
  int64_t time_stamp = hrt_absolute_time();
  switch (traj_type)
  {
    case basic_1:
      if (traj_basic_1(wpts, wpts_size, Tf, T_out)) break;
      else return false;

    case line_x_1:
      if (traj_line_x_1(wpts, wpts_size, Tf, T_out)) break;
      else return false;

    case line_xy_1:
      if (traj_line_xy_1(wpts, wpts_size, Tf, T_out)) break;
      else return false;

    case line_xyz_1:
      if (traj_line_xyz_1(wpts, wpts_size, Tf, T_out)) break;
      else return false;

    case rrt_test_1:
      if (traj_rrt_test_1(wpts, wpts_size, Tf, T_out)) break;
      else return false;

    case circle_1:
      if (traj_circle_1(wpts, wpts_size, Tf, T_out)) break;
      else return false;

    case circle_1_slow:
      if (traj_circle_1_slow(wpts, wpts_size, Tf, T_out)) break;
      else return false;

    case square_1:
      if (traj_square_1(wpts, wpts_size, Tf, T_out)) break;
      else return false;

    case square_1_slow:
      if (traj_square_1_slow(wpts, wpts_size, Tf, T_out)) break;
      else return false;

    case fig_8_1:
      if (traj_8_1(wpts, wpts_size, Tf, T_out)) break;
      else return false;

    case fig_8_1_slow:
      if (traj_8_1_slow(wpts, wpts_size, Tf, T_out)) break;
      else return false;

    case fig_8_yaw_1:
      if (traj_8_yaw_1(wpts, wpts_size, Tf, T_out)) break;
      else return false;

    case fig_8_yaw_1_slow:
      if (traj_8_yaw_1_slow(wpts, wpts_size, Tf, T_out)) break;
      else return false;

    default:
      printf("ERROR: unknown trajectory\n");
      return false;
  }

  PX4_INFO("Solving %s trajectory...", get_trajectory_type_string(traj_type));

  px4_usleep(10000);

  solve_trajectory_min_snap(*wpts, wpts_size, *Tf, use_time_allocation, show_details,\
        pp, *T_out, offsets, N_dim, N_segments);

  PX4_INFO("Trajectory: %s\nTime elapsed: %f s\n", get_trajectory_type_string(traj_type), static_cast<double>(hrt_absolute_time() - time_stamp) / 1000000.0);
  return true;
}

void test_solver_codegen(void)
{
  bool use_time_allocation = false;
  bool show_details = false;

  double* wpts = NULL;
  uint16_t wpts_size[2];
  double Tf;

  double* pp = NULL;
  double* T_out = NULL;
  double offsets[MAX_DIM];
  uint16_t N_dim;
  uint16_t N_segments;

  solve_preset_traj(basic_1, &wpts, wpts_size, &Tf, use_time_allocation, show_details,\
          &pp, &T_out, offsets, &N_dim, &N_segments);

  solve_preset_traj(line_x_1, &wpts, wpts_size, &Tf, use_time_allocation, show_details,\
          &pp, &T_out, offsets, &N_dim, &N_segments);

  solve_preset_traj(line_xy_1, &wpts, wpts_size, &Tf, use_time_allocation, show_details,\
          &pp, &T_out, offsets, &N_dim, &N_segments);

  solve_preset_traj(line_xyz_1, &wpts, wpts_size, &Tf, use_time_allocation, show_details,\
          &pp, &T_out, offsets, &N_dim, &N_segments);

  solve_preset_traj(rrt_test_1, &wpts, wpts_size, &Tf, use_time_allocation, show_details,\
          &pp, &T_out, offsets, &N_dim, &N_segments);

  solve_preset_traj(circle_1, &wpts, wpts_size, &Tf, use_time_allocation, show_details,\
          &pp, &T_out, offsets, &N_dim, &N_segments);

  solve_preset_traj(square_1, &wpts, wpts_size, &Tf, use_time_allocation, show_details,\
          &pp, &T_out, offsets, &N_dim, &N_segments);

  solve_preset_traj(fig_8_1, &wpts, wpts_size, &Tf, use_time_allocation, show_details,\
          &pp, &T_out, offsets, &N_dim, &N_segments);

  solve_preset_traj(fig_8_yaw_1, &wpts, wpts_size, &Tf, use_time_allocation, show_details,\
          &pp, &T_out, offsets, &N_dim, &N_segments);

  PX4_INFO("Done!");

  if (wpts != NULL) free(wpts);
  if (T_out != NULL) free(T_out);
  if (pp != NULL) free(pp);
}
