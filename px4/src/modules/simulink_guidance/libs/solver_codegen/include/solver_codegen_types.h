#ifndef SOLVER_CODEGEN_TYPES_H
#define SOLVER_CODEGEN_TYPES_H

#include "rtwtypes.h"
#include "coder_posix_time.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef typedef_c_robotics_core_internal_System
#define typedef_c_robotics_core_internal_System
typedef struct {
  coderTimespec StartTime;
} c_robotics_core_internal_System;
#endif

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T {
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};
#endif
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif

#ifndef typedef_b_struct_T
#define typedef_b_struct_T
typedef struct {
  emxArray_real_T *wpts;
} b_struct_T;
#endif

#ifndef typedef_anonymous_function
#define typedef_anonymous_function
typedef struct {
  b_struct_T workspace;
} anonymous_function;
#endif

#ifndef typedef_c_TrajectoryOptimizer_solver_me
#define typedef_c_TrajectoryOptimizer_solver_me
typedef struct {
  anonymous_function wptFnc;
  double n_dim_src;
  double n_dim;
  emxArray_real_T *n_dim_ids;
  double N_wps;
  double timeWt;
  double nontriv_wpts;
  emxArray_real_T *constraints;
  emxArray_real_T *waypoints_offset;
  emxArray_real_T *waypoints;
  emxArray_real_T *timePoints;
  emxArray_real_T *minSegmentTime;
  double maxSegmentTime;
  boolean_T timeOptim;
  boolean_T print_stats_fl;
  boolean_T cost_is_good;
  emxArray_real_T *pp;
  emxArray_real_T *timeOfArrival;
  double J;
  double Iterations;
  double ExitFlag;
  double N_segments;
  double stateSize;
} c_TrajectoryOptimizer_solver_me;
#endif

#ifndef typedef_c_struct_T
#define typedef_c_struct_T
typedef struct {
  c_TrajectoryOptimizer_solver_me *this_;
} c_struct_T;
#endif

#ifndef typedef_b_anonymous_function
#define typedef_b_anonymous_function
typedef struct {
  c_struct_T workspace;
} b_anonymous_function;
#endif

#ifndef typedef_d_struct_T
#define typedef_d_struct_T
typedef struct {
  double cost;
  emxArray_real_T *grads;
} d_struct_T;
#endif

#ifndef typedef_c_TrajectoryOptimizer_DampedBFG
#define typedef_c_TrajectoryOptimizer_DampedBFG
typedef struct {
  emxArray_real_T *ConstraintMatrix;
  emxArray_real_T *ConstraintBound;
  boolean_T ConstraintsOn;
  b_anonymous_function CostFcn;
  b_anonymous_function GradientFcn;
  b_anonymous_function SolutionEvaluationFcn;
  double SolutionTolerance;
  boolean_T RandomRestart;
  b_anonymous_function RandomSeedFcn;
  d_struct_T ExtraArgs;
  double MaxNumIteration;
  double MaxTime;
  emxArray_real_T *SeedInternal;
  double MaxTimeInternal;
  double MaxNumIterationInternal;
  double StepTolerance;
  c_robotics_core_internal_System TimeObj;
  double GradientTolerance;
  double ArmijoRuleBeta;
  double ArmijoRuleSigma;
  c_robotics_core_internal_System TimeObjInternal;
} c_TrajectoryOptimizer_DampedBFG;
#endif

#ifndef struct_emxArray_int8_T
#define struct_emxArray_int8_T
struct emxArray_int8_T {
  signed char *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};
#endif
#ifndef typedef_emxArray_int8_T
#define typedef_emxArray_int8_T
typedef struct emxArray_int8_T emxArray_int8_T;
#endif

#ifndef typedef_cell_wrap_1
#define typedef_cell_wrap_1
typedef struct {
  emxArray_real_T *f1;
} cell_wrap_1;
#endif

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T
struct emxArray_boolean_T {
  boolean_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};
#endif
#ifndef typedef_emxArray_boolean_T
#define typedef_emxArray_boolean_T
typedef struct emxArray_boolean_T emxArray_boolean_T;
#endif

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T
struct emxArray_int32_T {
  int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};
#endif
#ifndef typedef_emxArray_int32_T
#define typedef_emxArray_int32_T
typedef struct emxArray_int32_T emxArray_int32_T;
#endif

#ifndef typedef_sparse
#define typedef_sparse
typedef struct {
  emxArray_real_T *d;
  emxArray_int32_T *colidx;
  emxArray_int32_T *rowidx;
  int m;
  int n;
  int maxnz;
} sparse;
#endif

#ifndef typedef_b_sparse
#define typedef_b_sparse
typedef struct {
  emxArray_real_T *d;
  emxArray_int32_T *colidx;
  emxArray_int32_T *rowidx;
} b_sparse;
#endif

#ifdef __cplusplus
}
#endif

#endif
