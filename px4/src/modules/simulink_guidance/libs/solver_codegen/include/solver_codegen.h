#ifndef SOLVER_CODEGEN_H
#define SOLVER_CODEGEN_H

#include "rtwtypes.h"
#include "solver_codegen_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void solver_codegen(const double wpts_data[], const int wpts_size[2],
                           double Tf, boolean_T opt_time_allocation_fl,
                           boolean_T ShowDetails, emxArray_real_T *T,
                           emxArray_real_T *pp, double *n_dim,
                           emxArray_real_T *n_dim_ids, double *N_segments,
                           emxArray_real_T *offset, double *Cost,
                           double *ExitFlag, double *Iterations);

extern void solver_codegen_initialize(void);

extern void solver_codegen_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
