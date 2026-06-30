#ifndef SOLVER_CODEGEN_EMXUTIL_H
#define SOLVER_CODEGEN_EMXUTIL_H

#include "rtwtypes.h"
#include "solver_codegen_types.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void
c_emxFreeStruct_TrajectoryOptim(c_TrajectoryOptimizer_solver_me *pStruct);

extern void c_emxFreeStruct_anonymous_funct(anonymous_function *pStruct);

extern void
c_emxInitStruct_TrajectoryOptim(c_TrajectoryOptimizer_solver_me *pStruct);

extern void c_emxInitStruct_anonymous_funct(anonymous_function *pStruct);

extern void
d_emxFreeStruct_TrajectoryOptim(c_TrajectoryOptimizer_DampedBFG *pStruct);

extern void
d_emxInitStruct_TrajectoryOptim(c_TrajectoryOptimizer_DampedBFG *pStruct);

extern void emxCopyStruct_struct_T(d_struct_T *dst, const d_struct_T *src);

extern void emxCopy_real_T(emxArray_real_T **dst, emxArray_real_T *const *src);

extern void emxEnsureCapacity_boolean_T(emxArray_boolean_T *emxArray,
                                        int oldNumel);

extern void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int oldNumel);

extern void emxEnsureCapacity_int8_T(emxArray_int8_T *emxArray, int oldNumel);

extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);

extern void emxFreeMatrix_cell_wrap_1(cell_wrap_1 pMatrix[2]);

extern void emxFreeStruct_cell_wrap_1(cell_wrap_1 *pStruct);

extern void emxFreeStruct_sparse(sparse *pStruct);

extern void emxFreeStruct_sparse1(b_sparse *pStruct);

extern void emxFreeStruct_struct_T(b_struct_T *pStruct);

extern void emxFreeStruct_struct_T1(d_struct_T *pStruct);

extern void emxFree_boolean_T(emxArray_boolean_T **pEmxArray);

extern void emxFree_int32_T(emxArray_int32_T **pEmxArray);

extern void emxFree_int8_T(emxArray_int8_T **pEmxArray);

extern void emxFree_real_T(emxArray_real_T **pEmxArray);

extern void emxInitMatrix_cell_wrap_1(cell_wrap_1 pMatrix[2]);

extern void emxInitStruct_cell_wrap_1(cell_wrap_1 *pStruct);

extern void emxInitStruct_sparse(sparse *pStruct);

extern void emxInitStruct_sparse1(b_sparse *pStruct);

extern void emxInitStruct_struct_T(b_struct_T *pStruct);

extern void emxInitStruct_struct_T1(d_struct_T *pStruct);

extern void emxInit_boolean_T(emxArray_boolean_T **pEmxArray);

extern void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);

extern void emxInit_int8_T(emxArray_int8_T **pEmxArray);

extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#ifdef __cplusplus
}
#endif

#endif
