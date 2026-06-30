#include "solver_codegen_emxutil.h"
#include "rt_nonfinite.h"
#include "solver_codegen_types.h"
#include <stdlib.h>
#include <string.h>

void c_emxFreeStruct_TrajectoryOptim(c_TrajectoryOptimizer_solver_me *pStruct)
{
  c_emxFreeStruct_anonymous_funct(&pStruct->wptFnc);
  emxFree_real_T(&pStruct->n_dim_ids);
  emxFree_real_T(&pStruct->constraints);
  emxFree_real_T(&pStruct->waypoints_offset);
  emxFree_real_T(&pStruct->waypoints);
  emxFree_real_T(&pStruct->timePoints);
  emxFree_real_T(&pStruct->minSegmentTime);
  emxFree_real_T(&pStruct->pp);
  emxFree_real_T(&pStruct->timeOfArrival);
}

void c_emxFreeStruct_anonymous_funct(anonymous_function *pStruct)
{
  emxFreeStruct_struct_T(&pStruct->workspace);
}

void c_emxInitStruct_TrajectoryOptim(c_TrajectoryOptimizer_solver_me *pStruct)
{
  c_emxInitStruct_anonymous_funct(&pStruct->wptFnc);
  emxInit_real_T(&pStruct->n_dim_ids, 1);
  emxInit_real_T(&pStruct->constraints, 2);
  emxInit_real_T(&pStruct->waypoints_offset, 1);
  emxInit_real_T(&pStruct->waypoints, 2);
  emxInit_real_T(&pStruct->timePoints, 2);
  emxInit_real_T(&pStruct->minSegmentTime, 2);
  emxInit_real_T(&pStruct->pp, 3);
  emxInit_real_T(&pStruct->timeOfArrival, 2);
}

void c_emxInitStruct_anonymous_funct(anonymous_function *pStruct)
{
  emxInitStruct_struct_T(&pStruct->workspace);
}

void d_emxFreeStruct_TrajectoryOptim(c_TrajectoryOptimizer_DampedBFG *pStruct)
{
  emxFree_real_T(&pStruct->ConstraintMatrix);
  emxFree_real_T(&pStruct->ConstraintBound);
  emxFreeStruct_struct_T1(&pStruct->ExtraArgs);
  emxFree_real_T(&pStruct->SeedInternal);
}

void d_emxInitStruct_TrajectoryOptim(c_TrajectoryOptimizer_DampedBFG *pStruct)
{
  emxInit_real_T(&pStruct->ConstraintMatrix, 2);
  emxInit_real_T(&pStruct->ConstraintBound, 1);
  emxInitStruct_struct_T1(&pStruct->ExtraArgs);
  emxInit_real_T(&pStruct->SeedInternal, 1);
}

void emxCopyStruct_struct_T(d_struct_T *dst, const d_struct_T *src)
{
  dst->cost = src->cost;
  emxCopy_real_T(&dst->grads, &src->grads);
}

void emxCopy_real_T(emxArray_real_T **dst, emxArray_real_T *const *src)
{
  int i;
  int numElDst;
  int numElSrc;
  numElDst = 1;
  numElSrc = 1;
  for (i = 0; i < (*dst)->numDimensions; i++) {
    numElDst *= (*dst)->size[i];
    numElSrc *= (*src)->size[i];
  }
  for (i = 0; i < (*dst)->numDimensions; i++) {
    (*dst)->size[i] = (*src)->size[i];
  }
  emxEnsureCapacity_real_T(*dst, numElDst);
  for (i = 0; i < numElSrc; i++) {
    (*dst)->data[i] = (*src)->data[i];
  }
}

void emxEnsureCapacity_boolean_T(emxArray_boolean_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = calloc((unsigned int)i, sizeof(boolean_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data,
             sizeof(boolean_T) * (unsigned int)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }
    emxArray->data = (boolean_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = calloc((unsigned int)i, sizeof(int));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int) * (unsigned int)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }
    emxArray->data = (int *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxEnsureCapacity_int8_T(emxArray_int8_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = calloc((unsigned int)i, sizeof(signed char));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data,
             sizeof(signed char) * (unsigned int)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }
    emxArray->data = (signed char *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = calloc((unsigned int)i, sizeof(double));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(double) * (unsigned int)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }
    emxArray->data = (double *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void emxFreeMatrix_cell_wrap_1(cell_wrap_1 pMatrix[2])
{
  int i;
  for (i = 0; i < 2; i++) {
    emxFreeStruct_cell_wrap_1(&pMatrix[i]);
  }
}

void emxFreeStruct_cell_wrap_1(cell_wrap_1 *pStruct)
{
  emxFree_real_T(&pStruct->f1);
}

void emxFreeStruct_sparse(sparse *pStruct)
{
  emxFree_real_T(&pStruct->d);
  emxFree_int32_T(&pStruct->colidx);
  emxFree_int32_T(&pStruct->rowidx);
}

void emxFreeStruct_sparse1(b_sparse *pStruct)
{
  emxFree_real_T(&pStruct->d);
  emxFree_int32_T(&pStruct->colidx);
  emxFree_int32_T(&pStruct->rowidx);
}

void emxFreeStruct_struct_T(b_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->wpts);
}

void emxFreeStruct_struct_T1(d_struct_T *pStruct)
{
  emxFree_real_T(&pStruct->grads);
}

void emxFree_boolean_T(emxArray_boolean_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_boolean_T *)NULL) {
    if (((*pEmxArray)->data != (boolean_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }
    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_boolean_T *)NULL;
  }
}

void emxFree_int32_T(emxArray_int32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T *)NULL) {
    if (((*pEmxArray)->data != (int *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }
    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_int32_T *)NULL;
  }
}

void emxFree_int8_T(emxArray_int8_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int8_T *)NULL) {
    if (((*pEmxArray)->data != (signed char *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }
    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_int8_T *)NULL;
  }
}

void emxFree_real_T(emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (double *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }
    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

void emxInitMatrix_cell_wrap_1(cell_wrap_1 pMatrix[2])
{
  int i;
  for (i = 0; i < 2; i++) {
    emxInitStruct_cell_wrap_1(&pMatrix[i]);
  }
}

void emxInitStruct_cell_wrap_1(cell_wrap_1 *pStruct)
{
  emxInit_real_T(&pStruct->f1, 2);
}

void emxInitStruct_sparse(sparse *pStruct)
{
  emxInit_real_T(&pStruct->d, 1);
  emxInit_int32_T(&pStruct->colidx, 1);
  emxInit_int32_T(&pStruct->rowidx, 1);
}

void emxInitStruct_sparse1(b_sparse *pStruct)
{
  emxInit_real_T(&pStruct->d, 1);
  emxInit_int32_T(&pStruct->colidx, 1);
  emxInit_int32_T(&pStruct->rowidx, 1);
}

void emxInitStruct_struct_T(b_struct_T *pStruct)
{
  emxInit_real_T(&pStruct->wpts, 2);
}

void emxInitStruct_struct_T1(d_struct_T *pStruct)
{
  emxInit_real_T(&pStruct->grads, 2);
}

void emxInit_boolean_T(emxArray_boolean_T **pEmxArray)
{
  emxArray_boolean_T *emxArray;
  *pEmxArray = (emxArray_boolean_T *)malloc(sizeof(emxArray_boolean_T));
  emxArray = *pEmxArray;
  emxArray->data = (boolean_T *)NULL;
  emxArray->numDimensions = 1;
  emxArray->size = (int *)malloc(sizeof(int));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  emxArray->size[0] = 0;
}

void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions)
{
  emxArray_int32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int32_T *)malloc(sizeof(emxArray_int32_T));
  emxArray = *pEmxArray;
  emxArray->data = (int *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * (unsigned int)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void emxInit_int8_T(emxArray_int8_T **pEmxArray)
{
  emxArray_int8_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int8_T *)malloc(sizeof(emxArray_int8_T));
  emxArray = *pEmxArray;
  emxArray->data = (signed char *)NULL;
  emxArray->numDimensions = 2;
  emxArray->size = (int *)malloc(sizeof(int) * 2U);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < 2; i++) {
    emxArray->size[i] = 0;
  }
}

void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions)
{
  emxArray_real_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (double *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * (unsigned int)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}
