#include <px4_platform_common/posix.h>

#include "solver_codegen.h"
#include "rt_nonfinite.h"
#include "solver_codegen_emxutil.h"
#include "solver_codegen_types.h"
#include "coder_posix_time.h"
#include "cs.h"
#include "makeCXSparseMatrix.h"
#include "rt_nonfinite.h"
#include "solve_from_lu.h"
#include "solve_from_qr.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define DEBUG

#define PX4_SOLVER_WAIT_TIME_US 1000
#ifdef DEBUG

#define DELAY_FUNC1(string, delay_us) \
  PX4_INFO("%s",string);\
  px4_usleep(delay_us);\
  int64_t time_stamp = hrt_absolute_time();

#define DELAY_FUNC(string, delay_us) \
  PX4_INFO("%s time: %f (s)",string,(double)(hrt_absolute_time() - time_stamp) / 1000000.0);\
  px4_usleep(delay_us);\
  time_stamp = hrt_absolute_time();

#define DELAY_FUNC_END(string, delay_us) \
  PX4_INFO("%s time: %f (s)",string,(double)(hrt_absolute_time() - time_stamp) / 1000000.0);\
  px4_usleep(delay_us);

#else

#define DELAY_FUNC1(string, delay_us) \
  px4_usleep(delay_us);

#define DELAY_FUNC(string, delay_us) \
  px4_usleep(delay_us);

#define DELAY_FUNC_END(string, delay_us) \
  px4_usleep(delay_us);

#endif



#ifndef typedef_struct_T
#define typedef_struct_T

typedef struct {
  int xstart;
  int xend;
  int depth;
} struct_T;

#endif

static double freq;
static boolean_T freq_not_empty;
static const signed char iv[45] = { 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5,
  5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 10,
  10, 10, 10, 10 };

static const signed char iv1[45] = { 1, 6, 2, 6, 7, 3, 6, 7, 8, 4, 6, 7, 8, 9, 5,
  6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10, 6, 7, 8, 9, 10,
  6, 7, 8, 9, 10 };

static const signed char iv2[100] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
  5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7,
  7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10,
  10, 10, 10, 10, 10, 10, 10, 10 };

static const signed char iv3[100] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4,
  5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8, 9,
  10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4,
  5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8, 9,
  10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

static boolean_T isInitialized_solver_codegen = false;
static void CXSparseAPI_iteratedQR(const emxArray_real_T *A_d, const
  emxArray_int32_T *A_colidx, const emxArray_int32_T *A_rowidx, int A_m, int A_n,
  const emxArray_real_T *b, int n, emxArray_real_T *out);
static boolean_T any(const emxArray_real_T *x);
static boolean_T b_any(const emxArray_boolean_T *x);
static void b_binary_expand_op(emxArray_real_T *in1, double in2, const
  emxArray_real_T *in3);
static void b_eml_find(const emxArray_int32_T *x_colidx, const emxArray_int32_T *
  x_rowidx, emxArray_int32_T *i, emxArray_int32_T *j);
static void b_heapify(int x[100], int idx, int xstart, int xend);
static void b_heapsort(int x[45], int xstart, int xend);
static void b_insertionsort(int x[100], int xstart, int xend);
static void b_introsort(int x[100]);
static void b_minus(emxArray_real_T *in1, const emxArray_real_T *in2, const
                    emxArray_real_T *in3);
static void b_mtimes(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *C);
static double b_norm(const emxArray_real_T *x);
static void b_plus(emxArray_real_T *in1, const emxArray_real_T *in2, const
                   emxArray_real_T *in3);
static double b_solver_common_solvePoly(c_TrajectoryOptimizer_solver_me *this,
  const emxArray_real_T *T, emxArray_real_T *p);
static void b_sparse_fillIn(b_sparse *this);
static void b_sparse_mtimes(const emxArray_real_T *a_d, const emxArray_int32_T
  *a_colidx, const emxArray_int32_T *a_rowidx, int a_m, int a_n, const
  emxArray_real_T *b, emxArray_real_T *c);
static int b_sparse_parenReference(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx, const emxArray_int32_T *this_rowidx, const
  emxArray_real_T *varargin_1, const emxArray_real_T *varargin_2,
  emxArray_real_T *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx,
  int *s_n, int *s_maxnz);
static void binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
  double in3, const emxArray_real_T *in4);
static double c_DampedBFGSwGradientProjection(c_TrajectoryOptimizer_DampedBFG
  *obj, emxArray_real_T *xSol, double *err, double *iter);
static void c_binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
  const emxArray_real_T *in3, double in4);
static void c_heapify(emxArray_int32_T *x, int idx, int xstart, int xend, const
                      emxArray_int32_T *cmp_workspace_a, const emxArray_int32_T *
                      cmp_workspace_b);
static void c_heapsort(int x[100], int xstart, int xend);
static void c_insertionsort(emxArray_int32_T *x, int xstart, int xend, const
  emxArray_int32_T *cmp_workspace_a, const emxArray_int32_T *cmp_workspace_b);
static void c_introsort(emxArray_int32_T *x, int xend, const emxArray_int32_T
  *cmp_workspace_a, const emxArray_int32_T *cmp_workspace_b);
static void c_minus(emxArray_real_T *in1, const emxArray_real_T *in2, const
                    emxArray_real_T *in3);
static void c_mtimes(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *C);
static void d_binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
  const emxArray_real_T *in3, const emxArray_real_T *in4, double in5);
static void d_heapify(emxArray_int32_T *x, int idx, int xstart, int xend);
static void d_heapsort(emxArray_int32_T *x, int xstart, int xend, const
  emxArray_int32_T *cmp_workspace_a, const emxArray_int32_T *cmp_workspace_b);
static void d_insertionsort(emxArray_int32_T *x, int xstart, int xend);
static void d_introsort(emxArray_int32_T *x, int xstart, int xend);
static void d_mtimes(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *C);
static void diag(const emxArray_real_T *v, emxArray_real_T *d);
static void diff(const emxArray_real_T *x, emxArray_real_T *y);
static void e_binary_expand_op(emxArray_real_T *in1, double in2, const
  emxArray_real_T *in3, const emxArray_real_T *in4);
static void e_heapsort(emxArray_int32_T *x, int xstart, int xend);
static void e_mtimes(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *C);
static void eml_find(const emxArray_boolean_T *x, emxArray_int32_T *i);
static void f_binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
  const emxArray_real_T *in3);
static void f_mtimes(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *C);
static void g_binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
  const emxArray_real_T *in3, const emxArray_real_T *in4);
static void g_mtimes(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *C);
static void get_A(double T, b_sparse *A);
static void get_Q_prime(double T, b_sparse *Q_prime);
static double h_binary_expand_op(const emxArray_real_T *in1, const
  emxArray_real_T *in2, int *out2);
static void heapify(int x[45], int idx, int xstart, int xend);
static void i_binary_expand_op(emxArray_boolean_T *in1, const emxArray_real_T
  *in2, const c_TrajectoryOptimizer_DampedBFG *in3);
static void insertionsort(int x[45], int xstart, int xend);
static void introsort(int x[45]);
static void inv(emxArray_real_T *x);
static boolean_T isPositiveDefinite(const emxArray_real_T *B);
static void j_binary_expand_op(emxArray_real_T *in1, const
  c_TrajectoryOptimizer_solver_me *in2);
static double maximum(const emxArray_real_T *x, int *idx);
static double minimum(const emxArray_real_T *x, int *idx);
static void minus(emxArray_real_T *in1, const emxArray_real_T *in2);
static void mldivide(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *Y);
static void mtimes(const emxArray_real_T *A, const emxArray_real_T *B,
                   emxArray_real_T *C);
static void plus(emxArray_real_T *in1, const emxArray_real_T *in2);
static double rt_hypotd_snf(double u0, double u1);
static double rt_powd_snf(double u0, double u1);
static double rt_roundd_snf(double u);
static int solver_common_constructM(const c_TrajectoryOptimizer_solver_me *this,
  const emxArray_real_T *xCons, emxArray_real_T *M_d, emxArray_int32_T *M_colidx,
  emxArray_int32_T *M_rowidx, int *M_n);
static double solver_common_optimize(c_TrajectoryOptimizer_solver_me *this,
  const emxArray_real_T *initGuess, emxArray_real_T *p, emxArray_real_T *t, char
  exitstruct_Status_data[], int exitstruct_Status_size[2], double
  *exitstruct_Iterations, double *exitstruct_RRAttempts, double
  *exitstruct_Error, double *exitstruct_ExitFlag);
static void solver_common_optimize_anonFcn3(c_TrajectoryOptimizer_solver_me
  *this, const emxArray_real_T *varargin_1, emxArray_real_T *varargout_1);
static double solver_common_solvePoly(c_TrajectoryOptimizer_solver_me *this,
  const emxArray_real_T *T, emxArray_real_T *p);
static void spalloc(double m, double n, double nzmax, sparse *s);
static void sparse_ctranspose(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx, const emxArray_int32_T *this_rowidx, int this_m,
  int this_n, sparse *y);
static void sparse_fillIn(sparse *this);
static int sparse_locBsearch(const emxArray_int32_T *x, int xi, int xstart, int
  xend, boolean_T *found);
static void sparse_mldivide(const emxArray_real_T *A_d, const emxArray_int32_T
  *A_colidx, const emxArray_int32_T *A_rowidx, int A_m, int A_n, const
  emxArray_real_T *b, emxArray_real_T *y);
static int sparse_mtimes(const emxArray_real_T *a_d, const emxArray_int32_T
  *a_colidx, const emxArray_int32_T *a_rowidx, int a_m, const emxArray_real_T
  *b_d, const emxArray_int32_T *b_colidx, const emxArray_int32_T *b_rowidx, int
  b_n, emxArray_real_T *c_d, emxArray_int32_T *c_colidx, emxArray_int32_T
  *c_rowidx, int *c_n, int *c_maxnz);
static void sparse_parenAssign(sparse *this, const emxArray_real_T *rhs_d, const
  emxArray_int32_T *rhs_colidx, const emxArray_int32_T *rhs_rowidx, int rhs_m,
  const emxArray_real_T *varargin_1, const emxArray_real_T *varargin_2);
static int sparse_parenReference(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx, const emxArray_int32_T *this_rowidx, const
  emxArray_real_T *varargin_1, const emxArray_real_T *varargin_2,
  emxArray_real_T *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx,
  int *s_n, int *s_maxnz);
static double sum(const emxArray_real_T *x);
static double tic(double *tstart_tv_nsec);
static double toc(double tstart_tv_sec, double tstart_tv_nsec);
static void xgeqp3(emxArray_real_T *A, emxArray_real_T *tau, emxArray_int32_T
                   *jpvt);
static double xnrm2(int n, const emxArray_real_T *x, int ix0);
static int xzgetrf(int m, int n, emxArray_real_T *A, int lda, emxArray_int32_T
                   *ipiv);
static void CXSparseAPI_iteratedQR(const emxArray_real_T *A_d, const
  emxArray_int32_T *A_colidx, const emxArray_int32_T *A_rowidx, int A_m, int A_n,
  const emxArray_real_T *b, int n, emxArray_real_T *out)
{
  cs_di *cxA;
  cs_din *N;
  cs_dis *S;
  emxArray_real_T *outBuff;
  sparse expl_temp;
  const double *A_d_data;
  const double *b_data;
  double tol;
  double *outBuff_data;
  double *out_data;
  const int *A_colidx_data;
  const int *A_rowidx_data;
  int b_idx_0;
  int i;
  b_data = b->data;
  A_rowidx_data = A_rowidx->data;
  A_colidx_data = A_colidx->data;
  A_d_data = A_d->data;
  if (A_m < A_n) {
    emxInitStruct_sparse(&expl_temp);
    sparse_ctranspose(A_d, A_colidx, A_rowidx, A_m, A_n, &expl_temp);
    cxA = makeCXSparseMatrix(expl_temp.colidx->data[expl_temp.colidx->size[0] -
      1] - 1, expl_temp.n, expl_temp.m, &expl_temp.colidx->data[0],
      &expl_temp.rowidx->data[0], &expl_temp.d->data[0]);
    emxFreeStruct_sparse(&expl_temp);
  } else {
    cxA = makeCXSparseMatrix(A_colidx_data[A_colidx->size[0] - 1] - 1, A_n, A_m,
      &A_colidx_data[0], &A_rowidx_data[0], &A_d_data[0]);
  }

  S = cs_di_sqr(2, cxA, 1);
  N = cs_di_qr(cxA, S);
  cs_di_spfree(cxA);
  qr_rank_di(N, &tol);
  i = out->size[0];
  out->size[0] = n;
  emxEnsureCapacity_real_T(out, i);
  out_data = out->data;
  emxInit_real_T(&outBuff, 1);
  if (b->size[0] < n) {
    i = outBuff->size[0];
    outBuff->size[0] = n;
    emxEnsureCapacity_real_T(outBuff, i);
    outBuff_data = outBuff->data;
  } else {
    i = outBuff->size[0];
    outBuff->size[0] = b->size[0];
    emxEnsureCapacity_real_T(outBuff, i);
    outBuff_data = outBuff->data;
  }

  b_idx_0 = b->size[0];
  for (i = 0; i < b_idx_0; i++) {
    outBuff_data[i] = b_data[i];
  }

  solve_from_qr_di(N, S, (double *)&outBuff_data[0], b->size[0], n);
  if (n < 1) {
    b_idx_0 = 0;
  } else {
    b_idx_0 = n;
  }

  for (i = 0; i < b_idx_0; i++) {
    out_data[i] = outBuff_data[i];
  }

  emxFree_real_T(&outBuff);
  cs_di_sfree(S);
  cs_di_nfree(N);
}

static boolean_T any(const emxArray_real_T *x)
{
  const double *x_data;
  int k;
  boolean_T exitg1;
  boolean_T y;
  x_data = x->data;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= x->size[1] - 1)) {
    if ((fabs(x_data[k]) < DBL_EPSILON) || rtIsNaN(x_data[k])) {
      k++;
    } else {
      y = true;
      exitg1 = true;
    }
  }

  return y;
}

static boolean_T b_any(const emxArray_boolean_T *x)
{
  int k;
  const boolean_T *x_data;
  boolean_T exitg1;
  boolean_T y;
  x_data = x->data;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= x->size[0] - 1)) {
    if (x_data[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

static void b_binary_expand_op(emxArray_real_T *in1, double in2, const
  emxArray_real_T *in3)
{
  emxArray_real_T *b_in1;
  const double *in3_data;
  double *b_in1_data;
  double *in1_data;
  int aux_0_1;
  int aux_1_1;
  int b_loop_ub;
  int i;
  int i1;
  int loop_ub;
  int stride_0_0;
  int stride_0_1;
  int stride_1_0;
  int stride_1_1;
  in3_data = in3->data;
  in1_data = in1->data;
  emxInit_real_T(&b_in1, 2);
  if (in3->size[1] == 1) {
    loop_ub = in1->size[1];
  } else {
    loop_ub = in3->size[1];
  }

  i = b_in1->size[0] * b_in1->size[1];
  b_in1->size[1] = loop_ub;
  if (in3->size[0] == 1) {
    b_loop_ub = in1->size[0];
  } else {
    b_loop_ub = in3->size[0];
  }

  b_in1->size[0] = b_loop_ub;
  emxEnsureCapacity_real_T(b_in1, i);
  b_in1_data = b_in1->data;
  stride_0_0 = (in1->size[1] != 1);
  stride_0_1 = (in1->size[0] != 1);
  stride_1_0 = (in3->size[1] != 1);
  stride_1_1 = (in3->size[0] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < b_loop_ub; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      b_in1_data[i1 + b_in1->size[1] * i] = in1_data[i1 * stride_0_0 + in1->
        size[1] * aux_0_1] - in2 * in3_data[i1 * stride_1_0 + in3->size[1] *
        aux_1_1];
    }

    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }

  i = in1->size[0] * in1->size[1];
  in1->size[1] = b_in1->size[1];
  in1->size[0] = b_in1->size[0];
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  loop_ub = b_in1->size[0];
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = b_in1->size[1];
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      in1_data[i1 + in1->size[1] * i] = b_in1_data[i1 + b_in1->size[1] * i];
    }
  }

  emxFree_real_T(&b_in1);
}

static void b_eml_find(const emxArray_int32_T *x_colidx, const emxArray_int32_T *
  x_rowidx, emxArray_int32_T *i, emxArray_int32_T *j)
{
  const int *x_colidx_data;
  const int *x_rowidx_data;
  int idx;
  int nx_tmp;
  int *i_data;
  int *j_data;
  x_rowidx_data = x_rowidx->data;
  x_colidx_data = x_colidx->data;
  nx_tmp = x_colidx_data[x_colidx->size[0] - 1] - 1;
  if (nx_tmp == 0) {
    i->size[0] = 0;
    j->size[0] = 0;
  } else {
    int col;
    idx = i->size[0];
    i->size[0] = nx_tmp;
    emxEnsureCapacity_int32_T(i, idx);
    i_data = i->data;
    idx = j->size[0];
    j->size[0] = nx_tmp;
    emxEnsureCapacity_int32_T(j, idx);
    j_data = j->data;
    for (idx = 0; idx < nx_tmp; idx++) {
      i_data[idx] = x_rowidx_data[idx];
    }

    idx = 0;
    col = 1;
    while (idx < nx_tmp) {
      if (idx == x_colidx_data[col] - 1) {
        col++;
      } else {
        idx++;
        j_data[idx - 1] = col;
      }
    }

    if (nx_tmp == 1) {
      if (idx == 0) {
        i->size[0] = 0;
        j->size[0] = 0;
      }
    } else {
      if (idx < 1) {
        idx = 0;
      }

      col = i->size[0];
      i->size[0] = idx;
      emxEnsureCapacity_int32_T(i, col);
      col = j->size[0];
      j->size[0] = idx;
      emxEnsureCapacity_int32_T(j, col);
    }
  }
}

static void b_heapify(int x[100], int idx, int xstart, int xend)
{
  int ai;
  int aj;
  int extremum;
  int extremumIdx;
  int leftIdx;
  boolean_T changed;
  boolean_T exitg1;
  boolean_T varargout_1;
  changed = true;
  extremumIdx = (idx + xstart) - 2;
  leftIdx = ((idx << 1) + xstart) - 2;
  exitg1 = false;
  while ((!exitg1) && (leftIdx + 1 < xend)) {
    int aj_tmp_tmp;
    int cmpIdx;
    int xcmp;
    changed = false;
    extremum = x[extremumIdx];
    cmpIdx = leftIdx;
    xcmp = x[leftIdx];
    ai = iv2[x[leftIdx] - 1];
    aj_tmp_tmp = x[leftIdx + 1];
    aj = iv2[aj_tmp_tmp - 1];
    if (ai < aj) {
      varargout_1 = true;
    } else if (ai == aj) {
      varargout_1 = (iv3[x[leftIdx] - 1] < iv3[aj_tmp_tmp - 1]);
    } else {
      varargout_1 = false;
    }

    if (varargout_1) {
      cmpIdx = leftIdx + 1;
      xcmp = aj_tmp_tmp;
    }

    ai = iv2[x[extremumIdx] - 1];
    aj_tmp_tmp = iv2[xcmp - 1];
    if (ai < aj_tmp_tmp) {
      varargout_1 = true;
    } else if (ai == aj_tmp_tmp) {
      varargout_1 = (iv3[x[extremumIdx] - 1] < iv3[xcmp - 1]);
    } else {
      varargout_1 = false;
    }

    if (varargout_1) {
      x[extremumIdx] = xcmp;
      x[cmpIdx] = extremum;
      extremumIdx = cmpIdx;
      leftIdx = ((((cmpIdx - xstart) + 2) << 1) + xstart) - 2;
      changed = true;
    } else {
      exitg1 = true;
    }
  }

  if (changed && (leftIdx + 1 <= xend)) {
    extremum = x[extremumIdx];
    ai = iv2[x[extremumIdx] - 1];
    aj = iv2[x[leftIdx] - 1];
    if (ai < aj) {
      varargout_1 = true;
    } else if (ai == aj) {
      varargout_1 = (iv3[x[extremumIdx] - 1] < iv3[x[leftIdx] - 1]);
    } else {
      varargout_1 = false;
    }

    if (varargout_1) {
      x[extremumIdx] = x[leftIdx];
      x[leftIdx] = extremum;
    }
  }
}

static void b_heapsort(int x[45], int xstart, int xend)
{
  int idx;
  int k;
  int n;
  n = (xend - xstart) - 1;
  for (idx = n + 2; idx >= 1; idx--) {
    heapify(x, idx, xstart, xend);
  }

  for (k = 0; k <= n; k++) {
    int t;
    idx = (xend - k) - 1;
    t = x[idx];
    x[idx] = x[xstart - 1];
    x[xstart - 1] = t;
    heapify(x, 1, xstart, idx);
  }
}

static void b_insertionsort(int x[100], int xstart, int xend)
{
  int i;
  int k;
  i = xstart + 1;
  for (k = i; k <= xend; k++) {
    int idx;
    int xc;
    boolean_T exitg1;
    xc = x[k - 1];
    idx = k - 1;
    exitg1 = false;
    while ((!exitg1) && (idx >= xstart)) {
      int aj;
      int aj_tmp_tmp;
      int i1;
      boolean_T varargout_1;
      aj_tmp_tmp = x[idx - 1];
      aj = iv2[aj_tmp_tmp - 1];
      i1 = iv2[xc - 1];
      if (i1 < aj) {
        varargout_1 = true;
      } else if (i1 == aj) {
        varargout_1 = (iv3[xc - 1] < iv3[aj_tmp_tmp - 1]);
      } else {
        varargout_1 = false;
      }

      if (varargout_1) {
        x[idx] = aj_tmp_tmp;
        idx--;
      } else {
        exitg1 = true;
      }
    }

    x[idx] = xc;
  }
}

static void b_introsort(int x[100])
{
  struct_T st_d[24];
  int i;
  int st_n;
  for (i = 0; i < 24; i++) {
    st_d[i].xstart = 1;
    st_d[i].xend = 100;
    st_d[i].depth = 0;
  }

  st_d[0].xstart = 1;
  st_d[0].xend = 100;
  st_d[0].depth = 0;
  st_n = 1;
  while (st_n > 0) {
    struct_T expl_temp;
    int s_depth;
    int t;
    expl_temp = st_d[st_n - 1];
    s_depth = st_d[st_n - 1].depth;
    st_n--;
    t = expl_temp.xend - expl_temp.xstart;
    if (t + 1 <= 32) {
      b_insertionsort(x, expl_temp.xstart, expl_temp.xend);
    } else if (expl_temp.depth == 12) {
      c_heapsort(x, expl_temp.xstart, expl_temp.xend);
    } else {
      int ai;
      int aj;
      int j;
      int pivot;
      int xmid;
      boolean_T varargout_1;
      xmid = (expl_temp.xstart + t / 2) - 1;
      ai = iv2[x[xmid] - 1];
      t = x[expl_temp.xstart - 1];
      aj = iv2[t - 1];
      if (ai < aj) {
        varargout_1 = true;
      } else if (ai == aj) {
        varargout_1 = (iv3[x[xmid] - 1] < iv3[t - 1]);
      } else {
        varargout_1 = false;
      }

      if (varargout_1) {
        x[expl_temp.xstart - 1] = x[xmid];
        x[xmid] = t;
      }

      j = x[expl_temp.xend - 1];
      ai = iv2[j - 1];
      t = x[expl_temp.xstart - 1];
      aj = iv2[t - 1];
      if (ai < aj) {
        varargout_1 = true;
      } else if (ai == aj) {
        varargout_1 = (iv3[j - 1] < iv3[t - 1]);
      } else {
        varargout_1 = false;
      }

      if (varargout_1) {
        x[expl_temp.xstart - 1] = j;
        x[expl_temp.xend - 1] = t;
      }

      j = x[expl_temp.xend - 1];
      ai = iv2[j - 1];
      aj = iv2[x[xmid] - 1];
      if (ai < aj) {
        varargout_1 = true;
      } else if (ai == aj) {
        varargout_1 = (iv3[j - 1] < iv3[x[xmid] - 1]);
      } else {
        varargout_1 = false;
      }

      if (varargout_1) {
        t = x[xmid];
        x[xmid] = j;
        x[expl_temp.xend - 1] = t;
      }

      pivot = x[xmid] - 1;
      x[xmid] = x[expl_temp.xend - 2];
      x[expl_temp.xend - 2] = pivot + 1;
      i = expl_temp.xstart - 1;
      j = expl_temp.xend - 2;
      xmid = iv2[pivot];
      int exitg1;
      do {
        int exitg2;
        exitg1 = 0;
        i++;
        do {
          exitg2 = 0;
          ai = iv2[x[i] - 1];
          if (ai < xmid) {
            varargout_1 = true;
          } else if (ai == xmid) {
            varargout_1 = (iv3[x[i] - 1] < iv3[pivot]);
          } else {
            varargout_1 = false;
          }

          if (varargout_1) {
            i++;
          } else {
            exitg2 = 1;
          }
        } while (exitg2 == 0);

        j--;
        do {
          exitg2 = 0;
          aj = iv2[x[j] - 1];
          if (xmid < aj) {
            varargout_1 = true;
          } else if (xmid == aj) {
            varargout_1 = (iv3[pivot] < iv3[x[j] - 1]);
          } else {
            varargout_1 = false;
          }

          if (varargout_1) {
            j--;
          } else {
            exitg2 = 1;
          }
        } while (exitg2 == 0);

        if (i + 1 >= j + 1) {
          exitg1 = 1;
        } else {
          t = x[i];
          x[i] = x[j];
          x[j] = t;
        }
      } while (exitg1 == 0);

      x[expl_temp.xend - 2] = x[i];
      x[i] = pivot + 1;
      if (i + 2 < expl_temp.xend) {
        st_d[st_n].xstart = i + 2;
        st_d[st_n].xend = expl_temp.xend;
        st_d[st_n].depth = s_depth + 1;
        st_n++;
      }

      if (expl_temp.xstart < i + 1) {
        st_d[st_n].xstart = expl_temp.xstart;
        st_d[st_n].xend = i + 1;
        st_d[st_n].depth = s_depth + 1;
        st_n++;
      }
    }
  }
}

static void b_minus(emxArray_real_T *in1, const emxArray_real_T *in2, const
                    emxArray_real_T *in3)
{
  const double *in2_data;
  const double *in3_data;
  double *in1_data;
  int aux_0_1;
  int aux_1_1;
  int b_loop_ub;
  int i;
  int i1;
  int loop_ub;
  int stride_0_0;
  int stride_0_1;
  int stride_1_0;
  int stride_1_1;
  in3_data = in3->data;
  in2_data = in2->data;
  if (in3->size[1] == 1) {
    loop_ub = in2->size[1];
  } else {
    loop_ub = in3->size[1];
  }

  i = in1->size[0] * in1->size[1];
  in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(in1, i);
  if (in3->size[0] == 1) {
    b_loop_ub = in2->size[0];
  } else {
    b_loop_ub = in3->size[0];
  }

  i = in1->size[0] * in1->size[1];
  in1->size[0] = b_loop_ub;
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  stride_0_0 = (in2->size[1] != 1);
  stride_0_1 = (in2->size[0] != 1);
  stride_1_0 = (in3->size[1] != 1);
  stride_1_1 = (in3->size[0] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < b_loop_ub; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      in1_data[i1 + in1->size[1] * i] = in2_data[i1 * stride_0_0 + in2->size[1] *
        aux_0_1] - in3_data[i1 * stride_1_0 + in3->size[1] * aux_1_1];
    }

    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
}

static void b_mtimes(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *C)
{
  const double *A_data;
  const double *B_data;
  double *C_data;
  int inner;
  int j;
  int k;
  int n;
  B_data = B->data;
  A_data = A->data;
  inner = A->size[0];
  n = B->size[1];
  j = C->size[0] * C->size[1];
  C->size[1] = B->size[1];
  C->size[0] = 1;
  emxEnsureCapacity_real_T(C, j);
  C_data = C->data;
  for (j = 0; j < n; j++) {
    double s;
    s = 0.0;
    for (k = 0; k < inner; k++) {
      s += A_data[k] * B_data[j + B->size[1] * k];
    }

    C_data[j] = s;
  }
}

static double b_norm(const emxArray_real_T *x)
{
  const double *x_data;
  double y;
  int k;
  x_data = x->data;
  if (x->size[0] == 0) {
    y = 0.0;
  } else {
    y = 0.0;
    if (x->size[0] == 1) {
      y = fabs(x_data[0]);
    } else {
      double scale;
      int kend;
      scale = 3.3121686421112381E-170;
      kend = x->size[0];
      for (k = 0; k < kend; k++) {
        double absxk;
        absxk = fabs(x_data[k]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

static void b_plus(emxArray_real_T *in1, const emxArray_real_T *in2, const
                   emxArray_real_T *in3)
{
  const double *in2_data;
  const double *in3_data;
  double *in1_data;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  in3_data = in3->data;
  in2_data = in2->data;
  if (in3->size[0] == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = in3->size[0];
  }

  i = in1->size[0];
  in1->size[0] = loop_ub;
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in3->size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = in2_data[i * stride_0_0] + in3_data[i * stride_1_0];
  }
}

static double b_solver_common_solvePoly(c_TrajectoryOptimizer_solver_me *this,
  const emxArray_real_T *T, emxArray_real_T *p)
{

  DELAY_FUNC1("Entry", PX4_SOLVER_WAIT_TIME_US)
  b_sparse b_expl_temp;
  b_sparse c_expl_temp;
  emxArray_boolean_T *fixed_constr_ids_tmp;
  emxArray_boolean_T *nontriv_dim;
  emxArray_int32_T *M_colidx;
  emxArray_int32_T *M_rowidx;
  emxArray_int32_T *RPF_colidx;
  emxArray_int32_T *RPF_rowidx;
  emxArray_int32_T *RPP_colidx;
  emxArray_int32_T *RPP_rowidx;
  emxArray_int32_T *R_colidx;
  emxArray_int32_T *R_rowidx;
  emxArray_int32_T *ii;
  emxArray_int32_T *jj;
  emxArray_int32_T *r1;
  emxArray_real_T *D;
  emxArray_real_T *DP;
  emxArray_real_T *M_d;
  emxArray_real_T *RPF_d;
  emxArray_real_T *RPP_d;
  emxArray_real_T *R_d;
  emxArray_real_T *b_this;
  emxArray_real_T *b_waypoints;
  emxArray_real_T *c_p;
  emxArray_real_T *d_p;
  emxArray_real_T *e_p;
  emxArray_real_T *r;
  emxArray_real_T *r2;
  emxArray_real_T *tmp_bcs;
  emxArray_real_T *tmp_i_col;
  emxArray_real_T *tmp_i_row;
  emxArray_real_T *waypoints;
  emxArray_real_T *y;
  sparse A_total_sp;
  sparse Q_prime_total_sp;
  sparse d_expl_temp;
  const double *T_data;
  double J = 0.0;
  double numSegments = 0.0;
  double *DP_data;
  double *D_data;
  double *RPP_d_data;
  double *R_d_data;
  double *b_p_data;
  double *b_waypoints_data;
  double *c_p_data;
  double *p_data;
  double *tmp_bcs_data;
  double *tmp_i_col_data;
  double *tmp_i_row_data;
  double *waypoints_data;
  int M_n = 0;
  int RPF_n = 0;
  int RPP_n = 0;
  int R_n = 0;
  int b_ii = 0;
  int dimIdx = 0;
  int expl_temp = 0;
  int i = 0;
  int i1 = 0;
  int i_dim = 0;
  int k = 0;
  int loop_ub = 0;
  int nz = 0;
  int segment = 0;
  int this_idx_0_tmp = 0;
  int unnamed_idx_1 = 0;
  int vlen = 0;
  int *R_colidx_data;
  int *R_rowidx_data;
  int *ii_data;
  int *jj_data;
  boolean_T *fixed_constr_ids_tmp_data;
  boolean_T *nontriv_dim_data;
  DELAY_FUNC("Declarations", PX4_SOLVER_WAIT_TIME_US)
  T_data = T->data;
  vlen = this->wptFnc.workspace.wpts->size[0];
  i = this->waypoints_offset->size[0];
  this->waypoints_offset->size[0] = vlen;
  emxEnsureCapacity_real_T(this->waypoints_offset, i);
  for (i = 0; i < vlen; i++) {
    this->waypoints_offset->data[i] = this->wptFnc.workspace.wpts->data
      [this->wptFnc.workspace.wpts->size[1] * i];
  }

  emxInit_real_T(&waypoints, 2);
  if (this->waypoints_offset->size[0] == this->wptFnc.workspace.wpts->size[0]) {
    i = waypoints->size[0] * waypoints->size[1];
    waypoints->size[1] = this->wptFnc.workspace.wpts->size[1];
    waypoints->size[0] = this->wptFnc.workspace.wpts->size[0];
    emxEnsureCapacity_real_T(waypoints, i);
    waypoints_data = waypoints->data;
    loop_ub = this->wptFnc.workspace.wpts->size[0];
    for (i = 0; i < loop_ub; i++) {
      vlen = this->wptFnc.workspace.wpts->size[1];
      for (nz = 0; nz < vlen; nz++) {
        waypoints_data[nz + waypoints->size[1] * i] =
          this->wptFnc.workspace.wpts->data[nz + this->
          wptFnc.workspace.wpts->size[1] * i] - this->waypoints_offset->data[i];
      }
    }
  } else {
    j_binary_expand_op(waypoints, this);
    waypoints_data = waypoints->data;
  }

  this->n_dim_src = waypoints->size[0];
  this->N_wps = waypoints->size[1];
  this->N_segments = this->N_wps - 1.0;
  this->stateSize = 10.0 * this->N_segments;
  this_idx_0_tmp = (int)this->n_dim_src;
  emxInit_boolean_T(&nontriv_dim);
  i = nontriv_dim->size[0];
  nontriv_dim->size[0] = this_idx_0_tmp;
  emxEnsureCapacity_boolean_T(nontriv_dim, i);
  nontriv_dim_data = nontriv_dim->data;
  for (i = 0; i < this_idx_0_tmp; i++) {
    nontriv_dim_data[i] = false;
  }

  if (this_idx_0_tmp - 1 >= 0) {
    k = waypoints->size[1];
    expl_temp = waypoints->size[1];
  }

  emxInit_real_T(&b_waypoints, 2);
  for (i_dim = 0; i_dim < this_idx_0_tmp; i_dim++) {
    i = b_waypoints->size[0] * b_waypoints->size[1];
    b_waypoints->size[1] = k;
    b_waypoints->size[0] = 1;
    emxEnsureCapacity_real_T(b_waypoints, i);
    b_waypoints_data = b_waypoints->data;
    for (i = 0; i < expl_temp; i++) {
      b_waypoints_data[i] = waypoints_data[i + waypoints->size[1] * i_dim];
    }

    nontriv_dim_data[i_dim] = any(b_waypoints);
  }

  DELAY_FUNC("1", PX4_SOLVER_WAIT_TIME_US)

  expl_temp = 0;
  i = nontriv_dim->size[0];
  for (k = 0; k < i; k++) {
    if (nontriv_dim_data[k]) {
      expl_temp++;
    }
  }

  emxInit_int32_T(&ii, 1);
  eml_find(nontriv_dim, ii);
  ii_data = ii->data;
  i = this->n_dim_ids->size[0];
  this->n_dim_ids->size[0] = ii->size[0];
  emxEnsureCapacity_real_T(this->n_dim_ids, i);
  loop_ub = ii->size[0];
  for (i = 0; i < loop_ub; i++) {
    this->n_dim_ids->data[i] = ii_data[i];
  }

  this->n_dim = expl_temp;
  k = nontriv_dim->size[0] - 1;
  vlen = 0;
  for (loop_ub = 0; loop_ub <= k; loop_ub++) {
    if (nontriv_dim_data[loop_ub]) {
      vlen++;
    }
  }

  emxInit_int32_T(&jj, 1);
  i = jj->size[0];
  jj->size[0] = vlen;
  emxEnsureCapacity_int32_T(jj, i);
  jj_data = jj->data;
  vlen = 0;
  for (loop_ub = 0; loop_ub <= k; loop_ub++) {
    if (nontriv_dim_data[loop_ub]) {
      jj_data[vlen] = loop_ub;
      vlen++;
    }
  }

  emxFree_boolean_T(&nontriv_dim);
  i = this->waypoints->size[0] * this->waypoints->size[1];
  this->waypoints->size[1] = waypoints->size[1];
  this->waypoints->size[0] = jj->size[0];
  emxEnsureCapacity_real_T(this->waypoints, i);
  loop_ub = jj->size[0];
  for (i = 0; i < loop_ub; i++) {
    vlen = waypoints->size[1];
    for (nz = 0; nz < vlen; nz++) {
      this->waypoints->data[nz + this->waypoints->size[1] * i] =
        waypoints_data[nz + waypoints->size[1] * jj_data[i]];
    }
  }

  DELAY_FUNC("2", PX4_SOLVER_WAIT_TIME_US)

  emxFree_real_T(&waypoints);
  vlen = (int)(5.0 * this->N_wps);
  i = this->constraints->size[0] * this->constraints->size[1];
  this->constraints->size[1] = expl_temp;
  this->constraints->size[0] = vlen;
  emxEnsureCapacity_real_T(this->constraints, i);
  for (i = 0; i < vlen; i++) {
    for (nz = 0; nz < expl_temp; nz++) {
      this->constraints->data[nz + this->constraints->size[1] * i] = 0.0;
    }
  }

  DELAY_FUNC("3", PX4_SOLVER_WAIT_TIME_US)

  if (expl_temp > 0) {
    i = (int)this->N_wps;
    emxInit_real_T(&tmp_bcs, 2);
    for (k = 0; k < i; k++) {
      boolean_T b_p;
      vlen = (int)this->n_dim;
      nz = tmp_bcs->size[0] * tmp_bcs->size[1];
      tmp_bcs->size[1] = 5;
      tmp_bcs->size[0] = vlen;
      emxEnsureCapacity_real_T(tmp_bcs, nz);
      tmp_bcs_data = tmp_bcs->data;
      for (nz = 0; nz < vlen; nz++) {
        for (expl_temp = 0; expl_temp < 5; expl_temp++) {
          tmp_bcs_data[expl_temp + 5 * nz] = 0.0;
        }
      }

      loop_ub = this->waypoints->size[0];
      for (nz = 0; nz < loop_ub; nz++) {
        tmp_bcs_data[5 * nz] = this->waypoints->data[k + this->waypoints->size[1]
          * nz];
      }

      b_p = ((unsigned int)k + 1U == 1U);
      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        if ((!b_p) && (!(fabs((double)k + 1.0 - this->N_wps) < DBL_EPSILON))) {
          for (nz = 0; nz < vlen; nz++) {
            tmp_bcs_data[(loop_ub + 5 * nz) + 1] = rtNaN;
          }
        }
      }

      for (loop_ub = 0; loop_ub < vlen; loop_ub++) {
        for (b_ii = 0; b_ii < 5; b_ii++) {
          this->constraints->data[loop_ub + this->constraints->size[1] * ((int)
            (((double)b_ii + 1.0) + (((double)k + 1.0) - 1.0) * 5.0) - 1)] =
            tmp_bcs_data[b_ii + 5 * loop_ub];
        }
      }
    }

    emxFree_real_T(&tmp_bcs);
  }

  DELAY_FUNC("4", PX4_SOLVER_WAIT_TIME_US)

  this_idx_0_tmp = (int)this->n_dim;
  vlen = (int)this->N_segments;
  i = p->size[0] * p->size[1] * p->size[2];
  p->size[2] = this_idx_0_tmp;
  p->size[1] = 10;
  p->size[0] = vlen;
  emxEnsureCapacity_real_T(p, i);
  p_data = p->data;
  for (i = 0; i < vlen; i++) {
    for (nz = 0; nz < 10; nz++) {
      for (expl_temp = 0; expl_temp < this_idx_0_tmp; expl_temp++) {
        p_data[(expl_temp + p->size[2] * nz) + p->size[2] * 10 * i] = 0.0;
      }
    }
  }

  DELAY_FUNC("5", PX4_SOLVER_WAIT_TIME_US)

  J = 0.0;
  if (this_idx_0_tmp - 1 >= 0) {
    numSegments = this->N_segments;
    i1 = (int)numSegments;
    unnamed_idx_1 = (int)numSegments;
  }

  DELAY_FUNC("6", PX4_SOLVER_WAIT_TIME_US)

  emxInit_real_T(&r, 2);
  emxInitStruct_sparse(&A_total_sp);
  emxInitStruct_sparse(&Q_prime_total_sp);
  emxInit_real_T(&M_d, 1);
  emxInit_int32_T(&M_colidx, 1);
  emxInit_int32_T(&M_rowidx, 1);
  emxInit_real_T(&R_d, 1);
  emxInit_int32_T(&R_colidx, 1);
  emxInit_int32_T(&R_rowidx, 1);
  emxInit_real_T(&RPP_d, 1);
  emxInit_int32_T(&RPP_colidx, 1);
  emxInit_int32_T(&RPP_rowidx, 1);
  emxInit_real_T(&RPF_d, 1);
  emxInit_int32_T(&RPF_colidx, 1);
  emxInit_int32_T(&RPF_rowidx, 1);
  emxInit_real_T(&DP, 1);
  emxInit_real_T(&D, 1);
  emxInit_real_T(&tmp_i_row, 1);
  emxInit_real_T(&tmp_i_col, 1);
  emxInit_real_T(&c_p, 1);
  emxInit_real_T(&d_p, 2);
  emxInit_int32_T(&r1, 1);
  emxInit_boolean_T(&fixed_constr_ids_tmp);
  emxInit_real_T(&y, 2);
  emxInitStruct_sparse1(&b_expl_temp);
  emxInitStruct_sparse1(&c_expl_temp);
  emxInitStruct_sparse(&d_expl_temp);
  emxInit_real_T(&b_this, 1);
  emxInit_real_T(&e_p, 2);
  emxInit_real_T(&r2, 1);

  DELAY_FUNC("7", PX4_SOLVER_WAIT_TIME_US)
  for (dimIdx = 0; dimIdx < this_idx_0_tmp; dimIdx++) {
    double cd;
    int M_m;
    int RPF_m;
    int RPP_m;
    int R_m;
    int b_y;
    vlen = this->constraints->size[0];
    i = fixed_constr_ids_tmp->size[0];
    fixed_constr_ids_tmp->size[0] = vlen;
    emxEnsureCapacity_boolean_T(fixed_constr_ids_tmp, i);
    fixed_constr_ids_tmp_data = fixed_constr_ids_tmp->data;
    for (i = 0; i < vlen; i++) {
      fixed_constr_ids_tmp_data[i] = !rtIsNaN(this->constraints->data[dimIdx +
        this->constraints->size[1] * i]);
    }

    vlen = fixed_constr_ids_tmp->size[0];
    if (fixed_constr_ids_tmp->size[0] == 0) {
      nz = 0;
    } else {
      nz = fixed_constr_ids_tmp_data[0];
      for (k = 2; k <= vlen; k++) {
        if (vlen >= 2) {
          nz += fixed_constr_ids_tmp_data[k - 1];
        }
      }
    }

    vlen = fixed_constr_ids_tmp->size[0];
    if (fixed_constr_ids_tmp->size[0] == 0) {
      b_y = 0;
    } else {
      b_y = fixed_constr_ids_tmp_data[0];
      for (k = 2; k <= vlen; k++) {
        if (vlen >= 2) {
          b_y += fixed_constr_ids_tmp_data[k - 1];
        }
      }
    }

    DELAY_FUNC("8", PX4_SOLVER_WAIT_TIME_US)

    spalloc(10.0 * numSegments, 10.0 * numSegments, 100.0 * numSegments,
            &A_total_sp);
    DELAY_FUNC("spalloc", PX4_SOLVER_WAIT_TIME_US)

    spalloc(10.0 * numSegments, 10.0 * numSegments, 100.0 * numSegments,
            &Q_prime_total_sp);
    DELAY_FUNC("spalloc", PX4_SOLVER_WAIT_TIME_US)

    for (segment = 0; segment < i1; segment++) {
      double offset;
      offset = (((double)segment + 1.0) - 1.0) * 10.0;
      DELAY_FUNC("still ok? ", PX4_SOLVER_WAIT_TIME_US)

      cd = T_data[segment];
      DELAY_FUNC("Ok.. here we go...", 10*PX4_SOLVER_WAIT_TIME_US)

      get_A(cd, &b_expl_temp);
      DELAY_FUNC("get_A total ", PX4_SOLVER_WAIT_TIME_US)

      get_Q_prime(cd, &c_expl_temp);
      DELAY_FUNC("get_Q_prime total ", PX4_SOLVER_WAIT_TIME_US)

      b_eml_find(b_expl_temp.colidx, b_expl_temp.rowidx, ii, jj);

      DELAY_FUNC("9", PX4_SOLVER_WAIT_TIME_US)
      jj_data = jj->data;
      ii_data = ii->data;
      loop_ub = ii->size[0];
      i = tmp_i_row->size[0];
      tmp_i_row->size[0] = ii->size[0];
      emxEnsureCapacity_real_T(tmp_i_row, i);
      tmp_i_row_data = tmp_i_row->data;
      for (i = 0; i < loop_ub; i++) {
        tmp_i_row_data[i] = ii_data[i];
      }

      loop_ub = jj->size[0];
      i = tmp_i_col->size[0];
      tmp_i_col->size[0] = jj->size[0];
      emxEnsureCapacity_real_T(tmp_i_col, i);
      tmp_i_col_data = tmp_i_col->data;
      for (i = 0; i < loop_ub; i++) {
        tmp_i_col_data[i] = jj_data[i];
      }

      vlen = sparse_parenReference(b_expl_temp.d, b_expl_temp.colidx,
        b_expl_temp.rowidx, tmp_i_row, tmp_i_col, d_expl_temp.d,
        d_expl_temp.colidx, d_expl_temp.rowidx, &k, &expl_temp);
      loop_ub = tmp_i_row->size[0];
      for (i = 0; i < loop_ub; i++) {
        tmp_i_row_data[i] += offset;
      }

      loop_ub = tmp_i_col->size[0];
      for (i = 0; i < loop_ub; i++) {
        tmp_i_col_data[i] += offset;
      }

      sparse_parenAssign(&A_total_sp, d_expl_temp.d, d_expl_temp.colidx,
                         d_expl_temp.rowidx, vlen, tmp_i_row, tmp_i_col);
      b_eml_find(c_expl_temp.colidx, c_expl_temp.rowidx, ii, jj);
      jj_data = jj->data;
      ii_data = ii->data;
      loop_ub = ii->size[0];
      i = tmp_i_row->size[0];
      tmp_i_row->size[0] = ii->size[0];
      emxEnsureCapacity_real_T(tmp_i_row, i);
      tmp_i_row_data = tmp_i_row->data;
      for (i = 0; i < loop_ub; i++) {
        tmp_i_row_data[i] = ii_data[i];
      }

      DELAY_FUNC("10", PX4_SOLVER_WAIT_TIME_US)

      loop_ub = jj->size[0];
      i = tmp_i_col->size[0];
      tmp_i_col->size[0] = jj->size[0];
      emxEnsureCapacity_real_T(tmp_i_col, i);
      tmp_i_col_data = tmp_i_col->data;
      for (i = 0; i < loop_ub; i++) {
        tmp_i_col_data[i] = jj_data[i];
      }

      vlen = sparse_parenReference(c_expl_temp.d, c_expl_temp.colidx,
        c_expl_temp.rowidx, tmp_i_row, tmp_i_col, d_expl_temp.d,
        d_expl_temp.colidx, d_expl_temp.rowidx, &k, &expl_temp);
      loop_ub = tmp_i_row->size[0];
      for (i = 0; i < loop_ub; i++) {
        tmp_i_row_data[i] += offset;
      }

      loop_ub = tmp_i_col->size[0];
      for (i = 0; i < loop_ub; i++) {
        tmp_i_col_data[i] += offset;
      }

      sparse_parenAssign(&Q_prime_total_sp, d_expl_temp.d, d_expl_temp.colidx,
                         d_expl_temp.rowidx, vlen, tmp_i_row, tmp_i_col);
    }

    vlen = this->constraints->size[0];
    i = b_this->size[0];
    b_this->size[0] = vlen;
    emxEnsureCapacity_real_T(b_this, i);
    b_waypoints_data = b_this->data;
    for (i = 0; i < vlen; i++) {
      b_waypoints_data[i] = this->constraints->data[dimIdx + this->
        constraints->size[1] * i];
    }

    DELAY_FUNC("11", PX4_SOLVER_WAIT_TIME_US)

    M_m = solver_common_constructM(this, b_this, M_d, M_colidx, M_rowidx, &M_n);
    sparse_ctranspose(M_d, M_colidx, M_rowidx, M_m, M_n, &d_expl_temp);
    vlen = sparse_mtimes(d_expl_temp.d, d_expl_temp.colidx, d_expl_temp.rowidx,
                         d_expl_temp.m, Q_prime_total_sp.d,
                         Q_prime_total_sp.colidx, Q_prime_total_sp.rowidx,
                         Q_prime_total_sp.n, b_this, jj, ii, &k, &expl_temp);
    R_m = sparse_mtimes(b_this, jj, ii, vlen, M_d, M_colidx, M_rowidx, M_n, R_d,
                        R_colidx, R_rowidx, &R_n, &k);
    R_rowidx_data = R_rowidx->data;
    R_colidx_data = R_colidx->data;
    R_d_data = R_d->data;
    k = fixed_constr_ids_tmp->size[0] - 1;
    vlen = 0;
    for (loop_ub = 0; loop_ub <= k; loop_ub++) {
      if (fixed_constr_ids_tmp_data[loop_ub]) {
        vlen++;
      }
    }

    i = r1->size[0];
    r1->size[0] = vlen;
    emxEnsureCapacity_int32_T(r1, i);
    ii_data = r1->data;
    vlen = 0;
    for (loop_ub = 0; loop_ub <= k; loop_ub++) {
      if (fixed_constr_ids_tmp_data[loop_ub]) {
        ii_data[vlen] = loop_ub;
        vlen++;
      }
    }

    if (R_m < (double)b_y + 1.0) {
      b_waypoints->size[1] = 0;
      b_waypoints->size[0] = 1;
    } else {
      k = R_m - nz;
      i = b_waypoints->size[0] * b_waypoints->size[1];
      b_waypoints->size[1] = k;
      b_waypoints->size[0] = 1;
      emxEnsureCapacity_real_T(b_waypoints, i);
      b_waypoints_data = b_waypoints->data;
      for (i = 0; i < k; i++) {
        b_waypoints_data[i] = ((double)nz + 1.0) + (double)i;
      }
    }

    if (R_n < (double)b_y + 1.0) {
      y->size[1] = 0;
      y->size[0] = 1;
    } else {
      k = R_n - nz;
      i = y->size[0] * y->size[1];
      y->size[1] = k;
      y->size[0] = 1;
      emxEnsureCapacity_real_T(y, i);
      b_waypoints_data = y->data;
      for (i = 0; i < k; i++) {
        b_waypoints_data[i] = ((double)nz + 1.0) + (double)i;
      }
    }

    DELAY_FUNC("12", PX4_SOLVER_WAIT_TIME_US)

    RPP_m = b_sparse_parenReference(R_d, R_colidx, R_rowidx, b_waypoints, y,
      RPP_d, RPP_colidx, RPP_rowidx, &RPP_n, &k);
    RPP_d_data = RPP_d->data;
    if (R_m < (double)b_y + 1.0) {
      b_waypoints->size[1] = 0;
      b_waypoints->size[0] = 1;
    } else {
      k = R_m - nz;
      i = b_waypoints->size[0] * b_waypoints->size[1];
      b_waypoints->size[1] = k;
      b_waypoints->size[0] = 1;
      emxEnsureCapacity_real_T(b_waypoints, i);
      b_waypoints_data = b_waypoints->data;
      for (i = 0; i < k; i++) {
        b_waypoints_data[i] = ((double)nz + 1.0) + (double)i;
      }
    }

    if (nz < 1) {
      y->size[1] = 0;
      y->size[0] = 1;
    } else {
      i = y->size[0] * y->size[1];
      y->size[1] = b_y;
      y->size[0] = 1;
      emxEnsureCapacity_real_T(y, i);
      b_waypoints_data = y->data;
      for (i = 0; i < b_y; i++) {
        b_waypoints_data[i] = (double)i + 1.0;
      }
    }

    RPF_m = b_sparse_parenReference(R_d, R_colidx, R_rowidx, b_waypoints, y,
      RPF_d, RPF_colidx, RPF_rowidx, &RPF_n, &k);
    loop_ub = RPP_d->size[0];
    for (i = 0; i < loop_ub; i++) {
      RPP_d_data[i] = -RPP_d_data[i];
    }

    loop_ub = r1->size[0];
    i = b_this->size[0];
    b_this->size[0] = r1->size[0];
    emxEnsureCapacity_real_T(b_this, i);
    b_waypoints_data = b_this->data;
    for (i = 0; i < loop_ub; i++) {
      b_waypoints_data[i] = this->constraints->data[dimIdx + this->
        constraints->size[1] * ii_data[i]];
    }

    DELAY_FUNC("13", PX4_SOLVER_WAIT_TIME_US)

    b_sparse_mtimes(RPF_d, RPF_colidx, RPF_rowidx, RPF_m, RPF_n, b_this, r2);
    sparse_mldivide(RPP_d, RPP_colidx, RPP_rowidx, RPP_m, RPP_n, r2, DP);
    DP_data = DP->data;
    loop_ub = r1->size[0];
    i = D->size[0];
    D->size[0] = r1->size[0] + DP->size[0];
    emxEnsureCapacity_real_T(D, i);
    D_data = D->data;
    for (i = 0; i < loop_ub; i++) {
      D_data[i] = this->constraints->data[dimIdx + this->constraints->size[1] *
        ii_data[i]];
    }

    loop_ub = DP->size[0];
    for (i = 0; i < loop_ub; i++) {
      D_data[i + r1->size[0]] = DP_data[i];
    }

    b_sparse_mtimes(M_d, M_colidx, M_rowidx, M_m, M_n, D, r2);
    sparse_mldivide(A_total_sp.d, A_total_sp.colidx, A_total_sp.rowidx,
                    A_total_sp.m, A_total_sp.n, r2, c_p);
    b_p_data = c_p->data;
    i = 0;
    nz = 0;
    expl_temp = d_p->size[0] * d_p->size[1];
    d_p->size[1] = unnamed_idx_1;
    d_p->size[0] = 10;
    emxEnsureCapacity_real_T(d_p, expl_temp);
    c_p_data = d_p->data;
    for (expl_temp = 0; expl_temp < 10 * unnamed_idx_1; expl_temp++) {
      c_p_data[nz + d_p->size[1] * i] = b_p_data[expl_temp];
      i++;
      if (i > 9) {
        i = 0;
        nz++;
      }
    }

    DELAY_FUNC("14", PX4_SOLVER_WAIT_TIME_US)

    vlen = d_p->size[1];
    i = e_p->size[0] * e_p->size[1];
    e_p->size[1] = d_p->size[1];
    e_p->size[0] = 10;
    emxEnsureCapacity_real_T(e_p, i);
    b_p_data = e_p->data;
    for (i = 0; i < 10; i++) {
      for (nz = 0; nz < vlen; nz++) {
        b_p_data[nz + e_p->size[1] * i] = c_p_data[nz + d_p->size[1] * (9 - i)];
      }
    }

    i = d_p->size[0] * d_p->size[1];
    d_p->size[1] = e_p->size[1];
    d_p->size[0] = 10;
    emxEnsureCapacity_real_T(d_p, i);
    c_p_data = d_p->data;
    loop_ub = e_p->size[1];
    for (i = 0; i < 10; i++) {
      for (nz = 0; nz < loop_ub; nz++) {
        c_p_data[nz + d_p->size[1] * i] = b_p_data[nz + e_p->size[1] * i];
      }
    }

    DELAY_FUNC("15", PX4_SOLVER_WAIT_TIME_US)

    i = r->size[0] * r->size[1];
    r->size[1] = 10;
    loop_ub = d_p->size[1];
    r->size[0] = d_p->size[1];
    emxEnsureCapacity_real_T(r, i);
    b_waypoints_data = r->data;
    for (i = 0; i < loop_ub; i++) {
      for (nz = 0; nz < 10; nz++) {
        b_waypoints_data[nz + 10 * i] = c_p_data[i + d_p->size[1] * nz];
      }
    }

    loop_ub = r->size[0];
    for (i = 0; i < loop_ub; i++) {
      for (nz = 0; nz < 10; nz++) {
        p_data[(dimIdx + p->size[2] * nz) + p->size[2] * 10 * i] =
          b_waypoints_data[nz + 10 * i];
      }
    }

    i = b_waypoints->size[0] * b_waypoints->size[1];
    b_waypoints->size[1] = R_n;
    b_waypoints->size[0] = 1;
    emxEnsureCapacity_real_T(b_waypoints, i);
    b_waypoints_data = b_waypoints->data;
    for (i = 0; i < R_n; i++) {
      b_waypoints_data[i] = 0.0;
    }

    DELAY_FUNC("15", PX4_SOLVER_WAIT_TIME_US)

    if ((D->size[0] != 0) && (R_n != 0) && (R_colidx_data[R_colidx->size[0] - 1]
         - 1 != 0)) {
      for (k = 0; k < R_n; k++) {
        cd = 0.0;
        vlen = R_colidx_data[k + 1] - 1;
        i = R_colidx_data[k];
        for (expl_temp = i; expl_temp <= vlen; expl_temp++) {
          cd += R_d_data[expl_temp - 1] * D_data[R_rowidx_data[expl_temp - 1] -
            1];
        }

        b_waypoints_data[k] = cd;
      }
    }

    DELAY_FUNC("16", PX4_SOLVER_WAIT_TIME_US)

    cd = 0.0;
    loop_ub = D->size[0];
    for (i = 0; i < loop_ub; i++) {
      cd += D_data[i] * b_waypoints_data[i];
    }

    J += cd;
  }

  DELAY_FUNC("END of solution", PX4_SOLVER_WAIT_TIME_US)

  emxFree_real_T(&r2);
  emxFree_real_T(&e_p);
  emxFree_real_T(&b_this);
  emxFree_real_T(&b_waypoints);
  emxFreeStruct_sparse(&d_expl_temp);
  emxFreeStruct_sparse1(&c_expl_temp);
  emxFreeStruct_sparse1(&b_expl_temp);
  emxFree_real_T(&y);
  emxFree_int32_T(&jj);
  emxFree_int32_T(&ii);
  emxFree_boolean_T(&fixed_constr_ids_tmp);
  emxFree_int32_T(&r1);
  emxFree_real_T(&d_p);
  emxFree_real_T(&c_p);
  emxFree_real_T(&tmp_i_col);
  emxFree_real_T(&tmp_i_row);
  emxFree_real_T(&D);
  emxFree_real_T(&DP);
  emxFree_int32_T(&RPF_rowidx);
  emxFree_int32_T(&RPF_colidx);
  emxFree_real_T(&RPF_d);
  emxFree_int32_T(&RPP_rowidx);
  emxFree_int32_T(&RPP_colidx);
  emxFree_real_T(&RPP_d);
  emxFree_int32_T(&R_rowidx);
  emxFree_int32_T(&R_colidx);
  emxFree_real_T(&R_d);
  emxFree_int32_T(&M_rowidx);
  emxFree_int32_T(&M_colidx);
  emxFree_real_T(&M_d);
  emxFreeStruct_sparse(&Q_prime_total_sp);
  emxFreeStruct_sparse(&A_total_sp);
  emxFree_real_T(&r);

  DELAY_FUNC_END("Freed vars", PX4_SOLVER_WAIT_TIME_US)
  return J;
}

static void b_sparse_fillIn(b_sparse *this)
{

  DELAY_FUNC1("Entry", PX4_SOLVER_WAIT_TIME_US)
  int c;
  int i;
  int idx;
  idx = 1;
  i = this->colidx->size[0];
  for (c = 0; c <= i - 2; c++) {
    int ridx;
    ridx = this->colidx->data[c];
    this->colidx->data[c] = idx;
    int exitg1;
    int i1;
    do {
      exitg1 = 0;
      i1 = this->colidx->data[c + 1];
      DELAY_FUNC("do start", PX4_SOLVER_WAIT_TIME_US)
      if (ridx < i1) {
        double val;
        int currRowIdx;
        val = 0.0;
        currRowIdx = this->rowidx->data[ridx - 1];
        while ((ridx < i1) && (this->rowidx->data[ridx - 1] == currRowIdx)) {
          val += this->d->data[ridx - 1];
          ridx++;
          DELAY_FUNC("while inner", PX4_SOLVER_WAIT_TIME_US)
        }

        if (fabs(val) >= DBL_EPSILON) {
          this->d->data[idx - 1] = val;
          this->rowidx->data[idx - 1] = currRowIdx;
          idx++;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  this->colidx->data[this->colidx->size[0] - 1] = idx;

  DELAY_FUNC_END("End of function", PX4_SOLVER_WAIT_TIME_US)
}

static void b_sparse_mtimes(const emxArray_real_T *a_d, const emxArray_int32_T
  *a_colidx, const emxArray_int32_T *a_rowidx, int a_m, int a_n, const
  emxArray_real_T *b, emxArray_real_T *c)
{
  const double *a_d_data;
  const double *b_data;
  double *c_data;
  const int *a_colidx_data;
  const int *a_rowidx_data;
  int acol;
  int ap;
  int i;
  b_data = b->data;
  a_rowidx_data = a_rowidx->data;
  a_colidx_data = a_colidx->data;
  a_d_data = a_d->data;
  i = c->size[0];
  c->size[0] = a_m;
  emxEnsureCapacity_real_T(c, i);
  c_data = c->data;
  for (i = 0; i < a_m; i++) {
    c_data[i] = 0.0;
  }

  if ((a_n != 0) && (a_m != 0) && (a_colidx_data[a_colidx->size[0] - 1] - 1 != 0))
  {
    for (acol = 0; acol < a_n; acol++) {
      double bc;
      int nap;
      int nap_tmp;
      bc = b_data[acol];
      i = a_colidx_data[acol];
      nap_tmp = a_colidx_data[acol + 1];
      nap = nap_tmp - a_colidx_data[acol];
      if (nap >= 4) {
        int apend1;
        int c_tmp;
        apend1 = (nap_tmp - nap) + ((nap / 4) << 2);
        for (ap = i; ap <= apend1 - 1; ap += 4) {
          c_tmp = a_rowidx_data[ap - 1] - 1;
          c_data[c_tmp] += a_d_data[ap - 1] * bc;
          c_data[a_rowidx_data[ap] - 1] += a_d_data[ap] * bc;
          c_tmp = a_rowidx_data[ap + 1] - 1;
          c_data[c_tmp] += a_d_data[ap + 1] * bc;
          c_tmp = a_rowidx_data[ap + 2] - 1;
          c_data[c_tmp] += a_d_data[ap + 2] * bc;
        }

        nap = nap_tmp - 1;
        for (ap = apend1; ap <= nap; ap++) {
          c_tmp = a_rowidx_data[ap - 1] - 1;
          c_data[c_tmp] += a_d_data[ap - 1] * bc;
        }
      } else {
        nap = nap_tmp - 1;
        for (ap = i; ap <= nap; ap++) {
          int c_tmp;
          c_tmp = a_rowidx_data[ap - 1] - 1;
          c_data[c_tmp] += a_d_data[ap - 1] * bc;
        }
      }
    }
  }
}

static int b_sparse_parenReference(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx, const emxArray_int32_T *this_rowidx, const
  emxArray_real_T *varargin_1, const emxArray_real_T *varargin_2,
  emxArray_real_T *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx,
  int *s_n, int *s_maxnz)
{
  const double *this_d_data;
  const double *varargin_1_data;
  const double *varargin_2_data;
  double *s_d_data;
  const int *this_colidx_data;
  int cidx;
  int colNnz;
  int i;
  int k;
  int ridx;
  int s_m;
  int sm;
  int sn;
  int *s_colidx_data;
  int *s_rowidx_data;
  boolean_T found;
  varargin_2_data = varargin_2->data;
  varargin_1_data = varargin_1->data;
  this_colidx_data = this_colidx->data;
  this_d_data = this_d->data;
  sm = varargin_1->size[1];
  sn = varargin_2->size[1];
  s_d->size[0] = 0;
  s_rowidx->size[0] = 0;
  i = s_colidx->size[0];
  s_colidx->size[0] = varargin_2->size[1] + 1;
  emxEnsureCapacity_int32_T(s_colidx, i);
  s_colidx_data = s_colidx->data;
  colNnz = varargin_2->size[1] + 1;
  for (i = 0; i < colNnz; i++) {
    s_colidx_data[i] = 0;
  }

  s_colidx_data[0] = 1;
  colNnz = 1;
  k = 0;
  for (cidx = 0; cidx < sn; cidx++) {
    double nt;
    nt = varargin_2_data[cidx];
    for (ridx = 0; ridx < sm; ridx++) {
      int idx;
      idx = sparse_locBsearch(this_rowidx, (int)varargin_1_data[ridx],
        this_colidx_data[(int)nt - 1], this_colidx_data[(int)nt], &found);
      if (found) {
        double s_d_tmp;
        int i1;
        i = s_d->size[0];
        i1 = s_d->size[0];
        s_d->size[0]++;
        emxEnsureCapacity_real_T(s_d, i1);
        s_d_data = s_d->data;
        s_d_tmp = this_d_data[idx - 1];
        s_d_data[i] = s_d_tmp;
        i = s_rowidx->size[0];
        i1 = s_rowidx->size[0];
        s_rowidx->size[0]++;
        emxEnsureCapacity_int32_T(s_rowidx, i1);
        s_rowidx_data = s_rowidx->data;
        s_rowidx_data[i] = ridx + 1;
        s_d_data[k] = s_d_tmp;
        s_rowidx_data[k] = ridx + 1;
        k++;
        colNnz++;
      }
    }

    s_colidx_data[cidx + 1] = colNnz;
  }

  *s_maxnz = s_colidx_data[s_colidx->size[0] - 1] - 1;
  if (*s_maxnz == 0) {
    i = s_rowidx->size[0];
    s_rowidx->size[0] = 1;
    emxEnsureCapacity_int32_T(s_rowidx, i);
    s_rowidx_data = s_rowidx->data;
    s_rowidx_data[0] = 1;
    i = s_d->size[0];
    s_d->size[0] = 1;
    emxEnsureCapacity_real_T(s_d, i);
    s_d_data = s_d->data;
    s_d_data[0] = 0.0;
  }

  s_m = varargin_1->size[1];
  *s_n = varargin_2->size[1];
  if (*s_maxnz < 1) {
    *s_maxnz = 1;
  }

  return s_m;
}

static void binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
  double in3, const emxArray_real_T *in4)
{
  const double *in2_data;
  const double *in4_data;
  double *in1_data;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  in4_data = in4->data;
  in2_data = in2->data;
  if (in4->size[0] == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = in4->size[0];
  }

  i = in1->size[0];
  in1->size[0] = loop_ub;
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in4->size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = in2_data[i * stride_0_0] + in3 * in4_data[i * stride_1_0];
  }
}

static double c_DampedBFGSwGradientProjection(c_TrajectoryOptimizer_DampedBFG
  *obj, emxArray_real_T *xSol, double *err, double *iter)
{
  b_anonymous_function r;
  d_struct_T varargout_4;
  emxArray_boolean_T *activeSet;
  emxArray_boolean_T *x;
  emxArray_int32_T *activeConstraintIndices;
  emxArray_int32_T *inactiveConstraintIndices;
  emxArray_int32_T *r10;
  emxArray_int32_T *r11;
  emxArray_int32_T *r2;
  emxArray_int32_T *r8;
  emxArray_int32_T *r9;
  emxArray_real_T r13;
  emxArray_real_T *A;
  emxArray_real_T *AIn;
  emxArray_real_T *H;
  emxArray_real_T *Hg;
  emxArray_real_T *P;
  emxArray_real_T *V;
  emxArray_real_T *a;
  emxArray_real_T *a__4;
  emxArray_real_T *alpha;
  emxArray_real_T *bIn;
  emxArray_real_T *b_A;
  emxArray_real_T *b_H;
  emxArray_real_T *b_grad;
  emxArray_real_T *b_s;
  emxArray_real_T *b_x;
  emxArray_real_T *d_A;
  emxArray_real_T *grad;
  emxArray_real_T *gradNew;
  emxArray_real_T *lambdas;
  emxArray_real_T *r1;
  emxArray_real_T *r12;
  emxArray_real_T *r4;
  emxArray_real_T *r5;
  emxArray_real_T *r7;
  emxArray_real_T *sNew;
  emxArray_real_T *xNew;
  double J;
  double cost;
  double costNew;
  double s;
  double varargout_1;
  double *AIn_data;
  double *A_data;
  double *H_data;
  double *Hg_data;
  double *P_data;
  double *V_data;
  double *a_data;
  double *alpha_data;
  double *bIn_data;
  double *b_A_data;
  double *b_grad_data;
  double *b_x_data;
  double *gradNew_data;
  double *grad_data;
  double *lambdas_data;
  double *r3;
  double *sNew_data;
  double *s_data;
  double *xNew_data;
  double *xSol_data;
  int A_idx_0 = 0;
  int b_i = 0;
  int b_loop_ub = 0;
  int c_A = 0;
  int c_i = 0;
  int c_loop_ub = 0;
  int e_A = 0;
  int f_A = 0;
  int i = 0;
  int i1 = 0;
  int iindx = 0;
  int k = 0;
  int loop_ub = 0;
  int m = 0;
  int n = 0;
  unsigned int varargin_1_idx_0 = 0;
  int *activeConstraintIndices_data = NULL;
  int *inactiveConstraintIndices_data = NULL;
  int *r6 = NULL;
  boolean_T guard1 = false;
  boolean_T y = false;
  boolean_T *activeSet_data = NULL;
  boolean_T *x_data = NULL;
  i = xSol->size[0];
  xSol->size[0] = obj->SeedInternal->size[0];
  emxEnsureCapacity_real_T(xSol, i);
  xSol_data = xSol->data;
  loop_ub = obj->SeedInternal->size[0];
  for (i = 0; i < loop_ub; i++) {
    xSol_data[i] = obj->SeedInternal->data[i];
  }

  obj->TimeObjInternal.StartTime.tv_sec = tic
    (&obj->TimeObjInternal.StartTime.tv_nsec);
  n = xSol->size[0];
  r = obj->CostFcn;
  emxInitStruct_struct_T1(&varargout_4);
  emxCopyStruct_struct_T(&varargout_4, &obj->ExtraArgs);
  emxInit_real_T(&a__4, 3);
  J = solver_common_solvePoly(r.workspace.this_, xSol, a__4);
  varargout_1 = J + r.workspace.this_->timeWt * sum(xSol);
  varargout_4.cost = varargout_1;
  cost = varargout_1;
  emxCopyStruct_struct_T(&obj->ExtraArgs, &varargout_4);
  r = obj->GradientFcn;
  emxInit_real_T(&grad, 2);
  solver_common_optimize_anonFcn3(r.workspace.this_, xSol, grad);
  grad_data = grad->data;
  emxInit_real_T(&b_grad, 1);
  i = b_grad->size[0];
  b_grad->size[0] = grad->size[1];
  emxEnsureCapacity_real_T(b_grad, i);
  b_grad_data = b_grad->data;
  loop_ub = grad->size[1];
  for (i = 0; i < loop_ub; i++) {
    b_grad_data[i] = grad_data[i];
  }

  m = xSol->size[0];
  emxInit_real_T(&H, 2);
  i = H->size[0] * H->size[1];
  H->size[1] = xSol->size[0];
  H->size[0] = xSol->size[0];
  emxEnsureCapacity_real_T(H, i);
  H_data = H->data;
  loop_ub = xSol->size[0];
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = xSol->size[0];
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      H_data[i1 + H->size[1] * i] = 0.0;
    }
  }

  if (xSol->size[0] > 0) {
    for (k = 0; k < m; k++) {
      H_data[k + H->size[1] * k] = 1.0;
    }
  }

  emxInit_boolean_T(&activeSet);
  emxInit_real_T(&A, 2);
  A_data = A->data;
  emxInit_real_T(&r1, 2);
  emxInit_int32_T(&r2, 1);
  emxInit_real_T(&b_A, 1);
  if (obj->ConstraintsOn) {
    i = r1->size[0] * r1->size[1];
    r1->size[1] = obj->ConstraintMatrix->size[1];
    r1->size[0] = obj->ConstraintMatrix->size[0];
    emxEnsureCapacity_real_T(r1, i);
    r3 = r1->data;
    loop_ub = obj->ConstraintMatrix->size[0];
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = obj->ConstraintMatrix->size[1];
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r3[i1 + r1->size[1] * i] = obj->ConstraintMatrix->data[i1 +
          obj->ConstraintMatrix->size[1] * i];
      }
    }

    mtimes(r1, xSol, b_A);
    b_A_data = b_A->data;
    if (b_A->size[0] == obj->ConstraintBound->size[0]) {
      i = activeSet->size[0];
      activeSet->size[0] = b_A->size[0];
      emxEnsureCapacity_boolean_T(activeSet, i);
      activeSet_data = activeSet->data;
      loop_ub = b_A->size[0];
      for (i = 0; i < loop_ub; i++) {
        activeSet_data[i] = (b_A_data[i] >= obj->ConstraintBound->data[i]);
      }
    } else {
      i_binary_expand_op(activeSet, b_A, obj);
      activeSet_data = activeSet->data;
    }

    m = activeSet->size[0] - 1;
    iindx = 0;
    for (b_i = 0; b_i <= m; b_i++) {
      if (activeSet_data[b_i]) {
        iindx++;
      }
    }

    i = r2->size[0];
    r2->size[0] = iindx;
    emxEnsureCapacity_int32_T(r2, i);
    r6 = r2->data;
    iindx = 0;
    for (b_i = 0; b_i <= m; b_i++) {
      if (activeSet_data[b_i]) {
        r6[iindx] = b_i;
        iindx++;
      }
    }

    i = A->size[0] * A->size[1];
    A->size[1] = r2->size[0];
    A->size[0] = obj->ConstraintMatrix->size[0];
    emxEnsureCapacity_real_T(A, i);
    A_data = A->data;
    loop_ub = obj->ConstraintMatrix->size[0];
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = r2->size[0];
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        A_data[i1 + A->size[1] * i] = obj->ConstraintMatrix->data[r6[i1] +
          obj->ConstraintMatrix->size[1] * i];
      }
    }
  } else {
    varargin_1_idx_0 = (unsigned int)obj->ConstraintBound->size[0];
    i = activeSet->size[0];
    activeSet->size[0] = (int)varargin_1_idx_0;
    emxEnsureCapacity_boolean_T(activeSet, i);
    activeSet_data = activeSet->data;
    loop_ub = (int)varargin_1_idx_0;
    for (i = 0; i < loop_ub; i++) {
      activeSet_data[i] = false;
    }

    A->size[1] = 0;
    A->size[0] = xSol->size[0];
  }

  i = A->size[1];
  if (A->size[1] - 1 >= 0) {
    c_A = A->size[0];
    c_loop_ub = A->size[0];
    A_idx_0 = A->size[0];
  }

  emxInit_real_T(&r4, 2);
  emxInit_real_T(&r5, 2);
  emxInit_real_T(&a, 2);
  emxInit_real_T(&d_A, 2);
  for (k = 0; k < i; k++) {
    i1 = b_A->size[0];
    b_A->size[0] = c_A;
    emxEnsureCapacity_real_T(b_A, i1);
    b_A_data = b_A->data;
    for (i1 = 0; i1 < c_loop_ub; i1++) {
      b_A_data[i1] = A_data[k + A->size[1] * i1];
    }

    b_mtimes(b_A, H, r4);
    r3 = r4->data;
    s = 0.0;
    loop_ub = A->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      s += A_data[k + A->size[1] * i1] * r3[i1];
    }

    s = 1.0 / s;
    i1 = a->size[0] * a->size[1];
    a->size[1] = H->size[1];
    a->size[0] = H->size[0];
    emxEnsureCapacity_real_T(a, i1);
    a_data = a->data;
    loop_ub = H->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      b_loop_ub = H->size[1];
      for (iindx = 0; iindx < b_loop_ub; iindx++) {
        a_data[iindx + a->size[1] * i1] = s * H_data[iindx + H->size[1] * i1];
      }
    }

    i1 = d_A->size[0] * d_A->size[1];
    d_A->size[1] = A_idx_0;
    d_A->size[0] = A->size[0];
    emxEnsureCapacity_real_T(d_A, i1);
    b_A_data = d_A->data;
    loop_ub = A->size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      for (iindx = 0; iindx < A_idx_0; iindx++) {
        b_A_data[iindx + d_A->size[1] * i1] = A_data[k + A->size[1] * iindx] *
          A_data[k + A->size[1] * i1];
      }
    }

    c_mtimes(a, d_A, r1);
    c_mtimes(r1, H, r5);
    a_data = r5->data;
    if ((H->size[1] == r5->size[1]) && (H->size[0] == r5->size[0])) {
      loop_ub = H->size[0];
      for (i1 = 0; i1 < loop_ub; i1++) {
        b_loop_ub = H->size[1];
        for (iindx = 0; iindx < b_loop_ub; iindx++) {
          H_data[iindx + H->size[1] * i1] -= a_data[iindx + r5->size[1] * i1];
        }
      }
    } else {
      minus(H, r5);
      H_data = H->data;
    }
  }

  emxInit_real_T(&xNew, 1);
  i = xNew->size[0];
  xNew->size[0] = xSol->size[0];
  emxEnsureCapacity_real_T(xNew, i);
  xNew_data = xNew->data;
  loop_ub = xSol->size[0];
  for (i = 0; i < loop_ub; i++) {
    xNew_data[i] = xSol_data[i];
  }

  *iter = 0.0;
  y = rtIsNaN(varargout_1);
  emxInit_real_T(&alpha, 1);
  emxInit_real_T(&Hg, 1);
  emxInit_int32_T(&activeConstraintIndices, 1);
  emxInit_real_T(&P, 2);
  emxInit_real_T(&b_s, 1);
  emxInit_real_T(&bIn, 1);
  emxInit_real_T(&AIn, 2);
  emxInit_int32_T(&inactiveConstraintIndices, 1);
  emxInit_real_T(&lambdas, 1);
  emxInit_real_T(&gradNew, 2);
  emxInit_real_T(&sNew, 1);
  emxInit_real_T(&V, 2);
  emxInit_real_T(&r7, 2);
  emxInit_int32_T(&r8, 1);
  emxInit_boolean_T(&x);
  emxInit_real_T(&b_x, 1);
  emxInit_int32_T(&r9, 1);
  emxInit_int32_T(&r10, 1);
  emxInit_int32_T(&r11, 1);
  emxInit_real_T(&r12, 2);
  emxInit_real_T(&b_H, 2);
  guard1 = false;
  if (y) {
    guard1 = true;
  } else {
    i = x->size[0];
    x->size[0] = grad->size[1];
    emxEnsureCapacity_boolean_T(x, i);
    x_data = x->data;
    loop_ub = grad->size[1];
    for (i = 0; i < loop_ub; i++) {
      x_data[i] = rtIsNaN(grad_data[i]);
    }

    if (b_any(x)) {
      guard1 = true;
    } else {
      double d;
      d = obj->MaxNumIterationInternal;
      b_i = 0;
      int exitg2;
      do {
        exitg2 = 0;
        if (b_i <= (int)d - 1) {
          s = toc(obj->TimeObjInternal.StartTime.tv_sec,
                  obj->TimeObjInternal.StartTime.tv_nsec);
          y = (s > obj->MaxTimeInternal);
          if (y) {
            iindx = 1;
            emxCopyStruct_struct_T(&varargout_4, &obj->ExtraArgs);
            *err = varargout_4.cost;
            *iter = (double)b_i + 1.0;
            exitg2 = 1;
          } else {
            boolean_T b_guard1 = false;
            boolean_T guard2 = false;
            boolean_T guard3 = false;
            boolean_T guard4 = false;
            if ((A->size[0] == 0) || (A->size[1] == 0)) {
              i = alpha->size[0];
              alpha->size[0] = 1;
              emxEnsureCapacity_real_T(alpha, i);
              alpha_data = alpha->data;
              alpha_data[0] = 0.0;
            } else {
              i = d_A->size[0] * d_A->size[1];
              d_A->size[1] = A->size[0];
              d_A->size[0] = A->size[1];
              emxEnsureCapacity_real_T(d_A, i);
              b_A_data = d_A->data;
              loop_ub = A->size[1];
              for (i = 0; i < loop_ub; i++) {
                b_loop_ub = A->size[0];
                for (i1 = 0; i1 < b_loop_ub; i1++) {
                  b_A_data[i1 + d_A->size[1] * i] = A_data[i + A->size[1] * i1];
                }
              }

              d_mtimes(A, A, r1);
              mldivide(r1, d_A, r5);
              e_mtimes(r5, b_grad, alpha);
              alpha_data = alpha->data;
            }

            e_mtimes(H, b_grad, Hg);
            Hg_data = Hg->data;
            s = b_norm(Hg);
            b_guard1 = false;
            guard2 = false;
            guard3 = false;
            guard4 = false;
            if (s < obj->GradientTolerance) {
              boolean_T exitg3;
              i = x->size[0];
              x->size[0] = alpha->size[0];
              emxEnsureCapacity_boolean_T(x, i);
              x_data = x->data;
              loop_ub = alpha->size[0];
              for (i = 0; i < loop_ub; i++) {
                x_data[i] = (alpha_data[i] <= 0.0);
              }

              y = true;
              k = 0;
              exitg3 = false;
              while ((!exitg3) && (k <= x->size[0] - 1)) {
                if (!x_data[k]) {
                  y = false;
                  exitg3 = true;
                } else {
                  k++;
                }
              }

              if (y) {
                iindx = 0;
                emxCopyStruct_struct_T(&varargout_4, &obj->ExtraArgs);
                *err = varargout_4.cost;
                *iter = (double)b_i + 1.0;
                exitg2 = 1;
              } else {
                guard4 = true;
              }
            } else {
              guard4 = true;
            }

            if (guard4) {
              if (obj->ConstraintsOn && ((A->size[0] != 0) && (A->size[1] != 0)))
              {
                double threshold;
                d_mtimes(A, A, r1);
                inv(r1);
                diag(r1, b_x);
                b_x_data = b_x->data;
                i = b_x->size[0];
                for (k = 0; k < i; k++) {
                  b_x_data[k] = sqrt(b_x_data[k]);
                }

                if (alpha->size[0] == b_x->size[0]) {
                  loop_ub = alpha->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    alpha_data[i] /= b_x_data[i];
                  }

                  threshold = maximum(alpha, &iindx);
                } else {
                  threshold = h_binary_expand_op(alpha, b_x, &iindx);
                }

                if (s < 0.5 * threshold) {
                  eml_find(activeSet, r2);
                  r6 = r2->data;
                  c_A = r6[iindx - 1] - 1;
                  activeSet_data[c_A] = false;
                  m = activeSet->size[0] - 1;
                  iindx = 0;
                  for (c_i = 0; c_i <= m; c_i++) {
                    if (activeSet_data[c_i]) {
                      iindx++;
                    }
                  }

                  i = r9->size[0];
                  r9->size[0] = iindx;
                  emxEnsureCapacity_int32_T(r9, i);
                  r6 = r9->data;
                  iindx = 0;
                  for (c_i = 0; c_i <= m; c_i++) {
                    if (activeSet_data[c_i]) {
                      r6[iindx] = c_i;
                      iindx++;
                    }
                  }

                  i = A->size[0] * A->size[1];
                  A->size[1] = r9->size[0];
                  A->size[0] = obj->ConstraintMatrix->size[0];
                  emxEnsureCapacity_real_T(A, i);
                  A_data = A->data;
                  loop_ub = obj->ConstraintMatrix->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    b_loop_ub = r9->size[0];
                    for (i1 = 0; i1 < b_loop_ub; i1++) {
                      A_data[i1 + A->size[1] * i] = obj->ConstraintMatrix->
                        data[r6[i1] + obj->ConstraintMatrix->size[1] * i];
                    }
                  }

                  i = r1->size[0] * r1->size[1];
                  r1->size[1] = n;
                  r1->size[0] = n;
                  emxEnsureCapacity_real_T(r1, i);
                  r3 = r1->data;
                  for (i = 0; i < n; i++) {
                    for (i1 = 0; i1 < n; i1++) {
                      r3[i1 + r1->size[1] * i] = 0.0;
                    }
                  }

                  if (n > 0) {
                    for (k = 0; k < n; k++) {
                      r3[k + r1->size[1] * k] = 1.0;
                    }
                  }

                  i = d_A->size[0] * d_A->size[1];
                  d_A->size[1] = A->size[0];
                  d_A->size[0] = A->size[1];
                  emxEnsureCapacity_real_T(d_A, i);
                  b_A_data = d_A->data;
                  loop_ub = A->size[1];
                  for (i = 0; i < loop_ub; i++) {
                    b_loop_ub = A->size[0];
                    for (i1 = 0; i1 < b_loop_ub; i1++) {
                      b_A_data[i1 + d_A->size[1] * i] = A_data[i + A->size[1] *
                        i1];
                    }
                  }

                  d_mtimes(A, A, r5);
                  mldivide(r5, d_A, b_H);
                  c_mtimes(A, b_H, r5);
                  a_data = r5->data;
                  if ((r1->size[1] == r5->size[1]) && (r1->size[0] == r5->size[0]))
                  {
                    i = P->size[0] * P->size[1];
                    P->size[1] = r1->size[1];
                    P->size[0] = r1->size[0];
                    emxEnsureCapacity_real_T(P, i);
                    P_data = P->data;
                    loop_ub = r1->size[0];
                    for (i = 0; i < loop_ub; i++) {
                      b_loop_ub = r1->size[1];
                      for (i1 = 0; i1 < b_loop_ub; i1++) {
                        P_data[i1 + P->size[1] * i] = r3[i1 + r1->size[1] * i] -
                          a_data[i1 + r5->size[1] * i];
                      }
                    }
                  } else {
                    b_minus(P, r1, r5);
                    P_data = P->data;
                  }

                  i = r7->size[0] * r7->size[1];
                  r7->size[1] = 1;
                  r7->size[0] = obj->ConstraintMatrix->size[0];
                  emxEnsureCapacity_real_T(r7, i);
                  b_A_data = r7->data;
                  loop_ub = obj->ConstraintMatrix->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    b_A_data[i] = obj->ConstraintMatrix->data[c_A +
                      obj->ConstraintMatrix->size[1] * i];
                  }

                  c_A = r7->size[0];
                  r13 = *r7;
                  e_A = c_A;
                  r13.size = &e_A;
                  r13.numDimensions = 1;
                  b_mtimes(&r13, P, r4);
                  r3 = r4->data;
                  s = 0.0;
                  loop_ub = r7->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    s += b_A_data[i] * r3[i];
                  }

                  s = 1.0 / s;
                  i = a->size[0] * a->size[1];
                  a->size[1] = P->size[1];
                  a->size[0] = P->size[0];
                  emxEnsureCapacity_real_T(a, i);
                  a_data = a->data;
                  loop_ub = P->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    b_loop_ub = P->size[1];
                    for (i1 = 0; i1 < b_loop_ub; i1++) {
                      a_data[i1 + a->size[1] * i] = s * P_data[i1 + P->size[1] *
                        i];
                    }
                  }

                  i = r5->size[0] * r5->size[1];
                  r5->size[1] = r7->size[0];
                  r5->size[0] = r7->size[0];
                  emxEnsureCapacity_real_T(r5, i);
                  a_data = r5->data;
                  loop_ub = r7->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    b_loop_ub = r7->size[0];
                    for (i1 = 0; i1 < b_loop_ub; i1++) {
                      a_data[i1 + r5->size[1] * i] = b_A_data[i1] * b_A_data[i];
                    }
                  }

                  c_mtimes(a, r5, r1);
                  c_mtimes(r1, P, r5);
                  a_data = r5->data;
                  if ((H->size[1] == r5->size[1]) && (H->size[0] == r5->size[0]))
                  {
                    loop_ub = H->size[0];
                    for (i = 0; i < loop_ub; i++) {
                      b_loop_ub = H->size[1];
                      for (i1 = 0; i1 < b_loop_ub; i1++) {
                        H_data[i1 + H->size[1] * i] += a_data[i1 + r5->size[1] *
                          i];
                      }
                    }
                  } else {
                    plus(H, r5);
                    H_data = H->data;
                  }

                  b_i++;
                } else {
                  guard3 = true;
                }
              } else {
                guard3 = true;
              }
            }

            if (guard3) {
              double b_gamma;
              double b_m;
              double beta;
              double c_x;
              double lambda;
              double sigma;
              int exitg1;
              int idxl;
              i = b_s->size[0];
              b_s->size[0] = Hg->size[0];
              emxEnsureCapacity_real_T(b_s, i);
              s_data = b_s->data;
              loop_ub = Hg->size[0];
              for (i = 0; i < loop_ub; i++) {
                s_data[i] = -Hg_data[i];
              }

              idxl = -2;
              if (obj->ConstraintsOn) {
                i = x->size[0];
                x->size[0] = activeSet->size[0];
                emxEnsureCapacity_boolean_T(x, i);
                x_data = x->data;
                loop_ub = activeSet->size[0];
                for (i = 0; i < loop_ub; i++) {
                  x_data[i] = !activeSet_data[i];
                }

                if (b_any(x)) {
                  m = x->size[0] - 1;
                  iindx = 0;
                  for (c_i = 0; c_i <= m; c_i++) {
                    if (x_data[c_i]) {
                      iindx++;
                    }
                  }

                  i = r8->size[0];
                  r8->size[0] = iindx;
                  emxEnsureCapacity_int32_T(r8, i);
                  r6 = r8->data;
                  iindx = 0;
                  for (c_i = 0; c_i <= m; c_i++) {
                    if (x_data[c_i]) {
                      r6[iindx] = c_i;
                      iindx++;
                    }
                  }

                  i = bIn->size[0];
                  bIn->size[0] = r8->size[0];
                  emxEnsureCapacity_real_T(bIn, i);
                  bIn_data = bIn->data;
                  loop_ub = r8->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    bIn_data[i] = obj->ConstraintBound->data[r6[i]];
                  }

                  i = AIn->size[0] * AIn->size[1];
                  AIn->size[1] = r8->size[0];
                  AIn->size[0] = obj->ConstraintMatrix->size[0];
                  emxEnsureCapacity_real_T(AIn, i);
                  AIn_data = AIn->data;
                  loop_ub = obj->ConstraintMatrix->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    b_loop_ub = r8->size[0];
                    for (i1 = 0; i1 < b_loop_ub; i1++) {
                      AIn_data[i1 + AIn->size[1] * i] = obj->
                        ConstraintMatrix->data[r6[i1] + obj->
                        ConstraintMatrix->size[1] * i];
                    }
                  }

                  eml_find(x, activeConstraintIndices);
                  activeConstraintIndices_data = activeConstraintIndices->data;
                  i = inactiveConstraintIndices->size[0];
                  inactiveConstraintIndices->size[0] =
                    activeConstraintIndices->size[0];
                  emxEnsureCapacity_int32_T(inactiveConstraintIndices, i);
                  inactiveConstraintIndices_data =
                    inactiveConstraintIndices->data;
                  loop_ub = activeConstraintIndices->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    inactiveConstraintIndices_data[i] =
                      activeConstraintIndices_data[i];
                  }

                  mtimes(AIn, xSol, b_A);
                  b_A_data = b_A->data;
                  mtimes(AIn, b_s, b_x);
                  b_x_data = b_x->data;
                  if (bIn->size[0] == 1) {
                    i = b_A->size[0];
                  } else {
                    i = bIn->size[0];
                  }

                  if ((bIn->size[0] == b_A->size[0]) && (i == b_x->size[0])) {
                    i = lambdas->size[0];
                    lambdas->size[0] = bIn->size[0];
                    emxEnsureCapacity_real_T(lambdas, i);
                    lambdas_data = lambdas->data;
                    loop_ub = bIn->size[0];
                    for (i = 0; i < loop_ub; i++) {
                      lambdas_data[i] = (bIn_data[i] - b_A_data[i]) / b_x_data[i];
                    }
                  } else {
                    g_binary_expand_op(lambdas, bIn, b_A, b_x);
                    lambdas_data = lambdas->data;
                  }

                  i = x->size[0];
                  x->size[0] = lambdas->size[0];
                  emxEnsureCapacity_boolean_T(x, i);
                  x_data = x->data;
                  loop_ub = lambdas->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    x_data[i] = (lambdas_data[i] > 0.0);
                  }

                  eml_find(x, activeConstraintIndices);
                  activeConstraintIndices_data = activeConstraintIndices->data;
                  if (activeConstraintIndices->size[0] != 0) {
                    m = x->size[0] - 1;
                    iindx = 0;
                    for (c_i = 0; c_i <= m; c_i++) {
                      if (x_data[c_i]) {
                        iindx++;
                      }
                    }

                    i = r10->size[0];
                    r10->size[0] = iindx;
                    emxEnsureCapacity_int32_T(r10, i);
                    r6 = r10->data;
                    iindx = 0;
                    for (c_i = 0; c_i <= m; c_i++) {
                      if (x_data[c_i]) {
                        r6[iindx] = c_i;
                        iindx++;
                      }
                    }

                    i = b_A->size[0];
                    b_A->size[0] = r10->size[0];
                    emxEnsureCapacity_real_T(b_A, i);
                    b_A_data = b_A->data;
                    loop_ub = r10->size[0];
                    for (i = 0; i < loop_ub; i++) {
                      b_A_data[i] = lambdas_data[r6[i]];
                    }

                    lambda = minimum(b_A, &iindx);
                    idxl =
                      inactiveConstraintIndices_data[activeConstraintIndices_data
                      [iindx - 1] - 1] - 1;
                  } else {
                    lambda = 0.0;
                  }
                } else {
                  lambda = 0.0;
                }
              } else {
                lambda = 0.0;
              }

              if (lambda > 0.0) {
                b_gamma = fmin(1.0, lambda);
              } else {
                b_gamma = 1.0;
              }

              beta = obj->ArmijoRuleBeta;
              sigma = obj->ArmijoRuleSigma;
              r = obj->CostFcn;
              if (xSol->size[0] == b_s->size[0]) {
                i = b_x->size[0];
                b_x->size[0] = xSol->size[0];
                emxEnsureCapacity_real_T(b_x, i);
                b_x_data = b_x->data;
                loop_ub = xSol->size[0];
                for (i = 0; i < loop_ub; i++) {
                  b_x_data[i] = xSol_data[i] + b_gamma * s_data[i];
                }
              } else {
                binary_expand_op(b_x, xSol, b_gamma, b_s);
              }

              emxCopyStruct_struct_T(&varargout_4, &obj->ExtraArgs);
              J = solver_common_solvePoly(r.workspace.this_, b_x, a__4);
              varargout_1 = J + r.workspace.this_->timeWt * sum(b_x);
              varargout_4.cost = varargout_1;
              costNew = varargout_1;
              emxCopyStruct_struct_T(&obj->ExtraArgs, &varargout_4);
              b_m = 0.0;
              do {
                exitg1 = 0;
                i = b_x->size[0];
                b_x->size[0] = b_s->size[0];
                emxEnsureCapacity_real_T(b_x, i);
                b_x_data = b_x->data;
                loop_ub = b_s->size[0];
                for (i = 0; i < loop_ub; i++) {
                  b_x_data[i] = b_gamma * s_data[i];
                }

                c_x = 0.0;
                loop_ub = b_x->size[0];
                for (i = 0; i < loop_ub; i++) {
                  c_x += b_x_data[i] * (-sigma * b_grad_data[i]);
                }

                if (cost - costNew < c_x) {
                  y = (b_gamma < obj->StepTolerance);
                  if (y) {
                    iindx = 2;
                    emxCopyStruct_struct_T(&varargout_4, &obj->ExtraArgs);
                    *err = varargout_4.cost;
                    *iter = (double)b_i + 1.0;
                    exitg1 = 1;
                  } else {
                    b_gamma *= beta;
                    b_m++;
                    r = obj->CostFcn;
                    if (xSol->size[0] == b_s->size[0]) {
                      i = b_x->size[0];
                      b_x->size[0] = xSol->size[0];
                      emxEnsureCapacity_real_T(b_x, i);
                      b_x_data = b_x->data;
                      loop_ub = xSol->size[0];
                      for (i = 0; i < loop_ub; i++) {
                        b_x_data[i] = xSol_data[i] + b_gamma * s_data[i];
                      }
                    } else {
                      binary_expand_op(b_x, xSol, b_gamma, b_s);
                    }

                    emxCopyStruct_struct_T(&varargout_4, &obj->ExtraArgs);
                    J = solver_common_solvePoly(r.workspace.this_, b_x, a__4);
                    varargout_1 = J + r.workspace.this_->timeWt * sum(b_x);
                    varargout_4.cost = varargout_1;
                    costNew = varargout_1;
                    emxCopyStruct_struct_T(&obj->ExtraArgs, &varargout_4);
                  }
                } else {
                  if (xSol->size[0] == b_s->size[0]) {
                    i = xNew->size[0];
                    xNew->size[0] = xSol->size[0];
                    emxEnsureCapacity_real_T(xNew, i);
                    xNew_data = xNew->data;
                    loop_ub = xSol->size[0];
                    for (i = 0; i < loop_ub; i++) {
                      xNew_data[i] = xSol_data[i] + b_gamma * s_data[i];
                    }
                  } else {
                    binary_expand_op(xNew, xSol, b_gamma, b_s);
                    xNew_data = xNew->data;
                  }

                  r = obj->GradientFcn;
                  solver_common_optimize_anonFcn3(r.workspace.this_, xNew,
                    gradNew);
                  gradNew_data = gradNew->data;
                  exitg1 = 2;
                }
              } while (exitg1 == 0);

              if (exitg1 == 1) {
                exitg2 = 1;
              } else if ((fabs(b_m) < DBL_EPSILON) && (fabs(b_gamma - lambda) <
                          1.4901161193847656E-8)) {
                i = r7->size[0] * r7->size[1];
                r7->size[1] = 1;
                r7->size[0] = obj->ConstraintMatrix->size[0];
                emxEnsureCapacity_real_T(r7, i);
                b_A_data = r7->data;
                loop_ub = obj->ConstraintMatrix->size[0];
                for (i = 0; i < loop_ub; i++) {
                  b_A_data[i] = obj->ConstraintMatrix->data[idxl +
                    obj->ConstraintMatrix->size[1] * i];
                }

                activeSet_data[idxl] = true;
                m = activeSet->size[0] - 1;
                iindx = 0;
                for (c_i = 0; c_i <= m; c_i++) {
                  if (activeSet_data[c_i]) {
                    iindx++;
                  }
                }

                i = r11->size[0];
                r11->size[0] = iindx;
                emxEnsureCapacity_int32_T(r11, i);
                r6 = r11->data;
                iindx = 0;
                for (c_i = 0; c_i <= m; c_i++) {
                  if (activeSet_data[c_i]) {
                    r6[iindx] = c_i;
                    iindx++;
                  }
                }

                i = A->size[0] * A->size[1];
                A->size[1] = r11->size[0];
                A->size[0] = obj->ConstraintMatrix->size[0];
                emxEnsureCapacity_real_T(A, i);
                A_data = A->data;
                loop_ub = obj->ConstraintMatrix->size[0];
                for (i = 0; i < loop_ub; i++) {
                  b_loop_ub = r11->size[0];
                  for (i1 = 0; i1 < b_loop_ub; i1++) {
                    A_data[i1 + A->size[1] * i] = obj->ConstraintMatrix->
                      data[r6[i1] + obj->ConstraintMatrix->size[1] * i];
                  }
                }

                c_A = r7->size[0];
                r13 = *r7;
                f_A = c_A;
                r13.size = &f_A;
                r13.numDimensions = 1;
                b_mtimes(&r13, H, r4);
                r3 = r4->data;
                s = 0.0;
                loop_ub = r7->size[0];
                i = r5->size[0] * r5->size[1];
                r5->size[1] = r7->size[0];
                r5->size[0] = r7->size[0];
                emxEnsureCapacity_real_T(r5, i);
                a_data = r5->data;
                for (i = 0; i < loop_ub; i++) {
                  s += b_A_data[i] * r3[i];
                  b_loop_ub = r7->size[0];
                  for (i1 = 0; i1 < b_loop_ub; i1++) {
                    a_data[i1 + r5->size[1] * i] = b_A_data[i1] * b_A_data[i];
                  }
                }

                s = 1.0 / s;
                c_mtimes(r5, H, r1);
                c_mtimes(H, r1, r5);
                a_data = r5->data;
                if ((H->size[1] == r5->size[1]) && (H->size[0] == r5->size[0]))
                {
                  loop_ub = H->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    b_loop_ub = H->size[1];
                    for (i1 = 0; i1 < b_loop_ub; i1++) {
                      H_data[i1 + H->size[1] * i] -= s * a_data[i1 + r5->size[1]
                        * i];
                    }
                  }
                } else {
                  b_binary_expand_op(H, s, r5);
                  H_data = H->data;
                }

                guard2 = true;
              } else {
                double theta;
                if (gradNew->size[1] == b_grad->size[0]) {
                  i = b_x->size[0];
                  b_x->size[0] = gradNew->size[1];
                  emxEnsureCapacity_real_T(b_x, i);
                  b_x_data = b_x->data;
                  loop_ub = gradNew->size[1];
                  for (i = 0; i < loop_ub; i++) {
                    b_x_data[i] = gradNew_data[i] - b_grad_data[i];
                  }
                } else {
                  f_binary_expand_op(b_x, gradNew, b_grad);
                  b_x_data = b_x->data;
                }

                i = r12->size[0] * r12->size[1];
                r12->size[1] = b_x->size[0];
                r12->size[0] = 1;
                emxEnsureCapacity_real_T(r12, i);
                a_data = r12->data;
                loop_ub = b_x->size[0];
                c_x = 0.0;
                for (i = 0; i < loop_ub; i++) {
                  a_data[i] = 0.2 * b_x_data[i];
                  c_x += b_x_data[i] * s_data[i];
                }

                f_mtimes(r12, H, r4);
                r3 = r4->data;
                s = 0.0;
                loop_ub = b_x->size[0];
                for (i = 0; i < loop_ub; i++) {
                  s += b_x_data[i] * r3[i];
                }

                if (c_x < s) {
                  i = r12->size[0] * r12->size[1];
                  r12->size[1] = b_x->size[0];
                  r12->size[0] = 1;
                  emxEnsureCapacity_real_T(r12, i);
                  a_data = r12->data;
                  loop_ub = b_x->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    a_data[i] = 0.8 * b_x_data[i];
                  }

                  f_mtimes(r12, H, r4);
                  r3 = r4->data;
                  b_mtimes(b_x, H, r12);
                  a_data = r12->data;
                  c_x = 0.0;
                  loop_ub = b_x->size[0];
                  s = 0.0;
                  varargout_1 = 0.0;
                  for (i = 0; i < loop_ub; i++) {
                    c_x += b_x_data[i] * r3[i];
                    s += b_x_data[i] * a_data[i];
                    varargout_1 += b_x_data[i] * s_data[i];
                  }

                  theta = c_x / (s - varargout_1);
                } else {
                  theta = 1.0;
                }

                i = r1->size[0] * r1->size[1];
                r1->size[1] = H->size[1];
                r1->size[0] = H->size[0];
                emxEnsureCapacity_real_T(r1, i);
                r3 = r1->data;
                loop_ub = H->size[0];
                for (i = 0; i < loop_ub; i++) {
                  b_loop_ub = H->size[1];
                  for (i1 = 0; i1 < b_loop_ub; i1++) {
                    r3[i1 + r1->size[1] * i] = (1.0 - theta) * H_data[i1 +
                      H->size[1] * i];
                  }
                }

                e_mtimes(r1, b_x, b_A);
                b_A_data = b_A->data;
                if (b_s->size[0] == b_A->size[0]) {
                  i = sNew->size[0];
                  sNew->size[0] = b_s->size[0];
                  emxEnsureCapacity_real_T(sNew, i);
                  sNew_data = sNew->data;
                  loop_ub = b_s->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    sNew_data[i] = theta * s_data[i] + b_A_data[i];
                  }
                } else {
                  e_binary_expand_op(sNew, theta, b_s, b_A);
                  sNew_data = sNew->data;
                }

                c_x = 0.0;
                loop_ub = b_x->size[0];
                for (i = 0; i < loop_ub; i++) {
                  c_x += b_x_data[i] * sNew_data[i];
                }

                i = r1->size[0] * r1->size[1];
                r1->size[1] = n;
                r1->size[0] = n;
                emxEnsureCapacity_real_T(r1, i);
                r3 = r1->data;
                for (i = 0; i < n; i++) {
                  for (i1 = 0; i1 < n; i1++) {
                    r3[i1 + r1->size[1] * i] = 0.0;
                  }
                }

                if (n > 0) {
                  for (k = 0; k < n; k++) {
                    r3[k + r1->size[1] * k] = 1.0;
                  }
                }

                if ((r1->size[1] == b_x->size[0]) && (sNew->size[0] == r1->size
                     [0])) {
                  i = V->size[0] * V->size[1];
                  V->size[1] = b_x->size[0];
                  V->size[0] = sNew->size[0];
                  emxEnsureCapacity_real_T(V, i);
                  V_data = V->data;
                  loop_ub = sNew->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    b_loop_ub = b_x->size[0];
                    for (i1 = 0; i1 < b_loop_ub; i1++) {
                      V_data[i1 + V->size[1] * i] = r3[i1 + r1->size[1] * i] -
                        b_x_data[i1] * sNew_data[i] / c_x;
                    }
                  }
                } else {
                  d_binary_expand_op(V, r1, b_x, sNew, c_x);
                }

                c_mtimes(V, H, r1);
                g_mtimes(r1, V, r5);
                a_data = r5->data;
                if ((r5->size[1] == sNew->size[0]) && (sNew->size[0] == r5->
                     size[0])) {
                  i = H->size[0] * H->size[1];
                  H->size[1] = sNew->size[0];
                  H->size[0] = sNew->size[0];
                  emxEnsureCapacity_real_T(H, i);
                  H_data = H->data;
                  loop_ub = sNew->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    b_loop_ub = sNew->size[0];
                    for (i1 = 0; i1 < b_loop_ub; i1++) {
                      H_data[i1 + H->size[1] * i] = a_data[i1 + r5->size[1] * i]
                        + sNew_data[i1] * sNew_data[i] / c_x;
                    }
                  }
                } else {
                  c_binary_expand_op(H, r5, sNew, c_x);
                  H_data = H->data;
                }

                varargin_1_idx_0 = (unsigned int)H->size[1];
                iindx = H->size[0];
                m = H->size[1];
                if (iindx <= m) {
                  m = iindx;
                }

                i = a->size[0] * a->size[1];
                a->size[1] = H->size[1];
                a->size[0] = H->size[0];
                emxEnsureCapacity_real_T(a, i);
                a_data = a->data;
                loop_ub = H->size[0];
                for (i = 0; i < loop_ub; i++) {
                  b_loop_ub = (int)varargin_1_idx_0;
                  for (i1 = 0; i1 < b_loop_ub; i1++) {
                    a_data[i1 + a->size[1] * i] = 0.0;
                  }
                }

                if (m > 0) {
                  for (k = 0; k < m; k++) {
                    a_data[k + a->size[1] * k] = 1.0;
                  }
                }

                if (a->size[1] == 1) {
                  loop_ub = H->size[1];
                } else {
                  loop_ub = a->size[1];
                }

                i = b_H->size[0] * b_H->size[1];
                b_H->size[1] = loop_ub;
                if (a->size[0] == 1) {
                  b_loop_ub = H->size[0];
                } else {
                  b_loop_ub = a->size[0];
                }

                b_H->size[0] = b_loop_ub;
                emxEnsureCapacity_real_T(b_H, i);
                b_A_data = b_H->data;
                iindx = (H->size[1] != 1);
                m = (H->size[0] != 1);
                c_A = (a->size[1] != 1);
                c_loop_ub = (a->size[0] != 1);
                A_idx_0 = 0;
                c_i = 0;
                for (i = 0; i < b_loop_ub; i++) {
                  for (i1 = 0; i1 < loop_ub; i1++) {
                    b_A_data[i1 + b_H->size[1] * i] = H_data[i1 * iindx +
                      H->size[1] * A_idx_0] + 1.4901161193847656E-8 * a_data[i1 *
                      c_A + a->size[1] * c_i];
                  }

                  c_i += c_loop_ub;
                  A_idx_0 += m;
                }

                i = a->size[0] * a->size[1];
                a->size[1] = b_H->size[1];
                a->size[0] = b_H->size[0];
                emxEnsureCapacity_real_T(a, i);
                a_data = a->data;
                loop_ub = b_H->size[0];
                for (i = 0; i < loop_ub; i++) {
                  b_loop_ub = b_H->size[1];
                  for (i1 = 0; i1 < b_loop_ub; i1++) {
                    a_data[i1 + a->size[1] * i] = b_A_data[i1 + b_H->size[1] * i];
                  }
                }

                if (!isPositiveDefinite(a)) {
                  i = xSol->size[0];
                  xSol->size[0] = xNew->size[0];
                  emxEnsureCapacity_real_T(xSol, i);
                  xSol_data = xSol->data;
                  loop_ub = xNew->size[0];
                  for (i = 0; i < loop_ub; i++) {
                    xSol_data[i] = xNew_data[i];
                  }

                  iindx = 3;
                  emxCopyStruct_struct_T(&varargout_4, &obj->ExtraArgs);
                  *err = varargout_4.cost;
                  *iter = (double)b_i + 1.0;
                  exitg2 = 1;
                } else {
                  guard2 = true;
                }
              }
            }

            if (guard2) {
              if (obj->ConstraintsOn) {
                m = obj->ConstraintMatrix->size[1];
                iindx = obj->ConstraintMatrix->size[0];
                i = b_A->size[0];
                b_A->size[0] = obj->ConstraintMatrix->size[1];
                emxEnsureCapacity_real_T(b_A, i);
                b_A_data = b_A->data;
                for (c_i = 0; c_i < m; c_i++) {
                  s = 0.0;
                  for (k = 0; k < iindx; k++) {
                    s += obj->ConstraintMatrix->data[c_i + obj->
                      ConstraintMatrix->size[1] * k] * xNew_data[k];
                  }

                  b_A_data[c_i] = s;
                }

                i = x->size[0];
                if (obj->ConstraintBound->size[0] == 1) {
                  x->size[0] = b_A->size[0];
                } else {
                  x->size[0] = obj->ConstraintBound->size[0];
                }

                emxEnsureCapacity_boolean_T(x, i);
                x_data = x->data;
                iindx = (b_A->size[0] != 1);
                c_A = (obj->ConstraintBound->size[0] != 1);
                if (obj->ConstraintBound->size[0] == 1) {
                  loop_ub = b_A->size[0];
                } else {
                  loop_ub = obj->ConstraintBound->size[0];
                }

                for (i = 0; i < loop_ub; i++) {
                  x_data[i] = (b_A_data[i * iindx] - obj->ConstraintBound->
                               data[i * c_A] > 1.4901161193847656E-8);
                }

                if (b_any(x)) {
                  iindx = 4;
                  emxCopyStruct_struct_T(&varargout_4, &obj->ExtraArgs);
                  *err = varargout_4.cost;
                  *iter = (double)b_i + 1.0;
                  exitg2 = 1;
                } else {
                  b_guard1 = true;
                }
              } else {
                b_guard1 = true;
              }
            }

            if (b_guard1) {
              i = xSol->size[0];
              xSol->size[0] = xNew->size[0];
              emxEnsureCapacity_real_T(xSol, i);
              xSol_data = xSol->data;
              loop_ub = xNew->size[0];
              for (i = 0; i < loop_ub; i++) {
                xSol_data[i] = xNew_data[i];
              }

              i = b_grad->size[0];
              b_grad->size[0] = gradNew->size[1];
              emxEnsureCapacity_real_T(b_grad, i);
              b_grad_data = b_grad->data;
              loop_ub = gradNew->size[1];
              for (i = 0; i < loop_ub; i++) {
                b_grad_data[i] = gradNew_data[i];
              }

              cost = costNew;
              b_i++;
            }
          }
        } else {
          iindx = 5;
          emxCopyStruct_struct_T(&varargout_4, &obj->ExtraArgs);
          *err = varargout_4.cost;
          *iter = obj->MaxNumIterationInternal;
          exitg2 = 1;
        }
      } while (exitg2 == 0);
    }
  }

  if (guard1) {
    printf("Objective should not be NaN\n");
    fflush(stdout);
    iindx = 3;
    *err = 1.7976931348623157E+308;
  }

  emxFree_real_T(&b_H);
  emxFree_real_T(&r12);
  emxFree_real_T(&d_A);
  emxFree_real_T(&a);
  emxFree_real_T(&b_A);
  emxFree_int32_T(&r11);
  emxFree_int32_T(&r10);
  emxFree_int32_T(&r9);
  emxFree_int32_T(&r2);
  emxFree_real_T(&r5);
  emxFree_real_T(&r4);
  emxFree_real_T(&b_x);
  emxFree_boolean_T(&x);
  emxFree_real_T(&a__4);
  emxFreeStruct_struct_T1(&varargout_4);
  emxFree_int32_T(&r8);
  emxFree_real_T(&r7);
  emxFree_real_T(&r1);
  emxFree_real_T(&b_grad);
  emxFree_real_T(&V);
  emxFree_real_T(&sNew);
  emxFree_real_T(&gradNew);
  emxFree_real_T(&lambdas);
  emxFree_int32_T(&inactiveConstraintIndices);
  emxFree_real_T(&AIn);
  emxFree_real_T(&bIn);
  emxFree_real_T(&b_s);
  emxFree_real_T(&P);
  emxFree_int32_T(&activeConstraintIndices);
  emxFree_real_T(&Hg);
  emxFree_real_T(&alpha);
  emxFree_real_T(&xNew);
  emxFree_real_T(&A);
  emxFree_boolean_T(&activeSet);
  emxFree_real_T(&H);
  emxFree_real_T(&grad);
  return iindx;
}

static void c_binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
  const emxArray_real_T *in3, double in4)
{
  emxArray_real_T *b_in2;
  const double *in2_data;
  const double *in3_data;
  double *b_in2_data;
  double *in1_data;
  int aux_0_1;
  int aux_1_1;
  int i;
  int i1;
  int in3_idx_0;
  int stride_0_0;
  int stride_0_1;
  int stride_1_0_tmp;
  int unnamed_idx_1;
  in3_data = in3->data;
  in2_data = in2->data;
  in3_idx_0 = in3->size[0];
  unnamed_idx_1 = in3->size[0];
  i = in1->size[0] * in1->size[1];
  in1->size[1] = in3_idx_0;
  in1->size[0] = unnamed_idx_1;
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  for (i = 0; i < unnamed_idx_1; i++) {
    for (i1 = 0; i1 < in3_idx_0; i1++) {
      in1_data[i1 + in1->size[1] * i] = in3_data[i1] * in3_data[i] / in4;
    }
  }

  emxInit_real_T(&b_in2, 2);
  if (in1->size[1] == 1) {
    in3_idx_0 = in2->size[1];
  } else {
    in3_idx_0 = in1->size[1];
  }

  i = b_in2->size[0] * b_in2->size[1];
  b_in2->size[1] = in3_idx_0;
  if (in1->size[0] == 1) {
    unnamed_idx_1 = in2->size[0];
  } else {
    unnamed_idx_1 = in1->size[0];
  }

  b_in2->size[0] = unnamed_idx_1;
  emxEnsureCapacity_real_T(b_in2, i);
  b_in2_data = b_in2->data;
  stride_0_0 = (in2->size[1] != 1);
  stride_0_1 = (in2->size[0] != 1);
  stride_1_0_tmp = (in1->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < unnamed_idx_1; i++) {
    for (i1 = 0; i1 < in3_idx_0; i1++) {
      b_in2_data[i1 + b_in2->size[1] * i] = in2_data[i1 * stride_0_0 + in2->
        size[1] * aux_0_1] + in1_data[i1 * stride_1_0_tmp + in1->size[1] *
        aux_1_1];
    }

    aux_1_1 += stride_1_0_tmp;
    aux_0_1 += stride_0_1;
  }

  i = in1->size[0] * in1->size[1];
  in1->size[1] = b_in2->size[1];
  in1->size[0] = b_in2->size[0];
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  in3_idx_0 = b_in2->size[0];
  for (i = 0; i < in3_idx_0; i++) {
    unnamed_idx_1 = b_in2->size[1];
    for (i1 = 0; i1 < unnamed_idx_1; i1++) {
      in1_data[i1 + in1->size[1] * i] = b_in2_data[i1 + b_in2->size[1] * i];
    }
  }

  emxFree_real_T(&b_in2);
}

static void c_heapify(emxArray_int32_T *x, int idx, int xstart, int xend, const
                      emxArray_int32_T *cmp_workspace_a, const emxArray_int32_T *
                      cmp_workspace_b)
{
  const int *cmp_workspace_a_data;
  const int *cmp_workspace_b_data;
  int extremum;
  int extremumIdx;
  int i;
  int i1;
  int leftIdx;
  int *x_data;
  boolean_T changed;
  boolean_T exitg1;
  boolean_T varargout_1;
  cmp_workspace_b_data = cmp_workspace_b->data;
  cmp_workspace_a_data = cmp_workspace_a->data;
  x_data = x->data;
  changed = true;
  extremumIdx = (idx + xstart) - 2;
  leftIdx = ((idx << 1) + xstart) - 2;
  exitg1 = false;
  while ((!exitg1) && (leftIdx + 1 < xend)) {
    int cmpIdx;
    int i2;
    int xcmp;
    changed = false;
    extremum = x_data[extremumIdx];
    cmpIdx = leftIdx;
    xcmp = x_data[leftIdx];
    i = cmp_workspace_a_data[x_data[leftIdx] - 1];
    i1 = x_data[leftIdx + 1] - 1;
    i2 = cmp_workspace_a_data[i1];
    if (i < i2) {
      varargout_1 = true;
    } else if (i == i2) {
      varargout_1 = (cmp_workspace_b_data[x_data[leftIdx] - 1] <
                     cmp_workspace_b_data[i1]);
    } else {
      varargout_1 = false;
    }

    if (varargout_1) {
      cmpIdx = leftIdx + 1;
      xcmp = x_data[leftIdx + 1];
    }

    i = cmp_workspace_a_data[x_data[extremumIdx] - 1];
    i1 = cmp_workspace_a_data[xcmp - 1];
    if (i < i1) {
      varargout_1 = true;
    } else if (i == i1) {
      varargout_1 = (cmp_workspace_b_data[x_data[extremumIdx] - 1] <
                     cmp_workspace_b_data[xcmp - 1]);
    } else {
      varargout_1 = false;
    }

    if (varargout_1) {
      x_data[extremumIdx] = xcmp;
      x_data[cmpIdx] = extremum;
      extremumIdx = cmpIdx;
      leftIdx = ((((cmpIdx - xstart) + 2) << 1) + xstart) - 2;
      changed = true;
    } else {
      exitg1 = true;
    }
  }

  if (changed && (leftIdx + 1 <= xend)) {
    extremum = x_data[extremumIdx];
    i = cmp_workspace_a_data[x_data[extremumIdx] - 1];
    i1 = cmp_workspace_a_data[x_data[leftIdx] - 1];
    if (i < i1) {
      varargout_1 = true;
    } else if (i == i1) {
      varargout_1 = (cmp_workspace_b_data[x_data[extremumIdx] - 1] <
                     cmp_workspace_b_data[x_data[leftIdx] - 1]);
    } else {
      varargout_1 = false;
    }

    if (varargout_1) {
      x_data[extremumIdx] = x_data[leftIdx];
      x_data[leftIdx] = extremum;
    }
  }
}

static void c_heapsort(int x[100], int xstart, int xend)
{
  int idx;
  int k;
  int n;
  n = (xend - xstart) - 1;
  for (idx = n + 2; idx >= 1; idx--) {
    b_heapify(x, idx, xstart, xend);
  }

  for (k = 0; k <= n; k++) {
    int t;
    idx = (xend - k) - 1;
    t = x[idx];
    x[idx] = x[xstart - 1];
    x[xstart - 1] = t;
    b_heapify(x, 1, xstart, idx);
  }
}

static void c_insertionsort(emxArray_int32_T *x, int xstart, int xend, const
  emxArray_int32_T *cmp_workspace_a, const emxArray_int32_T *cmp_workspace_b)
{
  const int *cmp_workspace_a_data;
  const int *cmp_workspace_b_data;
  int i;
  int k;
  int *x_data;
  cmp_workspace_b_data = cmp_workspace_b->data;
  cmp_workspace_a_data = cmp_workspace_a->data;
  x_data = x->data;
  i = xstart + 1;
  for (k = i; k <= xend; k++) {
    int idx;
    int xc;
    boolean_T exitg1;
    xc = x_data[k - 1] - 1;
    idx = k - 2;
    exitg1 = false;
    while ((!exitg1) && (idx + 1 >= xstart)) {
      int i1;
      boolean_T varargout_1;
      i1 = cmp_workspace_a_data[x_data[idx] - 1];
      if (cmp_workspace_a_data[xc] < i1) {
        varargout_1 = true;
      } else if (cmp_workspace_a_data[xc] == i1) {
        varargout_1 = (cmp_workspace_b_data[xc] <
                       cmp_workspace_b_data[x_data[idx] - 1]);
      } else {
        varargout_1 = false;
      }

      if (varargout_1) {
        x_data[idx + 1] = x_data[idx];
        idx--;
      } else {
        exitg1 = true;
      }
    }

    x_data[idx + 1] = xc + 1;
  }
}

static void c_introsort(emxArray_int32_T *x, int xend, const emxArray_int32_T
  *cmp_workspace_a, const emxArray_int32_T *cmp_workspace_b)
{
  struct_T frame;
  const int *cmp_workspace_a_data;
  const int *cmp_workspace_b_data;
  int i;
  int *x_data;
  cmp_workspace_b_data = cmp_workspace_b->data;
  cmp_workspace_a_data = cmp_workspace_a->data;
  x_data = x->data;
  if (xend > 1) {
    if (xend <= 32) {
      c_insertionsort(x, 1, xend, cmp_workspace_a, cmp_workspace_b);
    } else {
      struct_T st_d_data[120];
      int MAXDEPTH;
      int pmax;
      int pmin;
      int pow2p;
      int st_n;
      int t;
      boolean_T exitg1;
      pmax = 31;
      pmin = 0;
      exitg1 = false;
      while ((!exitg1) && (pmax - pmin > 1)) {
        t = (pmin + pmax) >> 1;
        pow2p = 1 << t;
        if (pow2p == xend) {
          pmax = t;
          exitg1 = true;
        } else if (pow2p > xend) {
          pmax = t;
        } else {
          pmin = t;
        }
      }

      MAXDEPTH = (pmax - 1) << 1;
      frame.xstart = 1;
      frame.xend = xend;
      frame.depth = 0;
      pmax = MAXDEPTH << 1;
      for (i = 0; i < pmax; i++) {
        st_d_data[i] = frame;
      }

      st_d_data[0] = frame;
      st_n = 1;
      while (st_n > 0) {
        frame = st_d_data[st_n - 1];
        st_n--;
        i = frame.xend - frame.xstart;
        if (i + 1 <= 32) {
          c_insertionsort(x, frame.xstart, frame.xend, cmp_workspace_a,
                          cmp_workspace_b);
          x_data = x->data;
        } else if (frame.depth == MAXDEPTH) {
          d_heapsort(x, frame.xstart, frame.xend, cmp_workspace_a,
                     cmp_workspace_b);
          x_data = x->data;
        } else {
          int xmid;
          boolean_T varargout_1;
          xmid = (frame.xstart + i / 2) - 1;
          i = cmp_workspace_a_data[x_data[xmid] - 1];
          pmax = x_data[frame.xstart - 1];
          pmin = cmp_workspace_a_data[pmax - 1];
          if (i < pmin) {
            varargout_1 = true;
          } else if (i == pmin) {
            varargout_1 = (cmp_workspace_b_data[x_data[xmid] - 1] <
                           cmp_workspace_b_data[pmax - 1]);
          } else {
            varargout_1 = false;
          }

          if (varargout_1) {
            x_data[frame.xstart - 1] = x_data[xmid];
            x_data[xmid] = pmax;
          }

          i = x_data[frame.xend - 1];
          pmax = cmp_workspace_a_data[i - 1];
          pmin = x_data[frame.xstart - 1];
          t = cmp_workspace_a_data[pmin - 1];
          if (pmax < t) {
            varargout_1 = true;
          } else if (pmax == t) {
            varargout_1 = (cmp_workspace_b_data[i - 1] <
                           cmp_workspace_b_data[pmin - 1]);
          } else {
            varargout_1 = false;
          }

          if (varargout_1) {
            x_data[frame.xstart - 1] = i;
            x_data[frame.xend - 1] = pmin;
          }

          i = x_data[frame.xend - 1];
          pmax = cmp_workspace_a_data[i - 1];
          pmin = cmp_workspace_a_data[x_data[xmid] - 1];
          if (pmax < pmin) {
            varargout_1 = true;
          } else if (pmax == pmin) {
            varargout_1 = (cmp_workspace_b_data[i - 1] <
                           cmp_workspace_b_data[x_data[xmid] - 1]);
          } else {
            varargout_1 = false;
          }

          if (varargout_1) {
            t = x_data[xmid];
            x_data[xmid] = i;
            x_data[frame.xend - 1] = t;
          }

          pow2p = x_data[xmid] - 1;
          x_data[xmid] = x_data[frame.xend - 2];
          x_data[frame.xend - 2] = pow2p + 1;
          pmax = frame.xstart - 1;
          pmin = frame.xend - 2;
          int exitg2;
          do {
            int exitg3;
            exitg2 = 0;
            pmax++;
            do {
              exitg3 = 0;
              i = cmp_workspace_a_data[x_data[pmax] - 1];
              if (i < cmp_workspace_a_data[pow2p]) {
                varargout_1 = true;
              } else if (i == cmp_workspace_a_data[pow2p]) {
                varargout_1 = (cmp_workspace_b_data[x_data[pmax] - 1] <
                               cmp_workspace_b_data[pow2p]);
              } else {
                varargout_1 = false;
              }

              if (varargout_1) {
                pmax++;
              } else {
                exitg3 = 1;
              }
            } while (exitg3 == 0);

            pmin--;
            do {
              exitg3 = 0;
              i = cmp_workspace_a_data[x_data[pmin] - 1];
              if (cmp_workspace_a_data[pow2p] < i) {
                varargout_1 = true;
              } else if (cmp_workspace_a_data[pow2p] == i) {
                varargout_1 = (cmp_workspace_b_data[pow2p] <
                               cmp_workspace_b_data[x_data[pmin] - 1]);
              } else {
                varargout_1 = false;
              }

              if (varargout_1) {
                pmin--;
              } else {
                exitg3 = 1;
              }
            } while (exitg3 == 0);

            if (pmax + 1 >= pmin + 1) {
              exitg2 = 1;
            } else {
              t = x_data[pmax];
              x_data[pmax] = x_data[pmin];
              x_data[pmin] = t;
            }
          } while (exitg2 == 0);

          x_data[frame.xend - 2] = x_data[pmax];
          x_data[pmax] = pow2p + 1;
          if (pmax + 2 < frame.xend) {
            st_d_data[st_n].xstart = pmax + 2;
            st_d_data[st_n].xend = frame.xend;
            st_d_data[st_n].depth = frame.depth + 1;
            st_n++;
          }

          if (frame.xstart < pmax + 1) {
            st_d_data[st_n].xstart = frame.xstart;
            st_d_data[st_n].xend = pmax + 1;
            st_d_data[st_n].depth = frame.depth + 1;
            st_n++;
          }
        }
      }
    }
  }
}

static void c_minus(emxArray_real_T *in1, const emxArray_real_T *in2, const
                    emxArray_real_T *in3)
{
  const double *in2_data;
  const double *in3_data;
  double *in1_data;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  in3_data = in3->data;
  in2_data = in2->data;
  if (in3->size[0] == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = in3->size[0];
  }

  i = in1->size[0];
  in1->size[0] = loop_ub;
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in3->size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = in2_data[i * stride_0_0] - in3_data[i * stride_1_0];
  }
}

static void c_mtimes(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *C)
{
  const double *A_data;
  const double *B_data;
  double *C_data;
  int i;
  int inner;
  int j;
  int k;
  int m;
  int n;
  B_data = B->data;
  A_data = A->data;
  m = A->size[0];
  inner = A->size[1];
  n = B->size[1];
  i = C->size[0] * C->size[1];
  C->size[1] = B->size[1];
  C->size[0] = A->size[0];
  emxEnsureCapacity_real_T(C, i);
  C_data = C->data;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) {
      double s;
      s = 0.0;
      for (k = 0; k < inner; k++) {
        s += A_data[k + A->size[1] * i] * B_data[j + B->size[1] * k];
      }

      C_data[j + C->size[1] * i] = s;
    }
  }
}

static void d_binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
  const emxArray_real_T *in3, const emxArray_real_T *in4, double in5)
{
  emxArray_real_T *b_in2;
  const double *in2_data;
  const double *in3_data;
  const double *in4_data;
  double *b_in2_data;
  double *in1_data;
  int aux_0_1;
  int aux_1_1;
  int i;
  int i1;
  int in3_idx_0;
  int stride_0_0;
  int stride_0_1;
  int stride_1_0;
  int stride_1_1;
  int unnamed_idx_1;
  in4_data = in4->data;
  in3_data = in3->data;
  in2_data = in2->data;
  in3_idx_0 = in3->size[0];
  unnamed_idx_1 = in4->size[0];
  i = in1->size[0] * in1->size[1];
  in1->size[1] = in3_idx_0;
  in1->size[0] = unnamed_idx_1;
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  for (i = 0; i < unnamed_idx_1; i++) {
    for (i1 = 0; i1 < in3_idx_0; i1++) {
      in1_data[i1 + in1->size[1] * i] = in3_data[i1] * in4_data[i] / in5;
    }
  }

  emxInit_real_T(&b_in2, 2);
  if (in1->size[1] == 1) {
    in3_idx_0 = in2->size[1];
  } else {
    in3_idx_0 = in1->size[1];
  }

  i = b_in2->size[0] * b_in2->size[1];
  b_in2->size[1] = in3_idx_0;
  if (in1->size[0] == 1) {
    unnamed_idx_1 = in2->size[0];
  } else {
    unnamed_idx_1 = in1->size[0];
  }

  b_in2->size[0] = unnamed_idx_1;
  emxEnsureCapacity_real_T(b_in2, i);
  b_in2_data = b_in2->data;
  stride_0_0 = (in2->size[1] != 1);
  stride_0_1 = (in2->size[0] != 1);
  stride_1_0 = (in1->size[1] != 1);
  stride_1_1 = (in1->size[0] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < unnamed_idx_1; i++) {
    for (i1 = 0; i1 < in3_idx_0; i1++) {
      b_in2_data[i1 + b_in2->size[1] * i] = in2_data[i1 * stride_0_0 + in2->
        size[1] * aux_0_1] - in1_data[i1 * stride_1_0 + in1->size[1] * aux_1_1];
    }

    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }

  i = in1->size[0] * in1->size[1];
  in1->size[1] = b_in2->size[1];
  in1->size[0] = b_in2->size[0];
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  in3_idx_0 = b_in2->size[0];
  for (i = 0; i < in3_idx_0; i++) {
    unnamed_idx_1 = b_in2->size[1];
    for (i1 = 0; i1 < unnamed_idx_1; i1++) {
      in1_data[i1 + in1->size[1] * i] = b_in2_data[i1 + b_in2->size[1] * i];
    }
  }

  emxFree_real_T(&b_in2);
}

static void d_heapify(emxArray_int32_T *x, int idx, int xstart, int xend)
{
  int cmpIdx;
  int extremum;
  int extremumIdx;
  int leftIdx;
  int *x_data;
  boolean_T changed;
  boolean_T exitg1;
  x_data = x->data;
  changed = true;
  extremumIdx = (idx + xstart) - 2;
  leftIdx = ((idx << 1) + xstart) - 1;
  exitg1 = false;
  while ((!exitg1) && (leftIdx < xend)) {
    int xcmp;
    changed = false;
    extremum = x_data[extremumIdx];
    cmpIdx = leftIdx - 1;
    xcmp = x_data[leftIdx - 1];
    if (xcmp < x_data[leftIdx]) {
      cmpIdx = leftIdx;
      xcmp = x_data[leftIdx];
    }

    if (x_data[extremumIdx] < xcmp) {
      x_data[extremumIdx] = xcmp;
      x_data[cmpIdx] = extremum;
      extremumIdx = cmpIdx;
      leftIdx = ((((cmpIdx - xstart) + 2) << 1) + xstart) - 1;
      changed = true;
    } else {
      exitg1 = true;
    }
  }

  if (changed && (leftIdx <= xend)) {
    extremum = x_data[extremumIdx];
    cmpIdx = x_data[leftIdx - 1];
    if (x_data[extremumIdx] < cmpIdx) {
      x_data[extremumIdx] = cmpIdx;
      x_data[leftIdx - 1] = extremum;
    }
  }
}

static void d_heapsort(emxArray_int32_T *x, int xstart, int xend, const
  emxArray_int32_T *cmp_workspace_a, const emxArray_int32_T *cmp_workspace_b)
{
  int idx;
  int k;
  int n;
  int *x_data;
  x_data = x->data;
  n = (xend - xstart) - 1;
  for (idx = n + 2; idx >= 1; idx--) {
    c_heapify(x, idx, xstart, xend, cmp_workspace_a, cmp_workspace_b);
    x_data = x->data;
  }

  for (k = 0; k <= n; k++) {
    int t;
    idx = (xend - k) - 1;
    t = x_data[idx];
    x_data[idx] = x_data[xstart - 1];
    x_data[xstart - 1] = t;
    c_heapify(x, 1, xstart, idx, cmp_workspace_a, cmp_workspace_b);
    x_data = x->data;
  }
}

static void d_insertionsort(emxArray_int32_T *x, int xstart, int xend)
{
  int i;
  int k;
  int *x_data;
  x_data = x->data;
  i = xstart + 1;
  for (k = i; k <= xend; k++) {
    int idx;
    int xc;
    boolean_T exitg1;
    xc = x_data[k - 1];
    idx = k - 1;
    exitg1 = false;
    while ((!exitg1) && (idx >= xstart)) {
      int i1;
      i1 = x_data[idx - 1];
      if (xc < i1) {
        x_data[idx] = i1;
        idx--;
      } else {
        exitg1 = true;
      }
    }

    x_data[idx] = xc;
  }
}

static void d_introsort(emxArray_int32_T *x, int xstart, int xend)
{
  struct_T frame;
  int nsort;
  int pmax;
  int pmin;
  int *x_data;
  x_data = x->data;
  if (xstart < xend) {
    nsort = (xend - xstart) + 1;
    if (nsort <= 32) {
      d_insertionsort(x, xstart, xend);
    } else {
      struct_T st_d_data[120];
      int MAXDEPTH;
      int pow2p;
      int t;
      boolean_T exitg1;
      pmax = 31;
      pmin = 0;
      exitg1 = false;
      while ((!exitg1) && (pmax - pmin > 1)) {
        t = (pmin + pmax) >> 1;
        pow2p = 1 << t;
        if (pow2p == nsort) {
          pmax = t;
          exitg1 = true;
        } else if (pow2p > nsort) {
          pmax = t;
        } else {
          pmin = t;
        }
      }

      MAXDEPTH = (pmax - 1) << 1;
      frame.xstart = xstart;
      frame.xend = xend;
      frame.depth = 0;
      nsort = MAXDEPTH << 1;
      for (pmin = 0; pmin < nsort; pmin++) {
        st_d_data[pmin] = frame;
      }

      st_d_data[0] = frame;
      pow2p = 1;
      while (pow2p > 0) {
        frame = st_d_data[pow2p - 1];
        pow2p--;
        pmin = frame.xend - frame.xstart;
        if (pmin + 1 <= 32) {
          d_insertionsort(x, frame.xstart, frame.xend);
          x_data = x->data;
        } else if (frame.depth == MAXDEPTH) {
          e_heapsort(x, frame.xstart, frame.xend);
          x_data = x->data;
        } else {
          pmax = (frame.xstart + pmin / 2) - 1;
          pmin = x_data[frame.xstart - 1];
          if (x_data[pmax] < pmin) {
            x_data[frame.xstart - 1] = x_data[pmax];
            x_data[pmax] = pmin;
          }

          pmin = x_data[frame.xstart - 1];
          nsort = x_data[frame.xend - 1];
          if (nsort < pmin) {
            x_data[frame.xstart - 1] = nsort;
            x_data[frame.xend - 1] = pmin;
          }

          pmin = x_data[frame.xend - 1];
          if (pmin < x_data[pmax]) {
            t = x_data[pmax];
            x_data[pmax] = pmin;
            x_data[frame.xend - 1] = t;
          }

          pmin = x_data[pmax];
          x_data[pmax] = x_data[frame.xend - 2];
          x_data[frame.xend - 2] = pmin;
          nsort = frame.xstart - 1;
          pmax = frame.xend - 2;
          int exitg2;
          do {
            exitg2 = 0;
            for (nsort++; x_data[nsort] < pmin; nsort++) {
            }

            for (pmax--; pmin < x_data[pmax]; pmax--) {
            }

            if (nsort + 1 >= pmax + 1) {
              exitg2 = 1;
            } else {
              t = x_data[nsort];
              x_data[nsort] = x_data[pmax];
              x_data[pmax] = t;
            }
          } while (exitg2 == 0);

          x_data[frame.xend - 2] = x_data[nsort];
          x_data[nsort] = pmin;
          if (nsort + 2 < frame.xend) {
            st_d_data[pow2p].xstart = nsort + 2;
            st_d_data[pow2p].xend = frame.xend;
            st_d_data[pow2p].depth = frame.depth + 1;
            pow2p++;
          }

          if (frame.xstart < nsort + 1) {
            st_d_data[pow2p].xstart = frame.xstart;
            st_d_data[pow2p].xend = nsort + 1;
            st_d_data[pow2p].depth = frame.depth + 1;
            pow2p++;
          }
        }
      }
    }
  }
}

static void d_mtimes(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *C)
{
  const double *A_data;
  const double *B_data;
  double *C_data;
  int i;
  int inner;
  int j;
  int k;
  int m;
  int n;
  B_data = B->data;
  A_data = A->data;
  m = A->size[1];
  inner = A->size[0];
  n = B->size[1];
  i = C->size[0] * C->size[1];
  C->size[1] = B->size[1];
  C->size[0] = A->size[1];
  emxEnsureCapacity_real_T(C, i);
  C_data = C->data;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) {
      double s;
      s = 0.0;
      for (k = 0; k < inner; k++) {
        s += A_data[i + A->size[1] * k] * B_data[j + B->size[1] * k];
      }

      C_data[j + C->size[1] * i] = s;
    }
  }
}

static void diag(const emxArray_real_T *v, emxArray_real_T *d)
{
  const double *v_data;
  double *d_data;
  int dlen;
  v_data = v->data;
  if ((v->size[0] == 1) && (v->size[1] == 1)) {
    int u1;
    u1 = d->size[0];
    d->size[0] = 1;
    emxEnsureCapacity_real_T(d, u1);
    d_data = d->data;
    d_data[0] = v_data[0];
  } else {
    int u1;
    dlen = v->size[0];
    u1 = v->size[1];
    if (dlen <= u1) {
      u1 = dlen;
    }

    if (v->size[1] > 0) {
      dlen = u1;
    } else {
      dlen = 0;
    }

    u1 = d->size[0];
    d->size[0] = dlen;
    emxEnsureCapacity_real_T(d, u1);
    d_data = d->data;
    u1 = dlen - 1;
    for (dlen = 0; dlen <= u1; dlen++) {
      d_data[dlen] = v_data[dlen + v->size[1] * dlen];
    }
  }
}

static void diff(const emxArray_real_T *x, emxArray_real_T *y)
{
  const double *x_data;
  double *y_data;
  int dimSize;
  int u0;
  x_data = x->data;
  dimSize = x->size[1];
  if (x->size[1] == 0) {
    y->size[1] = 0;
    y->size[0] = 1;
  } else {
    u0 = x->size[1] - 1;
    if (u0 > 1) {
      u0 = 1;
    }

    if (u0 < 1) {
      y->size[1] = 0;
      y->size[0] = 1;
    } else {
      u0 = y->size[0] * y->size[1];
      y->size[1] = x->size[1] - 1;
      y->size[0] = 1;
      emxEnsureCapacity_real_T(y, u0);
      y_data = y->data;
      if (x->size[1] - 1 != 0) {
        double work_data;
        work_data = x_data[0];
        for (u0 = 2; u0 <= dimSize; u0++) {
          double d;
          double tmp1;
          tmp1 = x_data[u0 - 1];
          d = tmp1;
          tmp1 -= work_data;
          work_data = d;
          y_data[u0 - 2] = tmp1;
        }
      }
    }
  }
}

static void e_binary_expand_op(emxArray_real_T *in1, double in2, const
  emxArray_real_T *in3, const emxArray_real_T *in4)
{
  const double *in3_data;
  const double *in4_data;
  double *in1_data;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  in4_data = in4->data;
  in3_data = in3->data;
  if (in4->size[0] == 1) {
    loop_ub = in3->size[0];
  } else {
    loop_ub = in4->size[0];
  }

  i = in1->size[0];
  in1->size[0] = loop_ub;
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  stride_0_0 = (in3->size[0] != 1);
  stride_1_0 = (in4->size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = in2 * in3_data[i * stride_0_0] + in4_data[i * stride_1_0];
  }
}

static void e_heapsort(emxArray_int32_T *x, int xstart, int xend)
{
  int idx;
  int k;
  int n;
  int *x_data;
  x_data = x->data;
  n = (xend - xstart) - 1;
  for (idx = n + 2; idx >= 1; idx--) {
    d_heapify(x, idx, xstart, xend);
    x_data = x->data;
  }

  for (k = 0; k <= n; k++) {
    int t;
    idx = (xend - k) - 1;
    t = x_data[idx];
    x_data[idx] = x_data[xstart - 1];
    x_data[xstart - 1] = t;
    d_heapify(x, 1, xstart, idx);
    x_data = x->data;
  }
}

static void e_mtimes(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *C)
{
  const double *A_data;
  const double *B_data;
  double *C_data;
  int i;
  int inner;
  int k;
  int m;
  B_data = B->data;
  A_data = A->data;
  m = A->size[0];
  inner = A->size[1];
  i = C->size[0];
  C->size[0] = A->size[0];
  emxEnsureCapacity_real_T(C, i);
  C_data = C->data;
  for (i = 0; i < m; i++) {
    double s;
    s = 0.0;
    for (k = 0; k < inner; k++) {
      s += A_data[k + A->size[1] * i] * B_data[k];
    }

    C_data[i] = s;
  }
}

static void eml_find(const emxArray_boolean_T *x, emxArray_int32_T *i)
{
  int idx;
  int ii;
  int nx;
  int *i_data;
  const boolean_T *x_data;
  boolean_T exitg1;
  x_data = x->data;
  nx = x->size[0];
  idx = 0;
  ii = i->size[0];
  i->size[0] = x->size[0];
  emxEnsureCapacity_int32_T(i, ii);
  i_data = i->data;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= nx - 1)) {
    if (x_data[ii]) {
      idx++;
      i_data[idx - 1] = ii + 1;
      if (idx >= nx) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }

  if (x->size[0] == 1) {
    if (idx == 0) {
      i->size[0] = 0;
    }
  } else {
    ii = i->size[0];
    if (idx < 1) {
      i->size[0] = 0;
    } else {
      i->size[0] = idx;
    }

    emxEnsureCapacity_int32_T(i, ii);
  }
}

static void f_binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
  const emxArray_real_T *in3)
{
  const double *in2_data;
  const double *in3_data;
  double *in1_data;
  int i;
  int in2_idx_0;
  int loop_ub;
  int stride_1_0;
  in3_data = in3->data;
  in2_data = in2->data;
  in2_idx_0 = in2->size[1];
  if (in3->size[0] == 1) {
    loop_ub = in2_idx_0;
  } else {
    loop_ub = in3->size[0];
  }

  i = in1->size[0];
  in1->size[0] = loop_ub;
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  in2_idx_0 = (in2_idx_0 != 1);
  stride_1_0 = (in3->size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = in2_data[i * in2_idx_0] - in3_data[i * stride_1_0];
  }
}

static void f_mtimes(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *C)
{
  const double *A_data;
  const double *B_data;
  double *C_data;
  int inner;
  int j;
  int k;
  int n;
  B_data = B->data;
  A_data = A->data;
  inner = A->size[1];
  n = B->size[1];
  j = C->size[0] * C->size[1];
  C->size[1] = B->size[1];
  C->size[0] = 1;
  emxEnsureCapacity_real_T(C, j);
  C_data = C->data;
  for (j = 0; j < n; j++) {
    double s;
    s = 0.0;
    for (k = 0; k < inner; k++) {
      s += A_data[k] * B_data[j + B->size[1] * k];
    }

    C_data[j] = s;
  }
}

static void g_binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
  const emxArray_real_T *in3, const emxArray_real_T *in4)
{
  const double *in2_data;
  const double *in3_data;
  const double *in4_data;
  double *in1_data;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  int stride_2_0;
  in4_data = in4->data;
  in3_data = in3->data;
  in2_data = in2->data;
  if (in4->size[0] == 1) {
    if (in3->size[0] == 1) {
      loop_ub = in2->size[0];
    } else {
      loop_ub = in3->size[0];
    }
  } else {
    loop_ub = in4->size[0];
  }

  i = in1->size[0];
  in1->size[0] = loop_ub;
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in3->size[0] != 1);
  stride_2_0 = (in4->size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = (in2_data[i * stride_0_0] - in3_data[i * stride_1_0]) /
      in4_data[i * stride_2_0];
  }
}

static void g_mtimes(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *C)
{
  const double *A_data;
  const double *B_data;
  double *C_data;
  int i;
  int inner;
  int j;
  int k;
  int m;
  int n;
  B_data = B->data;
  A_data = A->data;
  m = A->size[0];
  inner = A->size[1];
  n = B->size[0];
  i = C->size[0] * C->size[1];
  C->size[1] = B->size[0];
  C->size[0] = A->size[0];
  emxEnsureCapacity_real_T(C, i);
  C_data = C->data;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) {
      double s;
      s = 0.0;
      for (k = 0; k < inner; k++) {
        s += A_data[k + A->size[1] * i] * B_data[k + B->size[1] * j];
      }

      C_data[j + C->size[1] * i] = s;
    }
  }
}

static void get_A(double T, b_sparse *A)
{

  DELAY_FUNC1("get_A Start", PX4_SOLVER_WAIT_TIME_US)
  double yf[45];
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  int sortedIndices[45];
  int cptr;
  int k;
  signed char cidxInt[45];
  signed char ridxInt[45];
  t2 = 1.0 / T;
  t3 = t2 * t2;
  t4 = rt_powd_snf(t2, 3.0);
  t5 = t3 * t3;
  t6 = t3 * 2.0;
  t7 = t4 * 6.0;
  t8 = t5 * 24.0;
  yf[0] = 1.0;
  yf[1] = 1.0;
  yf[2] = t2;
  yf[3] = 1.0;
  yf[4] = t2;
  yf[5] = t6;
  yf[6] = 1.0;
  yf[7] = t2 * 2.0;
  yf[8] = t6;
  yf[9] = t7;
  yf[10] = 1.0;
  yf[11] = t2 * 3.0;
  yf[12] = t3 * 6.0;
  yf[13] = t7;
  yf[14] = t8;
  yf[15] = 1.0;
  yf[16] = t2 * 4.0;
  yf[17] = t3 * 12.0;
  yf[18] = t4 * 24.0;
  yf[19] = t8;
  yf[20] = 1.0;
  yf[21] = t2 * 5.0;
  yf[22] = t3 * 20.0;
  yf[23] = t4 * 60.0;
  yf[24] = t5 * 120.0;
  yf[25] = 1.0;
  yf[26] = t2 * 6.0;
  yf[27] = t3 * 30.0;
  yf[28] = t4 * 120.0;
  yf[29] = t5 * 360.0;
  yf[30] = 1.0;
  yf[31] = t2 * 7.0;
  yf[32] = t3 * 42.0;
  yf[33] = t4 * 210.0;
  yf[34] = t5 * 840.0;
  yf[35] = 1.0;
  yf[36] = t2 * 8.0;
  yf[37] = t3 * 56.0;
  yf[38] = t4 * 336.0;
  yf[39] = t5 * 1680.0;
  yf[40] = 1.0;
  yf[41] = t2 * 9.0;
  yf[42] = t3 * 72.0;
  yf[43] = t4 * 504.0;
  yf[44] = t5 * 3024.0;
  for (k = 0; k < 45; k++) {
    sortedIndices[k] = k + 1;
  }

  introsort(sortedIndices);
  cptr = A->d->size[0];
  A->d->size[0] = 45;
  emxEnsureCapacity_real_T(A->d, cptr);
  cptr = A->colidx->size[0];
  A->colidx->size[0] = 11;
  emxEnsureCapacity_int32_T(A->colidx, cptr);
  A->colidx->data[0] = 1;
  cptr = A->rowidx->size[0];
  A->rowidx->size[0] = 45;
  emxEnsureCapacity_int32_T(A->rowidx, cptr);
  for (k = 0; k < 45; k++) {
    cptr = sortedIndices[k];
    cidxInt[k] = iv[cptr - 1];
    ridxInt[k] = iv1[cptr - 1];
    A->d->data[k] = 0.0;
    A->rowidx->data[k] = 0;
  }

  cptr = 0;
  for (k = 0; k < 10; k++) {
    while ((cptr + 1 <= 45) && (cidxInt[cptr] == k + 1)) {
      A->rowidx->data[cptr] = ridxInt[cptr];
      cptr++;
    }

    A->colidx->data[k + 1] = cptr + 1;
  }

  for (k = 0; k < 45; k++) {
    A->d->data[k] = yf[sortedIndices[k] - 1];
  }

  DELAY_FUNC("get_A before b_sparse_fillIn", PX4_SOLVER_WAIT_TIME_US)

  b_sparse_fillIn(A);

  DELAY_FUNC_END("get_A End", PX4_SOLVER_WAIT_TIME_US)
}

static void get_Q_prime(double T, b_sparse *Q_prime)
{
  double yf[100];
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t15;
  double t18;
  double t19;
  double t2;
  double t20;
  double t21;
  double t22;
  double t27;
  double t28;
  double t29;
  double t3;
  double t30;
  double t33;
  double t34;
  double t35;
  double t36;
  double t37;
  double t4;
  double t41;
  double t42;
  double t5;
  double t6;
  double t9;
  int sortedIndices[100];
  int cptr;
  int k;
  signed char cidxInt[100];
  signed char ridxInt[100];
  t2 = 1.0 / T;
  t9 = T * 0.050505050505050504;
  t10 = T / 198.0;
  t3 = t2 * t2;
  t4 = rt_powd_snf(t2, 3.0);
  t6 = rt_powd_snf(t2, 5.0);
  t11 = t2 * 9.0909090909090917;
  t12 = t2 * 2.4242424242424243;
  t13 = t2 * 5.7575757575757578;
  t14 = t2 * 29.09090909090909;
  t5 = t3 * t3;
  t15 = t4 * 38.18181818181818;
  t18 = t3 * 15.757575757575758;
  t19 = t3 * 22.424242424242426;
  t20 = t3 * 121.8181818181818;
  t21 = t3 * 221.81818181818181;
  t22 = t4 * 701.81818181818176;
  t27 = t4 * 901.81818181818176;
  t28 = t4 * 1476.363636363636;
  t30 = t4 * 2036.363636363636;
  t35 = t6 * 16952.727272727268;
  t36 = t6 * 40116.36363636364;
  t37 = t6 * 42356.36363636364;
  t42 = rt_powd_snf(t2, 7.0) * 164945.4545454545;
  t29 = t5 * 1603.636363636364;
  t33 = t5 * 7916.363636363636;
  t34 = t5 * 9036.363636363636;
  t41 = rt_powd_snf(t3, 3.0) * 82472.727272727265;
  yf[0] = t42;
  yf[1] = t41;
  yf[2] = t35;
  yf[3] = t29;
  yf[4] = t15;
  yf[5] = -t42;
  yf[6] = t41;
  yf[7] = -t35;
  yf[8] = t29;
  yf[9] = -t15;
  yf[10] = t41;
  yf[11] = t37;
  yf[12] = t34;
  yf[13] = t27;
  yf[14] = t19;
  yf[15] = -t41;
  yf[16] = t36;
  yf[17] = -t33;
  yf[18] = t22;
  yf[19] = -t18;
  yf[20] = t35;
  yf[21] = t34;
  yf[22] = t30;
  yf[23] = t21;
  yf[24] = t13;
  yf[25] = -t35;
  yf[26] = t33;
  yf[27] = -t28;
  yf[28] = t20;
  yf[29] = -t12;
  yf[30] = t29;
  yf[31] = t27;
  yf[32] = t21;
  yf[33] = t14;
  yf[34] = 0.78787878787878785;
  yf[35] = -t29;
  yf[36] = t22;
  yf[37] = -t20;
  yf[38] = t11;
  yf[39] = -0.12121212121212122;
  yf[40] = t15;
  yf[41] = t19;
  yf[42] = t13;
  yf[43] = 0.78787878787878785;
  yf[44] = t9;
  yf[45] = -t15;
  yf[46] = t18;
  yf[47] = -t12;
  yf[48] = 0.12121212121212122;
  yf[49] = t10;
  yf[50] = -t42;
  yf[51] = -t41;
  yf[52] = -t35;
  yf[53] = -t29;
  yf[54] = -t15;
  yf[55] = t42;
  yf[56] = -t41;
  yf[57] = t35;
  yf[58] = -t29;
  yf[59] = t15;
  yf[60] = t41;
  yf[61] = t36;
  yf[62] = t33;
  yf[63] = t22;
  yf[64] = t18;
  yf[65] = -t41;
  yf[66] = t37;
  yf[67] = -t34;
  yf[68] = t27;
  yf[69] = -t19;
  yf[70] = -t35;
  yf[71] = -t33;
  yf[72] = -t28;
  yf[73] = -t20;
  yf[74] = -t12;
  yf[75] = t35;
  yf[76] = -t34;
  yf[77] = t30;
  yf[78] = -t21;
  yf[79] = t13;
  yf[80] = t29;
  yf[81] = t22;
  yf[82] = t20;
  yf[83] = t11;
  yf[84] = 0.12121212121212122;
  yf[85] = -t29;
  yf[86] = t27;
  yf[87] = -t21;
  yf[88] = t14;
  yf[89] = -0.78787878787878785;
  yf[90] = -t15;
  yf[91] = -t18;
  yf[92] = -t12;
  yf[93] = -0.12121212121212122;
  yf[94] = t10;
  yf[95] = t15;
  yf[96] = -t19;
  yf[97] = t13;
  yf[98] = -0.78787878787878785;
  yf[99] = t9;
  for (k = 0; k < 100; k++) {
    sortedIndices[k] = k + 1;
  }

  b_introsort(sortedIndices);
  cptr = Q_prime->d->size[0];
  Q_prime->d->size[0] = 100;
  emxEnsureCapacity_real_T(Q_prime->d, cptr);
  cptr = Q_prime->colidx->size[0];
  Q_prime->colidx->size[0] = 11;
  emxEnsureCapacity_int32_T(Q_prime->colidx, cptr);
  Q_prime->colidx->data[0] = 1;
  cptr = Q_prime->rowidx->size[0];
  Q_prime->rowidx->size[0] = 100;
  emxEnsureCapacity_int32_T(Q_prime->rowidx, cptr);
  for (k = 0; k < 100; k++) {
    cptr = sortedIndices[k];
    cidxInt[k] = iv2[cptr - 1];
    ridxInt[k] = iv3[cptr - 1];
    Q_prime->d->data[k] = 0.0;
    Q_prime->rowidx->data[k] = 0;
  }

  cptr = 0;
  for (k = 0; k < 10; k++) {
    while ((cptr + 1 <= 100) && (cidxInt[cptr] == k + 1)) {
      Q_prime->rowidx->data[cptr] = ridxInt[cptr];
      cptr++;
    }

    Q_prime->colidx->data[k + 1] = cptr + 1;
  }

  for (k = 0; k < 100; k++) {
    Q_prime->d->data[k] = yf[sortedIndices[k] - 1];
  }

  b_sparse_fillIn(Q_prime);
}

static double h_binary_expand_op(const emxArray_real_T *in1, const
  emxArray_real_T *in2, int *out2)
{
  emxArray_real_T *b_in1;
  const double *in1_data;
  const double *in2_data;
  double out1;
  double *b_in1_data;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  in2_data = in2->data;
  in1_data = in1->data;
  emxInit_real_T(&b_in1, 1);
  if (in2->size[0] == 1) {
    loop_ub = in1->size[0];
  } else {
    loop_ub = in2->size[0];
  }

  i = b_in1->size[0];
  b_in1->size[0] = loop_ub;
  emxEnsureCapacity_real_T(b_in1, i);
  b_in1_data = b_in1->data;
  stride_0_0 = (in1->size[0] != 1);
  stride_1_0 = (in2->size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    b_in1_data[i] = in1_data[i * stride_0_0] / in2_data[i * stride_1_0];
  }

  out1 = maximum(b_in1, out2);
  emxFree_real_T(&b_in1);
  return out1;
}

static void heapify(int x[45], int idx, int xstart, int xend)
{
  int ai;
  int aj;
  int extremum;
  int extremumIdx;
  int leftIdx;
  boolean_T changed;
  boolean_T exitg1;
  boolean_T varargout_1;
  changed = true;
  extremumIdx = (idx + xstart) - 2;
  leftIdx = ((idx << 1) + xstart) - 2;
  exitg1 = false;
  while ((!exitg1) && (leftIdx + 1 < xend)) {
    int aj_tmp_tmp;
    int cmpIdx;
    int xcmp;
    changed = false;
    extremum = x[extremumIdx];
    cmpIdx = leftIdx;
    xcmp = x[leftIdx];
    ai = iv[x[leftIdx] - 1];
    aj_tmp_tmp = x[leftIdx + 1];
    aj = iv[aj_tmp_tmp - 1];
    if (ai < aj) {
      varargout_1 = true;
    } else if (ai == aj) {
      varargout_1 = (iv1[x[leftIdx] - 1] < iv1[aj_tmp_tmp - 1]);
    } else {
      varargout_1 = false;
    }

    if (varargout_1) {
      cmpIdx = leftIdx + 1;
      xcmp = aj_tmp_tmp;
    }

    ai = iv[x[extremumIdx] - 1];
    aj_tmp_tmp = iv[xcmp - 1];
    if (ai < aj_tmp_tmp) {
      varargout_1 = true;
    } else if (ai == aj_tmp_tmp) {
      varargout_1 = (iv1[x[extremumIdx] - 1] < iv1[xcmp - 1]);
    } else {
      varargout_1 = false;
    }

    if (varargout_1) {
      x[extremumIdx] = xcmp;
      x[cmpIdx] = extremum;
      extremumIdx = cmpIdx;
      leftIdx = ((((cmpIdx - xstart) + 2) << 1) + xstart) - 2;
      changed = true;
    } else {
      exitg1 = true;
    }
  }

  if (changed && (leftIdx + 1 <= xend)) {
    extremum = x[extremumIdx];
    ai = iv[x[extremumIdx] - 1];
    aj = iv[x[leftIdx] - 1];
    if (ai < aj) {
      varargout_1 = true;
    } else if (ai == aj) {
      varargout_1 = (iv1[x[extremumIdx] - 1] < iv1[x[leftIdx] - 1]);
    } else {
      varargout_1 = false;
    }

    if (varargout_1) {
      x[extremumIdx] = x[leftIdx];
      x[leftIdx] = extremum;
    }
  }
}

static void i_binary_expand_op(emxArray_boolean_T *in1, const emxArray_real_T
  *in2, const c_TrajectoryOptimizer_DampedBFG *in3)
{
  const double *in2_data;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  boolean_T *in1_data;
  in2_data = in2->data;
  if (in3->ConstraintBound->size[0] == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = in3->ConstraintBound->size[0];
  }

  i = in1->size[0];
  in1->size[0] = loop_ub;
  emxEnsureCapacity_boolean_T(in1, i);
  in1_data = in1->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in3->ConstraintBound->size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = (in2_data[i * stride_0_0] >= in3->ConstraintBound->data[i *
                   stride_1_0]);
  }
}

static void insertionsort(int x[45], int xstart, int xend)
{
  int i;
  int k;
  i = xstart + 1;
  for (k = i; k <= xend; k++) {
    int idx;
    int xc;
    boolean_T exitg1;
    xc = x[k - 1];
    idx = k - 1;
    exitg1 = false;
    while ((!exitg1) && (idx >= xstart)) {
      int aj;
      int aj_tmp_tmp;
      int i1;
      boolean_T varargout_1;
      aj_tmp_tmp = x[idx - 1];
      aj = iv[aj_tmp_tmp - 1];
      i1 = iv[xc - 1];
      if (i1 < aj) {
        varargout_1 = true;
      } else if (i1 == aj) {
        varargout_1 = (iv1[xc - 1] < iv1[aj_tmp_tmp - 1]);
      } else {
        varargout_1 = false;
      }

      if (varargout_1) {
        x[idx] = aj_tmp_tmp;
        idx--;
      } else {
        exitg1 = true;
      }
    }

    x[idx] = xc;
  }
}

static void introsort(int x[45])
{
  struct_T st_d[20];
  int i;
  int st_n;
  for (i = 0; i < 20; i++) {
    st_d[i].xstart = 1;
    st_d[i].xend = 45;
    st_d[i].depth = 0;
  }

  st_d[0].xstart = 1;
  st_d[0].xend = 45;
  st_d[0].depth = 0;
  st_n = 1;
  while (st_n > 0) {
    struct_T expl_temp;
    int s_depth;
    int t;
    expl_temp = st_d[st_n - 1];
    s_depth = st_d[st_n - 1].depth;
    st_n--;
    t = expl_temp.xend - expl_temp.xstart;
    if (t + 1 <= 32) {
      insertionsort(x, expl_temp.xstart, expl_temp.xend);
    } else if (expl_temp.depth == 10) {
      b_heapsort(x, expl_temp.xstart, expl_temp.xend);
    } else {
      int ai;
      int aj;
      int j;
      int pivot;
      int xmid;
      boolean_T varargout_1;
      xmid = (expl_temp.xstart + t / 2) - 1;
      ai = iv[x[xmid] - 1];
      t = x[expl_temp.xstart - 1];
      aj = iv[t - 1];
      if (ai < aj) {
        varargout_1 = true;
      } else if (ai == aj) {
        varargout_1 = (iv1[x[xmid] - 1] < iv1[t - 1]);
      } else {
        varargout_1 = false;
      }

      if (varargout_1) {
        x[expl_temp.xstart - 1] = x[xmid];
        x[xmid] = t;
      }

      j = x[expl_temp.xend - 1];
      ai = iv[j - 1];
      t = x[expl_temp.xstart - 1];
      aj = iv[t - 1];
      if (ai < aj) {
        varargout_1 = true;
      } else if (ai == aj) {
        varargout_1 = (iv1[j - 1] < iv1[t - 1]);
      } else {
        varargout_1 = false;
      }

      if (varargout_1) {
        x[expl_temp.xstart - 1] = j;
        x[expl_temp.xend - 1] = t;
      }

      j = x[expl_temp.xend - 1];
      ai = iv[j - 1];
      aj = iv[x[xmid] - 1];
      if (ai < aj) {
        varargout_1 = true;
      } else if (ai == aj) {
        varargout_1 = (iv1[j - 1] < iv1[x[xmid] - 1]);
      } else {
        varargout_1 = false;
      }

      if (varargout_1) {
        t = x[xmid];
        x[xmid] = j;
        x[expl_temp.xend - 1] = t;
      }

      pivot = x[xmid] - 1;
      x[xmid] = x[expl_temp.xend - 2];
      x[expl_temp.xend - 2] = pivot + 1;
      i = expl_temp.xstart - 1;
      j = expl_temp.xend - 2;
      xmid = iv[pivot];
      int exitg1;
      do {
        int exitg2;
        exitg1 = 0;
        i++;
        do {
          exitg2 = 0;
          ai = iv[x[i] - 1];
          if (ai < xmid) {
            varargout_1 = true;
          } else if (ai == xmid) {
            varargout_1 = (iv1[x[i] - 1] < iv1[pivot]);
          } else {
            varargout_1 = false;
          }

          if (varargout_1) {
            i++;
          } else {
            exitg2 = 1;
          }
        } while (exitg2 == 0);

        j--;
        do {
          exitg2 = 0;
          aj = iv[x[j] - 1];
          if (xmid < aj) {
            varargout_1 = true;
          } else if (xmid == aj) {
            varargout_1 = (iv1[pivot] < iv1[x[j] - 1]);
          } else {
            varargout_1 = false;
          }

          if (varargout_1) {
            j--;
          } else {
            exitg2 = 1;
          }
        } while (exitg2 == 0);

        if (i + 1 >= j + 1) {
          exitg1 = 1;
        } else {
          t = x[i];
          x[i] = x[j];
          x[j] = t;
        }
      } while (exitg1 == 0);

      x[expl_temp.xend - 2] = x[i];
      x[i] = pivot + 1;
      if (i + 2 < expl_temp.xend) {
        st_d[st_n].xstart = i + 2;
        st_d[st_n].xend = expl_temp.xend;
        st_d[st_n].depth = s_depth + 1;
        st_n++;
      }

      if (expl_temp.xstart < i + 1) {
        st_d[st_n].xstart = expl_temp.xstart;
        st_d[st_n].xend = i + 1;
        st_d[st_n].depth = s_depth + 1;
        st_n++;
      }
    }
  }
}

static void inv(emxArray_real_T *x)
{
  emxArray_int32_T *ipiv;
  emxArray_int32_T *p;
  emxArray_real_T *b_x;
  emxArray_real_T *y;
  double *b_x_data;
  double *x_data;
  double *y_data;
  int b_i;
  int i;
  int i1;
  int j;
  int k;
  int *ipiv_data;
  int *p_data;
  boolean_T b;
  x_data = x->data;
  b = ((x->size[0] == 0) || (x->size[1] == 0));
  if (!b) {
    int b_n;
    int n;
    int yk;
    emxInit_real_T(&b_x, 2);
    i = b_x->size[0] * b_x->size[1];
    b_x->size[1] = x->size[0];
    b_x->size[0] = x->size[1];
    emxEnsureCapacity_real_T(b_x, i);
    b_x_data = b_x->data;
    yk = x->size[1];
    for (i = 0; i < yk; i++) {
      b_n = x->size[0];
      for (i1 = 0; i1 < b_n; i1++) {
        b_x_data[i1 + b_x->size[1] * i] = x_data[i + x->size[1] * i1];
      }
    }

    n = b_x->size[1];
    emxInit_real_T(&y, 2);
    i = y->size[0] * y->size[1];
    y->size[1] = b_x->size[1];
    y->size[0] = b_x->size[0];
    emxEnsureCapacity_real_T(y, i);
    y_data = y->data;
    yk = b_x->size[0];
    for (i = 0; i < yk; i++) {
      b_n = b_x->size[1];
      for (i1 = 0; i1 < b_n; i1++) {
        y_data[i1 + y->size[1] * i] = 0.0;
      }
    }

    emxInit_int32_T(&ipiv, 2);
    xzgetrf(b_x->size[1], b_x->size[1], b_x, b_x->size[1], ipiv);
    ipiv_data = ipiv->data;
    b_x_data = b_x->data;
    if (b_x->size[1] < 1) {
      b_n = 0;
    } else {
      b_n = b_x->size[1];
    }

    emxInit_int32_T(&p, 2);
    i = p->size[0] * p->size[1];
    p->size[1] = 1;
    p->size[0] = b_n;
    emxEnsureCapacity_int32_T(p, i);
    p_data = p->data;
    if (b_n > 0) {
      p_data[0] = 1;
      yk = 1;
      for (k = 2; k <= b_n; k++) {
        yk++;
        p_data[k - 1] = yk;
      }
    }

    i = ipiv->size[0];
    for (k = 0; k < i; k++) {
      i1 = ipiv_data[k];
      if (i1 > k + 1) {
        yk = p_data[i1 - 1];
        p_data[i1 - 1] = p_data[k];
        p_data[k] = yk;
      }
    }

    emxFree_int32_T(&ipiv);
    for (k = 0; k < n; k++) {
      i = p_data[k];
      y_data[k + y->size[1] * (i - 1)] = 1.0;
      for (j = k + 1; j <= n; j++) {
        if (fabs(y_data[(j + y->size[1] * (i - 1)) - 1]) >= DBL_EPSILON) {
          i1 = j + 1;
          for (b_i = i1; b_i <= n; b_i++) {
            y_data[(b_i + y->size[1] * (i - 1)) - 1] -= y_data[(j + y->size[1] *
              (i - 1)) - 1] * b_x_data[(b_i + b_x->size[1] * (j - 1)) - 1];
          }
        }
      }
    }

    emxFree_int32_T(&p);
    if (b_x->size[1] != 0) {
      for (j = 0; j < n; j++) {
        yk = n * j - 1;
        for (k = n; k >= 1; k--) {
          b_n = n * (k - 1) - 1;
          i = k + yk;
          if (fabs(y_data[i]) >= DBL_EPSILON) {
            y_data[i] /= b_x_data[k + b_n];
            for (b_i = 0; b_i <= k - 2; b_i++) {
              i1 = (b_i + yk) + 1;
              y_data[i1] -= y_data[i] * b_x_data[(b_i + b_n) + 1];
            }
          }
        }
      }
    }

    emxFree_real_T(&b_x);
    i = x->size[0] * x->size[1];
    x->size[1] = y->size[0];
    x->size[0] = y->size[1];
    emxEnsureCapacity_real_T(x, i);
    x_data = x->data;
    yk = y->size[1];
    for (i = 0; i < yk; i++) {
      b_n = y->size[0];
      for (i1 = 0; i1 < b_n; i1++) {
        x_data[i1 + x->size[1] * i] = y_data[i + y->size[1] * i1];
      }
    }

    emxFree_real_T(&y);
  }
}

static boolean_T isPositiveDefinite(const emxArray_real_T *B)
{
  emxArray_real_T *A;
  const double *B_data;
  double *A_data;
  int i;
  int i1;
  int ia;
  int iac;
  int info;
  int k;
  int mrows;
  int n;
  int nmj;
  B_data = B->data;
  emxInit_real_T(&A, 2);
  i = A->size[0] * A->size[1];
  A->size[1] = B->size[0];
  A->size[0] = B->size[1];
  emxEnsureCapacity_real_T(A, i);
  A_data = A->data;
  nmj = B->size[1];
  for (i = 0; i < nmj; i++) {
    k = B->size[0];
    for (i1 = 0; i1 < k; i1++) {
      A_data[i1 + A->size[1] * i] = B_data[i + B->size[1] * i1];
    }
  }

  mrows = A->size[1];
  nmj = A->size[1];
  n = A->size[0];
  if (nmj <= n) {
    n = nmj;
  }

  info = 0;
  if (n != 0) {
    int j;
    boolean_T exitg1;
    j = 0;
    exitg1 = false;
    while ((!exitg1) && (j <= n - 1)) {
      double ssq;
      int idxAjj;
      idxAjj = j + j * mrows;
      ssq = 0.0;
      if (j >= 1) {
        for (k = 0; k < j; k++) {
          nmj = j + k * mrows;
          ssq += A_data[nmj] * A_data[nmj];
        }
      }

      ssq = A_data[idxAjj] - ssq;
      if (ssq > 0.0) {
        ssq = sqrt(ssq);
        A_data[idxAjj] = ssq;
        if (j + 1 < n) {
          int idxAjp1j;
          nmj = (n - j) - 1;
          k = j + 2;
          idxAjp1j = idxAjj + 2;
          if ((nmj != 0) && (j != 0)) {
            int ix;
            ix = j;
            i = (j + mrows * (j - 1)) + 2;
            for (iac = k; mrows < 0 ? iac >= i : iac <= i; iac += mrows) {
              double c;
              c = -A_data[ix];
              i1 = (iac + nmj) - 1;
              for (ia = iac; ia <= i1; ia++) {
                int i2;
                i2 = ((idxAjj + ia) - iac) + 1;
                A_data[i2] += A_data[ia - 1] * c;
              }

              ix += mrows;
            }
          }

          ssq = 1.0 / ssq;
          i = idxAjj + nmj;
          for (k = idxAjp1j; k <= i + 1; k++) {
            A_data[k - 1] *= ssq;
          }
        }

        j++;
      } else {
        info = j + 1;
        exitg1 = true;
      }
    }
  }

  emxFree_real_T(&A);
  return info == 0;
}

static void j_binary_expand_op(emxArray_real_T *in1, const
  c_TrajectoryOptimizer_solver_me *in2)
{
  double *in1_data;
  int aux_0_1;
  int aux_1_1;
  int i;
  int i1;
  unsigned int in2_idx_0;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in2_idx_0 = (unsigned int)in2->waypoints_offset->size[0];
  i = in1->size[0] * in1->size[1];
  in1->size[1] = in2->wptFnc.workspace.wpts->size[1];
  if ((int)in2_idx_0 == 1) {
    in1->size[0] = in2->wptFnc.workspace.wpts->size[0];
  } else {
    in1->size[0] = (int)in2_idx_0;
  }

  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  stride_0_1 = (in2->wptFnc.workspace.wpts->size[0] != 1);
  stride_1_1 = ((int)in2_idx_0 != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  if ((int)in2_idx_0 == 1) {
    loop_ub = in2->wptFnc.workspace.wpts->size[0];
  } else {
    loop_ub = (int)in2_idx_0;
  }

  for (i = 0; i < loop_ub; i++) {
    int b_loop_ub;
    b_loop_ub = in2->wptFnc.workspace.wpts->size[1];
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      in1_data[i1 + in1->size[1] * i] = in2->wptFnc.workspace.wpts->data[i1 +
        in2->wptFnc.workspace.wpts->size[1] * aux_0_1] - in2->
        waypoints_offset->data[aux_1_1];
    }

    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
}

static double maximum(const emxArray_real_T *x, int *idx)
{
  const double *x_data;
  double ex;
  int k;
  int last;
  x_data = x->data;
  last = x->size[0];
  if (x->size[0] <= 2) {
    if (x->size[0] == 1) {
      ex = x_data[0];
      *idx = 1;
    } else {
      ex = x_data[x->size[0] - 1];
      if ((x_data[0] < ex) || (rtIsNaN(x_data[0]) && (!rtIsNaN(ex)))) {
        *idx = x->size[0];
      } else {
        ex = x_data[0];
        *idx = 1;
      }
    }
  } else {
    if (!rtIsNaN(x_data[0])) {
      *idx = 1;
    } else {
      boolean_T exitg1;
      *idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
        if (!rtIsNaN(x_data[k - 1])) {
          *idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (*idx == 0) {
      ex = x_data[0];
      *idx = 1;
    } else {
      int i;
      ex = x_data[*idx - 1];
      i = *idx + 1;
      for (k = i; k <= last; k++) {
        double d;
        d = x_data[k - 1];
        if (ex < d) {
          ex = d;
          *idx = k;
        }
      }
    }
  }

  return ex;
}

static double minimum(const emxArray_real_T *x, int *idx)
{
  const double *x_data;
  double ex;
  int k;
  int last;
  x_data = x->data;
  last = x->size[0];
  if (x->size[0] <= 2) {
    if (x->size[0] == 1) {
      ex = x_data[0];
      *idx = 1;
    } else {
      ex = x_data[x->size[0] - 1];
      if ((x_data[0] > ex) || (rtIsNaN(x_data[0]) && (!rtIsNaN(ex)))) {
        *idx = x->size[0];
      } else {
        ex = x_data[0];
        *idx = 1;
      }
    }
  } else {
    if (!rtIsNaN(x_data[0])) {
      *idx = 1;
    } else {
      boolean_T exitg1;
      *idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
        if (!rtIsNaN(x_data[k - 1])) {
          *idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (*idx == 0) {
      ex = x_data[0];
      *idx = 1;
    } else {
      int i;
      ex = x_data[*idx - 1];
      i = *idx + 1;
      for (k = i; k <= last; k++) {
        double d;
        d = x_data[k - 1];
        if (ex > d) {
          ex = d;
          *idx = k;
        }
      }
    }
  }

  return ex;
}

static void minus(emxArray_real_T *in1, const emxArray_real_T *in2)
{
  emxArray_real_T *b_in1;
  const double *in2_data;
  double *b_in1_data;
  double *in1_data;
  int aux_0_1;
  int aux_1_1;
  int b_loop_ub;
  int i;
  int i1;
  int loop_ub;
  int stride_0_0;
  int stride_0_1;
  int stride_1_0;
  int stride_1_1;
  in2_data = in2->data;
  in1_data = in1->data;
  emxInit_real_T(&b_in1, 2);
  if (in2->size[1] == 1) {
    loop_ub = in1->size[1];
  } else {
    loop_ub = in2->size[1];
  }

  i = b_in1->size[0] * b_in1->size[1];
  b_in1->size[1] = loop_ub;
  if (in2->size[0] == 1) {
    b_loop_ub = in1->size[0];
  } else {
    b_loop_ub = in2->size[0];
  }

  b_in1->size[0] = b_loop_ub;
  emxEnsureCapacity_real_T(b_in1, i);
  b_in1_data = b_in1->data;
  stride_0_0 = (in1->size[1] != 1);
  stride_0_1 = (in1->size[0] != 1);
  stride_1_0 = (in2->size[1] != 1);
  stride_1_1 = (in2->size[0] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < b_loop_ub; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      b_in1_data[i1 + b_in1->size[1] * i] = in1_data[i1 * stride_0_0 + in1->
        size[1] * aux_0_1] - in2_data[i1 * stride_1_0 + in2->size[1] * aux_1_1];
    }

    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }

  i = in1->size[0] * in1->size[1];
  in1->size[1] = b_in1->size[1];
  in1->size[0] = b_in1->size[0];
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  loop_ub = b_in1->size[0];
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = b_in1->size[1];
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      in1_data[i1 + in1->size[1] * i] = b_in1_data[i1 + b_in1->size[1] * i];
    }
  }

  emxFree_real_T(&b_in1);
}

static void mldivide(const emxArray_real_T *A, const emxArray_real_T *B,
                     emxArray_real_T *Y)
{
  emxArray_int32_T *jpvt;
  emxArray_real_T *b_A;
  emxArray_real_T *b_B;
  emxArray_real_T *b_Y;
  emxArray_real_T *tau;
  const double *A_data;
  const double *B_data;
  double *Y_data;
  double *b_A_data;
  double *b_B_data;
  double *b_Y_data;
  int b_i;
  int i;
  int i1;
  int j;
  int k;
  int *jpvt_data;
  B_data = B->data;
  A_data = A->data;
  emxInit_real_T(&b_Y, 2);
  emxInit_real_T(&b_B, 2);
  emxInit_real_T(&b_A, 2);
  emxInit_real_T(&tau, 1);
  emxInit_int32_T(&jpvt, 2);
  if ((A->size[0] == 0) || (A->size[1] == 0) || ((B->size[0] == 0) || (B->size[1]
        == 0))) {
    int minmn;
    i = Y->size[0] * Y->size[1];
    Y->size[1] = B->size[1];
    Y->size[0] = A->size[1];
    emxEnsureCapacity_real_T(Y, i);
    Y_data = Y->data;
    minmn = A->size[1];
    for (i = 0; i < minmn; i++) {
      int maxmn;
      maxmn = B->size[1];
      for (i1 = 0; i1 < maxmn; i1++) {
        Y_data[i1 + Y->size[1] * i] = 0.0;
      }
    }
  } else if (A->size[1] == A->size[0]) {
    int b_nb;
    int maxmn;
    int minmn;
    int mn;
    int nb;
    i = b_A->size[0] * b_A->size[1];
    b_A->size[1] = A->size[0];
    b_A->size[0] = A->size[1];
    emxEnsureCapacity_real_T(b_A, i);
    b_A_data = b_A->data;
    minmn = A->size[1];
    for (i = 0; i < minmn; i++) {
      maxmn = A->size[0];
      for (i1 = 0; i1 < maxmn; i1++) {
        b_A_data[i1 + b_A->size[1] * i] = A_data[i + A->size[1] * i1];
      }
    }

    i = b_B->size[0] * b_B->size[1];
    b_B->size[1] = B->size[0];
    b_B->size[0] = B->size[1];
    emxEnsureCapacity_real_T(b_B, i);
    b_B_data = b_B->data;
    minmn = B->size[1];
    for (i = 0; i < minmn; i++) {
      maxmn = B->size[0];
      for (i1 = 0; i1 < maxmn; i1++) {
        b_B_data[i1 + b_B->size[1] * i] = B_data[i + B->size[1] * i1];
      }
    }

    minmn = b_A->size[1];
    nb = b_A->size[0];
    if (minmn <= nb) {
      nb = minmn;
    }

    minmn = b_B->size[1];
    if (minmn <= nb) {
      nb = minmn;
    }

    minmn = b_B->size[0] - 1;
    xzgetrf(nb, nb, b_A, b_A->size[1], jpvt);
    jpvt_data = jpvt->data;
    b_A_data = b_A->data;
    for (b_i = 0; b_i <= nb - 2; b_i++) {
      i = jpvt_data[b_i];
      if (i != b_i + 1) {
        for (j = 0; j <= minmn; j++) {
          double tol;
          tol = b_B_data[b_i + b_B->size[1] * j];
          b_B_data[b_i + b_B->size[1] * j] = b_B_data[(i + b_B->size[1] * j) - 1];
          b_B_data[(i + b_B->size[1] * j) - 1] = tol;
        }
      }
    }

    for (j = 0; j <= minmn; j++) {
      b_nb = b_B->size[1] * j;
      for (k = 0; k < nb; k++) {
        mn = b_A->size[1] * k;
        i = k + b_nb;
        if (fabs(b_B_data[i]) >= DBL_EPSILON) {
          i1 = k + 2;
          for (b_i = i1; b_i <= nb; b_i++) {
            maxmn = (b_i + b_nb) - 1;
            b_B_data[maxmn] -= b_B_data[i] * b_A_data[(b_i + mn) - 1];
          }
        }
      }
    }

    minmn = b_B->size[0];
    for (j = 0; j < minmn; j++) {
      b_nb = b_B->size[1] * j - 1;
      for (k = nb; k >= 1; k--) {
        mn = b_A->size[1] * (k - 1) - 1;
        i = k + b_nb;
        if (fabs(b_B_data[i]) >= DBL_EPSILON) {
          b_B_data[i] /= b_A_data[k + mn];
          for (b_i = 0; b_i <= k - 2; b_i++) {
            i1 = (b_i + b_nb) + 1;
            b_B_data[i1] -= b_B_data[i] * b_A_data[(b_i + mn) + 1];
          }
        }
      }
    }

    i = Y->size[0] * Y->size[1];
    Y->size[1] = b_B->size[0];
    Y->size[0] = b_B->size[1];
    emxEnsureCapacity_real_T(Y, i);
    Y_data = Y->data;
    minmn = b_B->size[1];
    for (i = 0; i < minmn; i++) {
      maxmn = b_B->size[0];
      for (i1 = 0; i1 < maxmn; i1++) {
        Y_data[i1 + Y->size[1] * i] = b_B_data[i + b_B->size[1] * i1];
      }
    }
  } else {
    double tol;
    int b_nb;
    int maxmn;
    int minmn;
    int mn;
    int nb;
    int rankA;
    i = b_B->size[0] * b_B->size[1];
    b_B->size[1] = B->size[0];
    b_B->size[0] = B->size[1];
    emxEnsureCapacity_real_T(b_B, i);
    b_B_data = b_B->data;
    minmn = B->size[1];
    for (i = 0; i < minmn; i++) {
      maxmn = B->size[0];
      for (i1 = 0; i1 < maxmn; i1++) {
        b_B_data[i1 + b_B->size[1] * i] = B_data[i + B->size[1] * i1];
      }
    }

    i = b_A->size[0] * b_A->size[1];
    b_A->size[1] = A->size[0];
    b_A->size[0] = A->size[1];
    emxEnsureCapacity_real_T(b_A, i);
    b_A_data = b_A->data;
    minmn = A->size[1];
    for (i = 0; i < minmn; i++) {
      maxmn = A->size[0];
      for (i1 = 0; i1 < maxmn; i1++) {
        b_A_data[i1 + b_A->size[1] * i] = A_data[i + A->size[1] * i1];
      }
    }

    xgeqp3(b_A, tau, jpvt);
    jpvt_data = jpvt->data;
    Y_data = tau->data;
    b_A_data = b_A->data;
    rankA = 0;
    if (b_A->size[1] < b_A->size[0]) {
      minmn = b_A->size[1];
      maxmn = b_A->size[0];
    } else {
      minmn = b_A->size[0];
      maxmn = b_A->size[1];
    }

    if (minmn > 0) {
      tol = fmin(1.4901161193847656E-8, 2.2204460492503131E-15 * (double)maxmn) *
        fabs(b_A_data[0]);
      while ((rankA < minmn) && (!(fabs(b_A_data[rankA + b_A->size[1] * rankA]) <=
               tol))) {
        rankA++;
      }
    }

    nb = b_B->size[0];
    i = b_Y->size[0] * b_Y->size[1];
    b_Y->size[1] = b_A->size[0];
    b_Y->size[0] = b_B->size[0];
    emxEnsureCapacity_real_T(b_Y, i);
    b_Y_data = b_Y->data;
    minmn = b_B->size[0];
    for (i = 0; i < minmn; i++) {
      maxmn = b_A->size[0];
      for (i1 = 0; i1 < maxmn; i1++) {
        b_Y_data[i1 + b_Y->size[1] * i] = 0.0;
      }
    }

    maxmn = b_A->size[1];
    b_nb = b_B->size[0];
    minmn = b_A->size[1];
    mn = b_A->size[0];
    if (minmn <= mn) {
      mn = minmn;
    }

    for (j = 0; j < mn; j++) {
      if (fabs(Y_data[j]) >= DBL_EPSILON) {
        i = j + 2;
        for (k = 0; k < b_nb; k++) {
          tol = b_B_data[j + b_B->size[1] * k];
          for (b_i = i; b_i <= maxmn; b_i++) {
            tol += b_A_data[(b_i + b_A->size[1] * j) - 1] * b_B_data[(b_i +
              b_B->size[1] * k) - 1];
          }

          tol *= Y_data[j];
          if (fabs(tol) >= DBL_EPSILON) {
            b_B_data[j + b_B->size[1] * k] -= tol;
            for (b_i = i; b_i <= maxmn; b_i++) {
              b_B_data[(b_i + b_B->size[1] * k) - 1] -= b_A_data[(b_i +
                b_A->size[1] * j) - 1] * tol;
            }
          }
        }
      }
    }

    for (k = 0; k < nb; k++) {
      for (b_i = 0; b_i < rankA; b_i++) {
        b_Y_data[(jpvt_data[b_i] + b_Y->size[1] * k) - 1] = b_B_data[b_i +
          b_B->size[1] * k];
      }

      for (j = rankA; j >= 1; j--) {
        i = jpvt_data[j - 1];
        b_Y_data[(i + b_Y->size[1] * k) - 1] /= b_A_data[(j + b_A->size[1] * (j
          - 1)) - 1];
        for (b_i = 0; b_i <= j - 2; b_i++) {
          b_Y_data[(jpvt_data[b_i] + b_Y->size[1] * k) - 1] -= b_Y_data[(i +
            b_Y->size[1] * k) - 1] * b_A_data[b_i + b_A->size[1] * (j - 1)];
        }
      }
    }

    i = Y->size[0] * Y->size[1];
    Y->size[1] = b_Y->size[0];
    Y->size[0] = b_Y->size[1];
    emxEnsureCapacity_real_T(Y, i);
    Y_data = Y->data;
    minmn = b_Y->size[1];
    for (i = 0; i < minmn; i++) {
      maxmn = b_Y->size[0];
      for (i1 = 0; i1 < maxmn; i1++) {
        Y_data[i1 + Y->size[1] * i] = b_Y_data[i + b_Y->size[1] * i1];
      }
    }
  }

  emxFree_int32_T(&jpvt);
  emxFree_real_T(&tau);
  emxFree_real_T(&b_A);
  emxFree_real_T(&b_B);
  emxFree_real_T(&b_Y);
}

static void mtimes(const emxArray_real_T *A, const emxArray_real_T *B,
                   emxArray_real_T *C)
{
  const double *A_data;
  const double *B_data;
  double *C_data;
  int i;
  int inner;
  int k;
  int m;
  B_data = B->data;
  A_data = A->data;
  m = A->size[1];
  inner = A->size[0];
  i = C->size[0];
  C->size[0] = A->size[1];
  emxEnsureCapacity_real_T(C, i);
  C_data = C->data;
  for (i = 0; i < m; i++) {
    double s;
    s = 0.0;
    for (k = 0; k < inner; k++) {
      s += A_data[i + A->size[1] * k] * B_data[k];
    }

    C_data[i] = s;
  }
}

static void plus(emxArray_real_T *in1, const emxArray_real_T *in2)
{
  emxArray_real_T *b_in1;
  const double *in2_data;
  double *b_in1_data;
  double *in1_data;
  int aux_0_1;
  int aux_1_1;
  int b_loop_ub;
  int i;
  int i1;
  int loop_ub;
  int stride_0_0;
  int stride_0_1;
  int stride_1_0;
  int stride_1_1;
  in2_data = in2->data;
  in1_data = in1->data;
  emxInit_real_T(&b_in1, 2);
  if (in2->size[1] == 1) {
    loop_ub = in1->size[1];
  } else {
    loop_ub = in2->size[1];
  }

  i = b_in1->size[0] * b_in1->size[1];
  b_in1->size[1] = loop_ub;
  if (in2->size[0] == 1) {
    b_loop_ub = in1->size[0];
  } else {
    b_loop_ub = in2->size[0];
  }

  b_in1->size[0] = b_loop_ub;
  emxEnsureCapacity_real_T(b_in1, i);
  b_in1_data = b_in1->data;
  stride_0_0 = (in1->size[1] != 1);
  stride_0_1 = (in1->size[0] != 1);
  stride_1_0 = (in2->size[1] != 1);
  stride_1_1 = (in2->size[0] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < b_loop_ub; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      b_in1_data[i1 + b_in1->size[1] * i] = in1_data[i1 * stride_0_0 + in1->
        size[1] * aux_0_1] + in2_data[i1 * stride_1_0 + in2->size[1] * aux_1_1];
    }

    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }

  i = in1->size[0] * in1->size[1];
  in1->size[1] = b_in1->size[1];
  in1->size[0] = b_in1->size[0];
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  loop_ub = b_in1->size[0];
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = b_in1->size[1];
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      in1_data[i1 + in1->size[1] * i] = b_in1_data[i1 + b_in1->size[1] * i];
    }
  }

  emxFree_real_T(&b_in1);
}

static double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double b;
  double y;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = b * sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    y = a * sqrt(b * b + 1.0);
  } else if (rtIsNaN(b)) {
    y = rtNaN;
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (fabs(d - 1.0) < DBL_EPSILON) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (fabs(d1) < DBL_EPSILON) {
      y = 1.0;
    } else if (fabs(d1 - 1.0) < DBL_EPSILON) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (fabs(u1 - 2.0) < DBL_EPSILON) {
      y = u0 * u0;
    } else if ((fabs(u1 - 0.5) < DBL_EPSILON) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

static double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

static int solver_common_constructM(const c_TrajectoryOptimizer_solver_me *this,
  const emxArray_real_T *xCons, emxArray_real_T *M_d, emxArray_int32_T *M_colidx,
  emxArray_int32_T *M_rowidx, int *M_n)
{
  emxArray_boolean_T *b_tmp;
  emxArray_boolean_T *tmp;
  emxArray_int32_T *cidxInt;
  emxArray_int32_T *ridxInt;
  emxArray_int32_T *sortedIndices;
  emxArray_int32_T *t;
  emxArray_real_T *col_vec;
  emxArray_real_T *row_vec;
  emxArray_real_T *y;
  sparse M_c;
  sparse M_r;
  const double *xCons_data;
  double b;
  double nCons;
  double *col_vec_data;
  double *row_vec_data;
  double *y_data;
  int M_m;
  int i;
  int k;
  int nc;
  int ns;
  int ny;
  int row;
  int *cidxInt_data;
  int *ridxInt_data;
  int *sortedIndices_data;
  int *t_data;
  boolean_T *b_tmp_data;
  boolean_T *tmp_data;
  xCons_data = xCons->data;
  nCons = this->N_wps * 5.0;
  emxInit_boolean_T(&tmp);
  i = tmp->size[0];
  tmp->size[0] = xCons->size[0];
  emxEnsureCapacity_boolean_T(tmp, i);
  tmp_data = tmp->data;
  ns = xCons->size[0];
  for (i = 0; i < ns; i++) {
    tmp_data[i] = rtIsNaN(xCons_data[i]);
  }

  emxInit_real_T(&y, 2);
  y_data = y->data;
  if (rtIsNaN(nCons)) {
    i = y->size[0] * y->size[1];
    y->size[1] = 1;
    y->size[0] = 1;
    emxEnsureCapacity_real_T(y, i);
    y_data = y->data;
    y_data[0] = rtNaN;
  } else if (nCons < 1.0) {
    y->size[1] = 0;
    y->size[0] = 1;
  } else {
    ns = (int)(nCons - 1.0) + 1;
    i = y->size[0] * y->size[1];
    y->size[1] = (int)(nCons - 1.0) + 1;
    y->size[0] = 1;
    emxEnsureCapacity_real_T(y, i);
    y_data = y->data;
    for (i = 0; i < ns; i++) {
      y_data[i] = (double)i + 1.0;
    }
  }

  emxInit_boolean_T(&b_tmp);
  i = b_tmp->size[0];
  b_tmp->size[0] = tmp->size[0];
  emxEnsureCapacity_boolean_T(b_tmp, i);
  b_tmp_data = b_tmp->data;
  ns = tmp->size[0];
  for (i = 0; i < ns; i++) {
    b_tmp_data[i] = !tmp_data[i];
  }

  emxInit_int32_T(&ridxInt, 1);
  eml_find(b_tmp, ridxInt);
  ridxInt_data = ridxInt->data;
  emxFree_boolean_T(&b_tmp);
  emxInit_int32_T(&cidxInt, 1);
  eml_find(tmp, cidxInt);
  cidxInt_data = cidxInt->data;
  emxFree_boolean_T(&tmp);
  emxInit_int32_T(&sortedIndices, 1);
  i = sortedIndices->size[0];
  sortedIndices->size[0] = ridxInt->size[0] + cidxInt->size[0];
  emxEnsureCapacity_int32_T(sortedIndices, i);
  sortedIndices_data = sortedIndices->data;
  ns = ridxInt->size[0];
  for (i = 0; i < ns; i++) {
    sortedIndices_data[i] = ridxInt_data[i];
  }

  ns = cidxInt->size[0];
  for (i = 0; i < ns; i++) {
    sortedIndices_data[i + ridxInt->size[0]] = cidxInt_data[i];
  }

  emxInitStruct_sparse(&M_r);
  nc = y->size[1];
  ns = sortedIndices->size[0];
  i = ridxInt->size[0];
  ridxInt->size[0] = sortedIndices->size[0];
  emxEnsureCapacity_int32_T(ridxInt, i);
  ridxInt_data = ridxInt->data;
  for (k = 0; k < ns; k++) {
    ridxInt_data[k] = sortedIndices_data[k];
  }

  ns = y->size[1];
  i = cidxInt->size[0];
  cidxInt->size[0] = y->size[1];
  emxEnsureCapacity_int32_T(cidxInt, i);
  cidxInt_data = cidxInt->data;
  i = sortedIndices->size[0];
  sortedIndices->size[0] = y->size[1];
  emxEnsureCapacity_int32_T(sortedIndices, i);
  sortedIndices_data = sortedIndices->data;
  for (k = 0; k < ns; k++) {
    cidxInt_data[k] = (int)y_data[k];
    sortedIndices_data[k] = k + 1;
  }

  c_introsort(sortedIndices, cidxInt->size[0], cidxInt, ridxInt);
  sortedIndices_data = sortedIndices->data;
  ny = cidxInt->size[0];
  emxInit_int32_T(&t, 1);
  i = t->size[0];
  t->size[0] = cidxInt->size[0];
  emxEnsureCapacity_int32_T(t, i);
  t_data = t->data;
  ns = cidxInt->size[0];
  for (i = 0; i < ns; i++) {
    t_data[i] = cidxInt_data[i];
  }

  for (k = 0; k < ny; k++) {
    cidxInt_data[k] = t_data[sortedIndices_data[k] - 1];
  }

  ny = ridxInt->size[0];
  i = t->size[0];
  t->size[0] = ridxInt->size[0];
  emxEnsureCapacity_int32_T(t, i);
  t_data = t->data;
  ns = ridxInt->size[0];
  for (i = 0; i < ns; i++) {
    t_data[i] = ridxInt_data[i];
  }

  for (k = 0; k < ny; k++) {
    ridxInt_data[k] = t_data[sortedIndices_data[k] - 1];
  }

  if ((ridxInt->size[0] == 0) || (cidxInt->size[0] == 0)) {
    ny = 0;
    M_r.n = 0;
  } else {
    ns = ridxInt->size[0];
    ny = ridxInt_data[0];
    for (k = 2; k <= ns; k++) {
      i = ridxInt_data[k - 1];
      if (ny < i) {
        ny = i;
      }
    }

    M_r.n = cidxInt_data[cidxInt->size[0] - 1];
  }

  M_r.m = ny;
  ns = y->size[1];
  if (ns < 1) {
    ns = 1;
  }

  emxFree_real_T(&y);
  i = M_r.d->size[0];
  M_r.d->size[0] = ns;
  emxEnsureCapacity_real_T(M_r.d, i);
  M_r.maxnz = ns;
  i = M_r.colidx->size[0];
  M_r.colidx->size[0] = M_r.n + 1;
  emxEnsureCapacity_int32_T(M_r.colidx, i);
  M_r.colidx->data[0] = 1;
  i = M_r.rowidx->size[0];
  M_r.rowidx->size[0] = ns;
  emxEnsureCapacity_int32_T(M_r.rowidx, i);
  for (i = 0; i < ns; i++) {
    M_r.d->data[i] = 0.0;
    M_r.rowidx->data[i] = 0;
  }

  ns = 0;
  i = M_r.n;
  for (ny = 0; ny < i; ny++) {
    while ((ns + 1 <= nc) && (cidxInt_data[ns] == ny + 1)) {
      M_r.rowidx->data[ns] = ridxInt_data[ns];
      ns++;
    }

    M_r.colidx->data[ny + 1] = ns + 1;
  }

  for (k = 0; k < nc; k++) {
    M_r.d->data[k] = 1.0;
  }

  sparse_fillIn(&M_r);
  ny = (int)this->stateSize;
  emxInit_real_T(&col_vec, 2);
  i = col_vec->size[0] * col_vec->size[1];
  col_vec->size[1] = ny;
  col_vec->size[0] = 1;
  emxEnsureCapacity_real_T(col_vec, i);
  col_vec_data = col_vec->data;
  for (i = 0; i < ny; i++) {
    col_vec_data[i] = 0.0;
  }

  b = this->stateSize;
  emxInit_real_T(&row_vec, 2);
  row_vec_data = row_vec->data;
  if (rtIsNaN(b)) {
    i = row_vec->size[0] * row_vec->size[1];
    row_vec->size[1] = 1;
    row_vec->size[0] = 1;
    emxEnsureCapacity_real_T(row_vec, i);
    row_vec_data = row_vec->data;
    row_vec_data[0] = rtNaN;
  } else if (b < 1.0) {
    row_vec->size[1] = 0;
    row_vec->size[0] = 1;
  } else {
    ns = (int)(b - 1.0) + 1;
    i = row_vec->size[0] * row_vec->size[1];
    row_vec->size[1] = (int)(b - 1.0) + 1;
    row_vec->size[0] = 1;
    emxEnsureCapacity_real_T(row_vec, i);
    row_vec_data = row_vec->data;
    for (i = 0; i < ns; i++) {
      row_vec_data[i] = (double)i + 1.0;
    }
  }

  for (row = 0; row < ny; row++) {
    col_vec_data[row] = ((double)row + 1.0) - (ceil(((double)row + 1.0) / 10.0)
      - 1.0) * 5.0;
  }

  emxInitStruct_sparse(&M_c);
  nc = col_vec->size[1];
  ns = row_vec->size[1];
  i = ridxInt->size[0];
  ridxInt->size[0] = row_vec->size[1];
  emxEnsureCapacity_int32_T(ridxInt, i);
  ridxInt_data = ridxInt->data;
  for (k = 0; k < ns; k++) {
    ridxInt_data[k] = (int)row_vec_data[k];
  }

  emxFree_real_T(&row_vec);
  ns = col_vec->size[1];
  i = cidxInt->size[0];
  cidxInt->size[0] = col_vec->size[1];
  emxEnsureCapacity_int32_T(cidxInt, i);
  cidxInt_data = cidxInt->data;
  i = sortedIndices->size[0];
  sortedIndices->size[0] = col_vec->size[1];
  emxEnsureCapacity_int32_T(sortedIndices, i);
  sortedIndices_data = sortedIndices->data;
  for (k = 0; k < ns; k++) {
    cidxInt_data[k] = (int)col_vec_data[k];
    sortedIndices_data[k] = k + 1;
  }

  c_introsort(sortedIndices, cidxInt->size[0], cidxInt, ridxInt);
  sortedIndices_data = sortedIndices->data;
  ny = cidxInt->size[0];
  i = t->size[0];
  t->size[0] = cidxInt->size[0];
  emxEnsureCapacity_int32_T(t, i);
  t_data = t->data;
  ns = cidxInt->size[0];
  for (i = 0; i < ns; i++) {
    t_data[i] = cidxInt_data[i];
  }

  for (k = 0; k < ny; k++) {
    cidxInt_data[k] = t_data[sortedIndices_data[k] - 1];
  }

  ny = ridxInt->size[0];
  i = t->size[0];
  t->size[0] = ridxInt->size[0];
  emxEnsureCapacity_int32_T(t, i);
  t_data = t->data;
  ns = ridxInt->size[0];
  for (i = 0; i < ns; i++) {
    t_data[i] = ridxInt_data[i];
  }

  for (k = 0; k < ny; k++) {
    ridxInt_data[k] = t_data[sortedIndices_data[k] - 1];
  }

  emxFree_int32_T(&t);
  emxFree_int32_T(&sortedIndices);
  if ((ridxInt->size[0] == 0) || (cidxInt->size[0] == 0)) {
    ny = 0;
    M_c.n = 0;
  } else {
    ns = ridxInt->size[0];
    ny = ridxInt_data[0];
    for (k = 2; k <= ns; k++) {
      i = ridxInt_data[k - 1];
      if (ny < i) {
        ny = i;
      }
    }

    M_c.n = cidxInt_data[cidxInt->size[0] - 1];
  }

  M_c.m = ny;
  ns = col_vec->size[1];
  if (ns < 1) {
    ns = 1;
  }

  emxFree_real_T(&col_vec);
  i = M_c.d->size[0];
  M_c.d->size[0] = ns;
  emxEnsureCapacity_real_T(M_c.d, i);
  M_c.maxnz = ns;
  i = M_c.colidx->size[0];
  M_c.colidx->size[0] = M_c.n + 1;
  emxEnsureCapacity_int32_T(M_c.colidx, i);
  M_c.colidx->data[0] = 1;
  i = M_c.rowidx->size[0];
  M_c.rowidx->size[0] = ns;
  emxEnsureCapacity_int32_T(M_c.rowidx, i);
  for (i = 0; i < ns; i++) {
    M_c.d->data[i] = 0.0;
    M_c.rowidx->data[i] = 0;
  }

  ns = 0;
  i = M_c.n;
  for (ny = 0; ny < i; ny++) {
    while ((ns + 1 <= nc) && (cidxInt_data[ns] == ny + 1)) {
      M_c.rowidx->data[ns] = ridxInt_data[ns];
      ns++;
    }

    M_c.colidx->data[ny + 1] = ns + 1;
  }

  emxFree_int32_T(&cidxInt);
  emxFree_int32_T(&ridxInt);
  for (k = 0; k < nc; k++) {
    M_c.d->data[k] = 1.0;
  }

  sparse_fillIn(&M_c);
  M_m = sparse_mtimes(M_c.d, M_c.colidx, M_c.rowidx, M_c.m, M_r.d, M_r.colidx,
                      M_r.rowidx, M_r.n, M_d, M_colidx, M_rowidx, M_n, &ns);
  emxFreeStruct_sparse(&M_c);
  emxFreeStruct_sparse(&M_r);
  return M_m;
}

static double solver_common_optimize(c_TrajectoryOptimizer_solver_me *this_,
  const emxArray_real_T *initGuess, emxArray_real_T *p, emxArray_real_T *t, char
  exitstruct_Status_data[], int exitstruct_Status_size[2], double
  *exitstruct_Iterations, double *exitstruct_RRAttempts, double
  *exitstruct_Error, double *exitstruct_ExitFlag)
{
  static const char cv[14] = { 'b', 'e', 's', 't', ' ', 'a', 'v', 'a', 'i', 'l',
    'a', 'b', 'l', 'e' };

  static const char cv1[7] = { 's', 'u', 'c', 'c', 'e', 's', 's' };

  c_TrajectoryOptimizer_DampedBFG solver;
  cell_wrap_1 reshapes[2];
  emxArray_int8_T *A1;
  emxArray_int8_T *A11;
  emxArray_int8_T *args_grads;
  emxArray_int8_T *b_I;
  emxArray_real_T *b1;
  emxArray_real_T *b_b1;
  emxArray_real_T *reshapes_f1;
  emxArray_real_T *xSol;
  const double *initGuess_data;
  double J;
  double err;
  double errPrev;
  double exitFlag;
  double iter;
  double iterations;
  double reshapes_f2;
  double rrAttempts;
  double tol;
  double *b1_data;
  double *reshapes_f1_data;
  double *t_data;
  double *xSol_data;
  int i;
  int i1;
  int i2;
  int i3;
  int i4;
  int input_sizes_idx_1;
  int k;
  int m;
  signed char *A11_data;
  signed char *A1_data;
  signed char *I_data;
  signed char *args_grads_data;
  boolean_T exitg1;
  boolean_T sizes_idx_0_tmp;
  initGuess_data = initGuess->data;
  d_emxInitStruct_TrajectoryOptim(&solver);
  solver.MaxNumIteration = 300.0;
  solver.MaxTime = 10.0;
  solver.GradientTolerance = 1.0E-7;
  solver.SolutionTolerance = 1.0E-6;
  solver.ArmijoRuleBeta = 0.4;
  solver.ArmijoRuleSigma = 1.0E-5;
  solver.StepTolerance = 1.0E-9;
  solver.ConstraintMatrix->size[1] = 0;
  solver.ConstraintMatrix->size[0] = 0;
  solver.ConstraintBound->size[0] = 0;
  solver.TimeObj.StartTime.tv_sec = 0.0;
  solver.TimeObj.StartTime.tv_nsec = 0.0;
  solver.TimeObjInternal.StartTime.tv_sec = 0.0;
  solver.TimeObjInternal.StartTime.tv_nsec = 0.0;
  solver.ConstraintsOn = true;
  solver.RandomRestart = false;
  solver.CostFcn.workspace.this_ = this_;
  emxInit_int8_T(&args_grads);
  i = args_grads->size[0] * args_grads->size[1];
  args_grads->size[1] = initGuess->size[1];
  args_grads->size[0] = 1;
  emxEnsureCapacity_int8_T(args_grads, i);
  args_grads_data = args_grads->data;
  input_sizes_idx_1 = initGuess->size[1];
  for (i = 0; i < input_sizes_idx_1; i++) {
    args_grads_data[i] = 0;
  }

  solver.ExtraArgs.cost = 0.0;
  i = solver.ExtraArgs.grads->size[0] * solver.ExtraArgs.grads->size[1];
  solver.ExtraArgs.grads->size[1] = args_grads->size[1];
  solver.ExtraArgs.grads->size[0] = 1;
  emxEnsureCapacity_real_T(solver.ExtraArgs.grads, i);
  input_sizes_idx_1 = args_grads->size[1];
  emxFree_int8_T(&args_grads);
  for (i = 0; i < input_sizes_idx_1; i++) {
    solver.ExtraArgs.grads->data[i] = 0.0;
  }

  solver.RandomSeedFcn.workspace.this_ = this_;
  solver.GradientFcn.workspace.this_ = this_;
  solver.SolutionEvaluationFcn.workspace.this_ = this_;
  sizes_idx_0_tmp = (this_->minSegmentTime->size[1] == 1);
  emxInit_int8_T(&A1);
  emxInit_real_T(&b1, 2);
  emxInitMatrix_cell_wrap_1(reshapes);
  emxInit_int8_T(&b_I);
  if (sizes_idx_0_tmp) {
    signed char input_sizes_idx_0;
    m = initGuess->size[1];
    i = b_I->size[0] * b_I->size[1];
    b_I->size[1] = initGuess->size[1];
    b_I->size[0] = initGuess->size[1];
    emxEnsureCapacity_int8_T(b_I, i);
    I_data = b_I->data;
    input_sizes_idx_1 = initGuess->size[1];
    for (i = 0; i < input_sizes_idx_1; i++) {
      k = initGuess->size[1];
      for (i1 = 0; i1 < k; i1++) {
        I_data[i1 + b_I->size[1] * i] = 0;
      }
    }

    if (initGuess->size[1] > 0) {
      for (k = 0; k < m; k++) {
        I_data[k + b_I->size[1] * k] = 1;
      }
    }

    emxInit_int8_T(&A11);
    i = A11->size[0] * A11->size[1];
    A11->size[1] = b_I->size[1];
    A11->size[0] = b_I->size[0];
    emxEnsureCapacity_int8_T(A11, i);
    A11_data = A11->data;
    input_sizes_idx_1 = b_I->size[0];
    for (i = 0; i < input_sizes_idx_1; i++) {
      k = b_I->size[1];
      for (i1 = 0; i1 < k; i1++) {
        A11_data[i1 + A11->size[1] * i] = (signed char)-I_data[i1 + b_I->size[1]
          * i];
      }
    }

    sizes_idx_0_tmp = ((b_I->size[0] != 0) && (b_I->size[1] != 0));
    if (sizes_idx_0_tmp) {
      m = A11->size[0];
      input_sizes_idx_1 = A11->size[1];
    } else {
      m = initGuess->size[1];
      input_sizes_idx_1 = 0;
    }

    if ((A11->size[1] == input_sizes_idx_1) && (m == A11->size[0])) {
      i = reshapes[0].f1->size[0] * reshapes[0].f1->size[1];
      reshapes[0].f1->size[1] = input_sizes_idx_1;
      reshapes[0].f1->size[0] = m;
      emxEnsureCapacity_real_T(reshapes[0].f1, i);
      for (i = 0; i < m; i++) {
        for (i1 = 0; i1 < input_sizes_idx_1; i1++) {
          reshapes[0].f1->data[i1 + reshapes[0].f1->size[1] * i] = A11_data[i1 +
            input_sizes_idx_1 * i];
        }
      }
    } else {
      i = 0;
      i1 = 0;
      i2 = 0;
      i3 = 0;
      i4 = reshapes[0].f1->size[0] * reshapes[0].f1->size[1];
      reshapes[0].f1->size[1] = input_sizes_idx_1;
      reshapes[0].f1->size[0] = m;
      emxEnsureCapacity_real_T(reshapes[0].f1, i4);
      for (i4 = 0; i4 < m * input_sizes_idx_1; i4++) {
        reshapes[0].f1->data[i1 + reshapes[0].f1->size[1] * i] = A11_data[i3 +
          A11->size[1] * i2];
        i++;
        i2++;
        if (i > reshapes[0].f1->size[0] - 1) {
          i = 0;
          i1++;
        }

        if (i2 > A11->size[0] - 1) {
          i2 = 0;
          i3++;
        }
      }
    }

    emxFree_int8_T(&A11);
    i = 0;
    i1 = 0;
    i2 = reshapes[1].f1->size[0] * reshapes[1].f1->size[1];
    reshapes[1].f1->size[1] = 1;
    reshapes[1].f1->size[0] = m;
    emxEnsureCapacity_real_T(reshapes[1].f1, i2);
    for (i2 = 0; i2 < m; i2++) {
      reshapes[1].f1->data[i1 + reshapes[1].f1->size[1] * i] = -1.0;
      i++;
      if (i > reshapes[1].f1->size[0] - 1) {
        i = 0;
        i1++;
      }
    }

    i = A1->size[0] * A1->size[1];
    A1->size[1] = reshapes[0].f1->size[1] + 1;
    A1->size[0] = reshapes[0].f1->size[0];
    emxEnsureCapacity_int8_T(A1, i);
    A1_data = A1->data;
    input_sizes_idx_1 = reshapes[0].f1->size[0];
    for (i = 0; i < input_sizes_idx_1; i++) {
      k = reshapes[0].f1->size[1];
      for (i1 = 0; i1 < k; i1++) {
        A1_data[i1 + A1->size[1] * i] = (signed char)reshapes[0].f1->data[i1 +
          reshapes[0].f1->size[1] * i];
      }
    }

    input_sizes_idx_1 = reshapes[1].f1->size[0];
    for (i = 0; i < input_sizes_idx_1; i++) {
      A1_data[reshapes[0].f1->size[1] + A1->size[1] * i] = -1;
    }

    emxInit_real_T(&b_b1, 2);
    i = b_b1->size[0] * b_b1->size[1];
    b_b1->size[1] = this_->minSegmentTime->size[1];
    b_b1->size[0] = 1;
    emxEnsureCapacity_real_T(b_b1, i);
    reshapes_f1_data = b_b1->data;
    input_sizes_idx_1 = this_->minSegmentTime->size[1];
    for (i = 0; i < input_sizes_idx_1; i++) {
      reshapes_f1_data[i] = this_->minSegmentTime->data[i];
    }

    input_sizes_idx_1 = initGuess->size[1];
    i = 0;
    i1 = 0;
    i2 = reshapes[0].f1->size[0] * reshapes[0].f1->size[1];
    reshapes[0].f1->size[1] = 1;
    reshapes[0].f1->size[0] = initGuess->size[1];
    emxEnsureCapacity_real_T(reshapes[0].f1, i2);
    for (i2 = 0; i2 < input_sizes_idx_1; i2++) {
      reshapes[0].f1->data[i1 + reshapes[0].f1->size[1] * i] = 0.0;
      i++;
      if (i > reshapes[0].f1->size[0] - 1) {
        i = 0;
        i1++;
      }
    }

    input_sizes_idx_0 = (signed char)(b_b1->size[1] != 0);
    input_sizes_idx_1 = (b_b1->size[1] != 0);
    if ((b_b1->size[1] == 1) && (input_sizes_idx_0 == 1)) {
      i = reshapes[1].f1->size[0] * reshapes[1].f1->size[1];
      reshapes[1].f1->size[1] = 1;
      reshapes[1].f1->size[0] = input_sizes_idx_1;
      emxEnsureCapacity_real_T(reshapes[1].f1, i);
      for (i = 0; i < input_sizes_idx_1; i++) {
        reshapes[1].f1->data[0] = reshapes_f1_data[0];
      }
    } else {
      i = reshapes[1].f1->size[0] * reshapes[1].f1->size[1];
      reshapes[1].f1->size[1] = 1;
      reshapes[1].f1->size[0] = input_sizes_idx_0;
      emxEnsureCapacity_real_T(reshapes[1].f1, i);
      for (i = 0; i < input_sizes_idx_1; i++) {
        reshapes[1].f1->data[0] = reshapes_f1_data[0];
      }
    }

    emxFree_real_T(&b_b1);
    i = b1->size[0] * b1->size[1];
    b1->size[1] = 1;
    b1->size[0] = reshapes[0].f1->size[0] + reshapes[1].f1->size[0];
    emxEnsureCapacity_real_T(b1, i);
    b1_data = b1->data;
    input_sizes_idx_1 = reshapes[0].f1->size[0];
    for (i = 0; i < input_sizes_idx_1; i++) {
      b1_data[b1->size[1] * i] = -0.0;
    }

    input_sizes_idx_1 = reshapes[1].f1->size[0];
    for (i = 0; i < input_sizes_idx_1; i++) {
      b1_data[b1->size[1] * reshapes[0].f1->size[0]] = -reshapes[1].f1->data[0];
    }
  } else {
    m = initGuess->size[1];
    i = b_I->size[0] * b_I->size[1];
    b_I->size[1] = initGuess->size[1];
    b_I->size[0] = initGuess->size[1];
    emxEnsureCapacity_int8_T(b_I, i);
    I_data = b_I->data;
    input_sizes_idx_1 = initGuess->size[1];
    for (i = 0; i < input_sizes_idx_1; i++) {
      k = initGuess->size[1];
      for (i1 = 0; i1 < k; i1++) {
        I_data[i1 + b_I->size[1] * i] = 0;
      }
    }

    if (initGuess->size[1] > 0) {
      for (k = 0; k < m; k++) {
        I_data[k + b_I->size[1] * k] = 1;
      }
    }

    i = A1->size[0] * A1->size[1];
    A1->size[1] = b_I->size[1];
    A1->size[0] = b_I->size[0];
    emxEnsureCapacity_int8_T(A1, i);
    A1_data = A1->data;
    input_sizes_idx_1 = b_I->size[0];
    for (i = 0; i < input_sizes_idx_1; i++) {
      k = b_I->size[1];
      for (i1 = 0; i1 < k; i1++) {
        A1_data[i1 + A1->size[1] * i] = (signed char)-I_data[i1 + b_I->size[1] *
          i];
      }
    }

    input_sizes_idx_1 = this_->minSegmentTime->size[1];
    emxInit_real_T(&b_b1, 1);
    i = b_b1->size[0];
    b_b1->size[0] = input_sizes_idx_1;
    emxEnsureCapacity_real_T(b_b1, i);
    reshapes_f1_data = b_b1->data;
    for (i = 0; i < input_sizes_idx_1; i++) {
      reshapes_f1_data[i] = -this_->minSegmentTime->data[i];
    }

    i = b1->size[0] * b1->size[1];
    b1->size[1] = 1;
    b1->size[0] = b_b1->size[0];
    emxEnsureCapacity_real_T(b1, i);
    b1_data = b1->data;
    input_sizes_idx_1 = b_b1->size[0];
    for (i = 0; i < input_sizes_idx_1; i++) {
      b1_data[b1->size[1] * i] = reshapes_f1_data[i];
    }

    emxFree_real_T(&b_b1);
  }

  emxFree_int8_T(&b_I);
  reshapes_f2 = this_->maxSegmentTime;
  sizes_idx_0_tmp = ((A1->size[0] != 0) && (A1->size[1] != 0));
  if (sizes_idx_0_tmp) {
    m = A1->size[0];
    k = A1->size[1];
  } else {
    m = initGuess->size[1];
    k = 0;
  }

  if ((A1->size[1] == k) && (m == A1->size[0])) {
    i = reshapes[0].f1->size[0] * reshapes[0].f1->size[1];
    reshapes[0].f1->size[1] = k;
    reshapes[0].f1->size[0] = m;
    emxEnsureCapacity_real_T(reshapes[0].f1, i);
    for (i = 0; i < m; i++) {
      for (i1 = 0; i1 < k; i1++) {
        reshapes[0].f1->data[i1 + reshapes[0].f1->size[1] * i] = A1_data[i1 + k *
          i];
      }
    }
  } else {
    i = 0;
    i1 = 0;
    i2 = 0;
    i3 = 0;
    i4 = reshapes[0].f1->size[0] * reshapes[0].f1->size[1];
    reshapes[0].f1->size[1] = k;
    reshapes[0].f1->size[0] = m;
    emxEnsureCapacity_real_T(reshapes[0].f1, i4);
    for (i4 = 0; i4 < m * k; i4++) {
      reshapes[0].f1->data[i1 + reshapes[0].f1->size[1] * i] = A1_data[i3 +
        A1->size[1] * i2];
      i++;
      i2++;
      if (i > reshapes[0].f1->size[0] - 1) {
        i = 0;
        i1++;
      }

      if (i2 > A1->size[0] - 1) {
        i2 = 0;
        i3++;
      }
    }
  }

  emxFree_int8_T(&A1);
  i = 0;
  i1 = 0;
  i2 = reshapes[1].f1->size[0] * reshapes[1].f1->size[1];
  reshapes[1].f1->size[1] = 1;
  reshapes[1].f1->size[0] = m;
  emxEnsureCapacity_real_T(reshapes[1].f1, i2);
  for (i2 = 0; i2 < m; i2++) {
    reshapes[1].f1->data[i1 + reshapes[1].f1->size[1] * i] = 1.0;
    i++;
    if (i > reshapes[1].f1->size[0] - 1) {
      i = 0;
      i1++;
    }
  }

  i = solver.ConstraintMatrix->size[0] * solver.ConstraintMatrix->size[1];
  solver.ConstraintMatrix->size[1] = reshapes[0].f1->size[1] + 1;
  solver.ConstraintMatrix->size[0] = reshapes[0].f1->size[0];
  emxEnsureCapacity_real_T(solver.ConstraintMatrix, i);
  input_sizes_idx_1 = reshapes[0].f1->size[0];
  for (i = 0; i < input_sizes_idx_1; i++) {
    k = reshapes[0].f1->size[1];
    for (i1 = 0; i1 < k; i1++) {
      solver.ConstraintMatrix->data[i1 + solver.ConstraintMatrix->size[1] * i] =
        reshapes[0].f1->data[i1 + reshapes[0].f1->size[1] * i];
    }
  }

  input_sizes_idx_1 = reshapes[1].f1->size[0];
  for (i = 0; i < input_sizes_idx_1; i++) {
    solver.ConstraintMatrix->data[reshapes[0].f1->size[1] +
      solver.ConstraintMatrix->size[1] * i] = 1.0;
  }

  emxFreeMatrix_cell_wrap_1(reshapes);
  if (b1->size[0] != 0) {
    input_sizes_idx_1 = b1->size[0];
  } else {
    input_sizes_idx_1 = 0;
  }

  i = 0;
  i1 = 0;
  emxInit_real_T(&reshapes_f1, 1);
  i2 = reshapes_f1->size[0];
  reshapes_f1->size[0] = input_sizes_idx_1;
  emxEnsureCapacity_real_T(reshapes_f1, i2);
  reshapes_f1_data = reshapes_f1->data;
  for (i2 = 0; i2 < input_sizes_idx_1; i2++) {
    reshapes_f1_data[i2] = b1_data[i1 + b1->size[1] * i];
    i++;
    if (i > b1->size[0] - 1) {
      i = 0;
      i1++;
    }
  }

  emxFree_real_T(&b1);
  i = solver.ConstraintBound->size[0];
  solver.ConstraintBound->size[0] = reshapes_f1->size[0] + 1;
  emxEnsureCapacity_real_T(solver.ConstraintBound, i);
  input_sizes_idx_1 = reshapes_f1->size[0];
  for (i = 0; i < input_sizes_idx_1; i++) {
    solver.ConstraintBound->data[i] = reshapes_f1_data[i];
  }

  solver.ConstraintBound->data[reshapes_f1->size[0]] = reshapes_f2;
  emxFree_real_T(&reshapes_f1);
  solver.MaxNumIterationInternal = solver.MaxNumIteration;
  solver.MaxTimeInternal = solver.MaxTime;
  i = solver.SeedInternal->size[0];
  solver.SeedInternal->size[0] = initGuess->size[1];
  emxEnsureCapacity_real_T(solver.SeedInternal, i);
  input_sizes_idx_1 = initGuess->size[1];
  for (i = 0; i < input_sizes_idx_1; i++) {
    solver.SeedInternal->data[i] = initGuess_data[i];
  }

  tol = solver.SolutionTolerance;
  solver.TimeObj.StartTime.tv_sec = tic(&solver.TimeObj.StartTime.tv_nsec);
  exitFlag = c_DampedBFGSwGradientProjection(&solver, t, &err, &iter);
  rrAttempts = 0.0;
  iterations = iter;
  errPrev = err;
  *exitstruct_ExitFlag = exitFlag;
  emxInit_real_T(&xSol, 1);
  exitg1 = false;
  while ((!exitg1) && (solver.RandomRestart && (err > tol))) {
    solver.MaxNumIterationInternal -= iter;
    reshapes_f2 = toc(solver.TimeObj.StartTime.tv_sec,
                      solver.TimeObj.StartTime.tv_nsec);
    solver.MaxTimeInternal = solver.MaxTime - reshapes_f2;
    if (solver.MaxNumIterationInternal <= 0.0) {
      exitFlag = 5.0;
    }

    if ((fabs(exitFlag - 5.0) < DBL_EPSILON) || (fabs(exitFlag - 1.0) < DBL_EPSILON)) {
      *exitstruct_ExitFlag = exitFlag;
      exitg1 = true;
    } else {
      input_sizes_idx_1 = solver.SeedInternal->size[0];
      i = solver.SeedInternal->size[0];
      solver.SeedInternal->size[0] = input_sizes_idx_1;
      emxEnsureCapacity_real_T(solver.SeedInternal, i);
      for (i = 0; i < input_sizes_idx_1; i++) {
        solver.SeedInternal->data[i] = 0.1;
      }

      exitFlag = c_DampedBFGSwGradientProjection(&solver, xSol, &err, &iter);
      xSol_data = xSol->data;
      if (err < errPrev) {
        i = t->size[0];
        t->size[0] = xSol->size[0];
        emxEnsureCapacity_real_T(t, i);
        t_data = t->data;
        input_sizes_idx_1 = xSol->size[0];
        for (i = 0; i < input_sizes_idx_1; i++) {
          t_data[i] = xSol_data[i];
        }

        errPrev = err;
        *exitstruct_ExitFlag = exitFlag;
      }

      rrAttempts++;
      iterations += iter;
    }
  }

  emxFree_real_T(&xSol);
  if (errPrev < tol) {
    exitstruct_Status_size[1] = 7;
    exitstruct_Status_size[0] = 1;
    for (i = 0; i < 7; i++) {
      exitstruct_Status_data[i] = cv1[i];
    }
  } else {
    exitstruct_Status_size[1] = 14;
    exitstruct_Status_size[0] = 1;
    for (i = 0; i < 14; i++) {
      exitstruct_Status_data[i] = cv[i];
    }
  }

  J = solver_common_solvePoly(this_, t, p);
  *exitstruct_Iterations = iterations;
  *exitstruct_RRAttempts = rrAttempts;
  *exitstruct_Error = errPrev;
  d_emxFreeStruct_TrajectoryOptim(&solver);
  return J;
}

static void solver_common_optimize_anonFcn3(c_TrajectoryOptimizer_solver_me
  *this, const emxArray_real_T *varargin_1, emxArray_real_T *varargout_1)
{
  emxArray_real_T *a__5;
  emxArray_real_T *a__6;
  emxArray_real_T *deltaTN;
  emxArray_real_T *deltaTP;
  emxArray_real_T *deltavec;
  const double *varargin_1_data;
  double *deltaTN_data;
  double *deltaTP_data;
  double *deltavec_data;
  double *varargout_1_data;
  int i;
  int i1;
  int kk;
  int this_idx_0_tmp_tmp;
  varargin_1_data = varargin_1->data;
  this_idx_0_tmp_tmp = (int)this->N_segments;
  i = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1] = this_idx_0_tmp_tmp;
  varargout_1->size[0] = 1;
  emxEnsureCapacity_real_T(varargout_1, i);
  varargout_1_data = varargout_1->data;
  for (i = 0; i < this_idx_0_tmp_tmp; i++) {
    varargout_1_data[i] = 0.0;
  }

  i = varargin_1->size[0];
  emxInit_real_T(&deltavec, 1);
  emxInit_real_T(&deltaTP, 1);
  emxInit_real_T(&deltaTN, 1);
  emxInit_real_T(&a__5, 3);
  emxInit_real_T(&a__6, 3);
  for (kk = 0; kk < i; kk++) {
    double er1;
    double er2;
    this_idx_0_tmp_tmp = (int)this->N_segments;
    i1 = deltavec->size[0];
    deltavec->size[0] = this_idx_0_tmp_tmp;
    emxEnsureCapacity_real_T(deltavec, i1);
    deltavec_data = deltavec->data;
    for (i1 = 0; i1 < this_idx_0_tmp_tmp; i1++) {
      deltavec_data[i1] = 0.0;
    }

    deltavec_data[kk] = 1.0E-5;
    this_idx_0_tmp_tmp = varargin_1->size[0];
    if (varargin_1->size[0] == deltavec->size[0]) {
      i1 = deltaTP->size[0];
      deltaTP->size[0] = varargin_1->size[0];
      emxEnsureCapacity_real_T(deltaTP, i1);
      deltaTP_data = deltaTP->data;
      for (i1 = 0; i1 < this_idx_0_tmp_tmp; i1++) {
        deltaTP_data[i1] = varargin_1_data[i1] + deltavec_data[i1];
      }
    } else {
      b_plus(deltaTP, varargin_1, deltavec);
    }

    this_idx_0_tmp_tmp = varargin_1->size[0];
    if (varargin_1->size[0] == deltavec->size[0]) {
      i1 = deltaTN->size[0];
      deltaTN->size[0] = varargin_1->size[0];
      emxEnsureCapacity_real_T(deltaTN, i1);
      deltaTN_data = deltaTN->data;
      for (i1 = 0; i1 < this_idx_0_tmp_tmp; i1++) {
        deltaTN_data[i1] = varargin_1_data[i1] - deltavec_data[i1];
      }
    } else {
      c_minus(deltaTN, varargin_1, deltavec);
    }

    er1 = solver_common_solvePoly(this, deltaTP, a__5);
    er2 = solver_common_solvePoly(this, deltaTN, a__6);
    varargout_1_data[kk] = ((er1 + this->timeWt * sum(deltaTP)) - (er2 +
      this->timeWt * sum(deltaTN))) * 49999.999999999993;
  }

  emxFree_real_T(&a__6);
  emxFree_real_T(&a__5);
  emxFree_real_T(&deltaTN);
  emxFree_real_T(&deltaTP);
  emxFree_real_T(&deltavec);
}

static double solver_common_solvePoly(c_TrajectoryOptimizer_solver_me *this,
  const emxArray_real_T *T, emxArray_real_T *p)
{
  b_sparse b_expl_temp;
  b_sparse c_expl_temp;
  emxArray_boolean_T *fixed_constr_ids_tmp;
  emxArray_boolean_T *nontriv_dim;
  emxArray_int32_T *M_colidx;
  emxArray_int32_T *M_rowidx;
  emxArray_int32_T *RPF_colidx;
  emxArray_int32_T *RPF_rowidx;
  emxArray_int32_T *RPP_colidx;
  emxArray_int32_T *RPP_rowidx;
  emxArray_int32_T *R_colidx;
  emxArray_int32_T *R_rowidx;
  emxArray_int32_T *ii;
  emxArray_int32_T *jj;
  emxArray_int32_T *r1;
  emxArray_real_T *D;
  emxArray_real_T *DP;
  emxArray_real_T *M_d;
  emxArray_real_T *RPF_d;
  emxArray_real_T *RPP_d;
  emxArray_real_T *R_d;
  emxArray_real_T *b_this;
  emxArray_real_T *b_waypoints;
  emxArray_real_T *c_p;
  emxArray_real_T *d_p;
  emxArray_real_T *e_p;
  emxArray_real_T *r;
  emxArray_real_T *r2;
  emxArray_real_T *tmp_bcs;
  emxArray_real_T *tmp_i_col;
  emxArray_real_T *tmp_i_row;
  emxArray_real_T *waypoints;
  emxArray_real_T *y;
  sparse A_total_sp;
  sparse Q_prime_total_sp;
  sparse d_expl_temp;
  const double *T_data;
  double J;
  double *DP_data;
  double *D_data;
  double *RPP_d_data;
  double *R_d_data;
  double *b_p_data;
  double *b_waypoints_data;
  double *c_p_data;
  double *p_data;
  double *tmp_bcs_data;
  double *tmp_i_col_data;
  double *tmp_i_row_data;
  double *waypoints_data;
  int M_n;
  int RPF_n;
  int RPP_n;
  int R_n;
  int b_ii;
  int dimIdx;
  int expl_temp;
  int i;
  int i_dim;
  int k;
  int loop_ub;
  int nz;
  int segment;
  int this_idx_0_tmp;
  int vlen;
  int *R_colidx_data;
  int *R_rowidx_data;
  int *ii_data;
  int *jj_data;
  boolean_T *fixed_constr_ids_tmp_data;
  boolean_T *nontriv_dim_data;
  T_data = T->data;
  vlen = this->wptFnc.workspace.wpts->size[0];
  i = this->waypoints_offset->size[0];
  this->waypoints_offset->size[0] = vlen;
  emxEnsureCapacity_real_T(this->waypoints_offset, i);
  for (i = 0; i < vlen; i++) {
    this->waypoints_offset->data[i] = this->wptFnc.workspace.wpts->data
      [this->wptFnc.workspace.wpts->size[1] * i];
  }

  emxInit_real_T(&waypoints, 2);
  if (this->waypoints_offset->size[0] == this->wptFnc.workspace.wpts->size[0]) {
    i = waypoints->size[0] * waypoints->size[1];
    waypoints->size[1] = this->wptFnc.workspace.wpts->size[1];
    waypoints->size[0] = this->wptFnc.workspace.wpts->size[0];
    emxEnsureCapacity_real_T(waypoints, i);
    waypoints_data = waypoints->data;
    loop_ub = this->wptFnc.workspace.wpts->size[0];
    for (i = 0; i < loop_ub; i++) {
      vlen = this->wptFnc.workspace.wpts->size[1];
      for (nz = 0; nz < vlen; nz++) {
        waypoints_data[nz + waypoints->size[1] * i] =
          this->wptFnc.workspace.wpts->data[nz + this->
          wptFnc.workspace.wpts->size[1] * i] - this->waypoints_offset->data[i];
      }
    }
  } else {
    j_binary_expand_op(waypoints, this);
    waypoints_data = waypoints->data;
  }

  this->n_dim_src = waypoints->size[0];
  this->N_wps = waypoints->size[1];
  this->N_segments = this->N_wps - 1.0;
  this->stateSize = 10.0 * this->N_segments;
  this_idx_0_tmp = (int)this->n_dim_src;
  emxInit_boolean_T(&nontriv_dim);
  i = nontriv_dim->size[0];
  nontriv_dim->size[0] = this_idx_0_tmp;
  emxEnsureCapacity_boolean_T(nontriv_dim, i);
  nontriv_dim_data = nontriv_dim->data;
  for (i = 0; i < this_idx_0_tmp; i++) {
    nontriv_dim_data[i] = false;
  }

  if (this_idx_0_tmp - 1 >= 0) {
    k = waypoints->size[1];
    expl_temp = waypoints->size[1];
  }

  emxInit_real_T(&b_waypoints, 2);
  for (i_dim = 0; i_dim < this_idx_0_tmp; i_dim++) {
    i = b_waypoints->size[0] * b_waypoints->size[1];
    b_waypoints->size[1] = k;
    b_waypoints->size[0] = 1;
    emxEnsureCapacity_real_T(b_waypoints, i);
    b_waypoints_data = b_waypoints->data;
    for (i = 0; i < expl_temp; i++) {
      b_waypoints_data[i] = waypoints_data[i + waypoints->size[1] * i_dim];
    }

    nontriv_dim_data[i_dim] = any(b_waypoints);
  }

  expl_temp = 0;
  i = nontriv_dim->size[0];
  for (k = 0; k < i; k++) {
    if (nontriv_dim_data[k]) {
      expl_temp++;
    }
  }

  emxInit_int32_T(&ii, 1);
  eml_find(nontriv_dim, ii);
  ii_data = ii->data;
  i = this->n_dim_ids->size[0];
  this->n_dim_ids->size[0] = ii->size[0];
  emxEnsureCapacity_real_T(this->n_dim_ids, i);
  loop_ub = ii->size[0];
  for (i = 0; i < loop_ub; i++) {
    this->n_dim_ids->data[i] = ii_data[i];
  }

  this->n_dim = expl_temp;
  k = nontriv_dim->size[0] - 1;
  vlen = 0;
  for (loop_ub = 0; loop_ub <= k; loop_ub++) {
    if (nontriv_dim_data[loop_ub]) {
      vlen++;
    }
  }

  emxInit_int32_T(&jj, 1);
  i = jj->size[0];
  jj->size[0] = vlen;
  emxEnsureCapacity_int32_T(jj, i);
  jj_data = jj->data;
  vlen = 0;
  for (loop_ub = 0; loop_ub <= k; loop_ub++) {
    if (nontriv_dim_data[loop_ub]) {
      jj_data[vlen] = loop_ub;
      vlen++;
    }
  }

  emxFree_boolean_T(&nontriv_dim);
  i = this->waypoints->size[0] * this->waypoints->size[1];
  this->waypoints->size[1] = waypoints->size[1];
  this->waypoints->size[0] = jj->size[0];
  emxEnsureCapacity_real_T(this->waypoints, i);
  loop_ub = jj->size[0];
  for (i = 0; i < loop_ub; i++) {
    vlen = waypoints->size[1];
    for (nz = 0; nz < vlen; nz++) {
      this->waypoints->data[nz + this->waypoints->size[1] * i] =
        waypoints_data[nz + waypoints->size[1] * jj_data[i]];
    }
  }

  emxFree_real_T(&waypoints);
  vlen = (int)(5.0 * this->N_wps);
  i = this->constraints->size[0] * this->constraints->size[1];
  this->constraints->size[1] = expl_temp;
  this->constraints->size[0] = vlen;
  emxEnsureCapacity_real_T(this->constraints, i);
  for (i = 0; i < vlen; i++) {
    for (nz = 0; nz < expl_temp; nz++) {
      this->constraints->data[nz + this->constraints->size[1] * i] = 0.0;
    }
  }

  if (expl_temp > 0) {
    i = (int)this->N_wps;
    emxInit_real_T(&tmp_bcs, 2);
    for (k = 0; k < i; k++) {
      boolean_T b_p;
      vlen = (int)this->n_dim;
      nz = tmp_bcs->size[0] * tmp_bcs->size[1];
      tmp_bcs->size[1] = 5;
      tmp_bcs->size[0] = vlen;
      emxEnsureCapacity_real_T(tmp_bcs, nz);
      tmp_bcs_data = tmp_bcs->data;
      for (nz = 0; nz < vlen; nz++) {
        for (expl_temp = 0; expl_temp < 5; expl_temp++) {
          tmp_bcs_data[expl_temp + 5 * nz] = 0.0;
        }
      }

      loop_ub = this->waypoints->size[0];
      for (nz = 0; nz < loop_ub; nz++) {
        tmp_bcs_data[5 * nz] = this->waypoints->data[k + this->waypoints->size[1]
          * nz];
      }

      b_p = ((unsigned int)k + 1U == 1U);
      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        if ((!b_p) && (!(fabs((double)k + 1.0 - this->N_wps) < DBL_EPSILON))) {
          for (nz = 0; nz < vlen; nz++) {
            tmp_bcs_data[(loop_ub + 5 * nz) + 1] = rtNaN;
          }
        }
      }

      for (loop_ub = 0; loop_ub < vlen; loop_ub++) {
        for (b_ii = 0; b_ii < 5; b_ii++) {
          this->constraints->data[loop_ub + this->constraints->size[1] * ((int)
            (((double)b_ii + 1.0) + (((double)k + 1.0) - 1.0) * 5.0) - 1)] =
            tmp_bcs_data[b_ii + 5 * loop_ub];
        }
      }
    }

    emxFree_real_T(&tmp_bcs);
  }

  this_idx_0_tmp = (int)this->n_dim;
  vlen = (int)this->N_segments;
  i = p->size[0] * p->size[1] * p->size[2];
  p->size[2] = this_idx_0_tmp;
  p->size[1] = 10;
  p->size[0] = vlen;
  emxEnsureCapacity_real_T(p, i);
  p_data = p->data;
  for (i = 0; i < vlen; i++) {
    for (nz = 0; nz < 10; nz++) {
      for (expl_temp = 0; expl_temp < this_idx_0_tmp; expl_temp++) {
        p_data[(expl_temp + p->size[2] * nz) + p->size[2] * 10 * i] = 0.0;
      }
    }
  }

  J = 0.0;
  emxInit_real_T(&r, 2);
  emxInitStruct_sparse(&A_total_sp);
  emxInitStruct_sparse(&Q_prime_total_sp);
  emxInit_real_T(&M_d, 1);
  emxInit_int32_T(&M_colidx, 1);
  emxInit_int32_T(&M_rowidx, 1);
  emxInit_real_T(&R_d, 1);
  emxInit_int32_T(&R_colidx, 1);
  emxInit_int32_T(&R_rowidx, 1);
  emxInit_real_T(&RPP_d, 1);
  emxInit_int32_T(&RPP_colidx, 1);
  emxInit_int32_T(&RPP_rowidx, 1);
  emxInit_real_T(&RPF_d, 1);
  emxInit_int32_T(&RPF_colidx, 1);
  emxInit_int32_T(&RPF_rowidx, 1);
  emxInit_real_T(&DP, 1);
  emxInit_real_T(&D, 1);
  emxInit_real_T(&tmp_i_row, 1);
  emxInit_real_T(&tmp_i_col, 1);
  emxInit_real_T(&c_p, 1);
  emxInit_real_T(&d_p, 2);
  emxInit_int32_T(&r1, 1);
  emxInit_boolean_T(&fixed_constr_ids_tmp);
  emxInit_real_T(&y, 2);
  emxInitStruct_sparse1(&b_expl_temp);
  emxInitStruct_sparse1(&c_expl_temp);
  emxInitStruct_sparse(&d_expl_temp);
  emxInit_real_T(&b_this, 1);
  emxInit_real_T(&e_p, 2);
  emxInit_real_T(&r2, 1);
  for (dimIdx = 0; dimIdx < this_idx_0_tmp; dimIdx++) {
    double cd;
    double numSegments;
    int M_m;
    int RPF_m;
    int RPP_m;
    int R_m;
    int b_y;
    int unnamed_idx_1;
    vlen = this->constraints->size[0];
    i = fixed_constr_ids_tmp->size[0];
    fixed_constr_ids_tmp->size[0] = vlen;
    emxEnsureCapacity_boolean_T(fixed_constr_ids_tmp, i);
    fixed_constr_ids_tmp_data = fixed_constr_ids_tmp->data;
    for (i = 0; i < vlen; i++) {
      fixed_constr_ids_tmp_data[i] = !rtIsNaN(this->constraints->data[dimIdx +
        this->constraints->size[1] * i]);
    }

    vlen = fixed_constr_ids_tmp->size[0];
    if (fixed_constr_ids_tmp->size[0] == 0) {
      nz = 0;
    } else {
      nz = fixed_constr_ids_tmp_data[0];
      for (k = 2; k <= vlen; k++) {
        if (vlen >= 2) {
          nz += fixed_constr_ids_tmp_data[k - 1];
        }
      }
    }

    vlen = fixed_constr_ids_tmp->size[0];
    if (fixed_constr_ids_tmp->size[0] == 0) {
      b_y = 0;
    } else {
      b_y = fixed_constr_ids_tmp_data[0];
      for (k = 2; k <= vlen; k++) {
        if (vlen >= 2) {
          b_y += fixed_constr_ids_tmp_data[k - 1];
        }
      }
    }

    numSegments = this->N_segments;
    spalloc(10.0 * numSegments, 10.0 * numSegments, 100.0 * numSegments,
            &A_total_sp);
    spalloc(10.0 * numSegments, 10.0 * numSegments, 100.0 * numSegments,
            &Q_prime_total_sp);
    unnamed_idx_1 = (int)numSegments;
    for (segment = 0; segment < unnamed_idx_1; segment++) {
      double offset;
      offset = (((double)segment + 1.0) - 1.0) * 10.0;
      get_A(T_data[segment], &b_expl_temp);
      get_Q_prime(T_data[segment], &c_expl_temp);
      b_eml_find(b_expl_temp.colidx, b_expl_temp.rowidx, ii, jj);
      jj_data = jj->data;
      ii_data = ii->data;
      loop_ub = ii->size[0];
      i = tmp_i_row->size[0];
      tmp_i_row->size[0] = ii->size[0];
      emxEnsureCapacity_real_T(tmp_i_row, i);
      tmp_i_row_data = tmp_i_row->data;
      for (i = 0; i < loop_ub; i++) {
        tmp_i_row_data[i] = ii_data[i];
      }

      loop_ub = jj->size[0];
      i = tmp_i_col->size[0];
      tmp_i_col->size[0] = jj->size[0];
      emxEnsureCapacity_real_T(tmp_i_col, i);
      tmp_i_col_data = tmp_i_col->data;
      for (i = 0; i < loop_ub; i++) {
        tmp_i_col_data[i] = jj_data[i];
      }

      vlen = sparse_parenReference(b_expl_temp.d, b_expl_temp.colidx,
        b_expl_temp.rowidx, tmp_i_row, tmp_i_col, d_expl_temp.d,
        d_expl_temp.colidx, d_expl_temp.rowidx, &k, &expl_temp);
      loop_ub = tmp_i_row->size[0];
      for (i = 0; i < loop_ub; i++) {
        tmp_i_row_data[i] += offset;
      }

      loop_ub = tmp_i_col->size[0];
      for (i = 0; i < loop_ub; i++) {
        tmp_i_col_data[i] += offset;
      }

      sparse_parenAssign(&A_total_sp, d_expl_temp.d, d_expl_temp.colidx,
                         d_expl_temp.rowidx, vlen, tmp_i_row, tmp_i_col);
      b_eml_find(c_expl_temp.colidx, c_expl_temp.rowidx, ii, jj);
      jj_data = jj->data;
      ii_data = ii->data;
      loop_ub = ii->size[0];
      i = tmp_i_row->size[0];
      tmp_i_row->size[0] = ii->size[0];
      emxEnsureCapacity_real_T(tmp_i_row, i);
      tmp_i_row_data = tmp_i_row->data;
      for (i = 0; i < loop_ub; i++) {
        tmp_i_row_data[i] = ii_data[i];
      }

      loop_ub = jj->size[0];
      i = tmp_i_col->size[0];
      tmp_i_col->size[0] = jj->size[0];
      emxEnsureCapacity_real_T(tmp_i_col, i);
      tmp_i_col_data = tmp_i_col->data;
      for (i = 0; i < loop_ub; i++) {
        tmp_i_col_data[i] = jj_data[i];
      }

      vlen = sparse_parenReference(c_expl_temp.d, c_expl_temp.colidx,
        c_expl_temp.rowidx, tmp_i_row, tmp_i_col, d_expl_temp.d,
        d_expl_temp.colidx, d_expl_temp.rowidx, &k, &expl_temp);
      loop_ub = tmp_i_row->size[0];
      for (i = 0; i < loop_ub; i++) {
        tmp_i_row_data[i] += offset;
      }

      loop_ub = tmp_i_col->size[0];
      for (i = 0; i < loop_ub; i++) {
        tmp_i_col_data[i] += offset;
      }

      sparse_parenAssign(&Q_prime_total_sp, d_expl_temp.d, d_expl_temp.colidx,
                         d_expl_temp.rowidx, vlen, tmp_i_row, tmp_i_col);
    }

    vlen = this->constraints->size[0];
    i = b_this->size[0];
    b_this->size[0] = vlen;
    emxEnsureCapacity_real_T(b_this, i);
    b_waypoints_data = b_this->data;
    for (i = 0; i < vlen; i++) {
      b_waypoints_data[i] = this->constraints->data[dimIdx + this->
        constraints->size[1] * i];
    }

    M_m = solver_common_constructM(this, b_this, M_d, M_colidx, M_rowidx, &M_n);
    sparse_ctranspose(M_d, M_colidx, M_rowidx, M_m, M_n, &d_expl_temp);
    vlen = sparse_mtimes(d_expl_temp.d, d_expl_temp.colidx, d_expl_temp.rowidx,
                         d_expl_temp.m, Q_prime_total_sp.d,
                         Q_prime_total_sp.colidx, Q_prime_total_sp.rowidx,
                         Q_prime_total_sp.n, b_this, jj, ii, &k, &expl_temp);
    R_m = sparse_mtimes(b_this, jj, ii, vlen, M_d, M_colidx, M_rowidx, M_n, R_d,
                        R_colidx, R_rowidx, &R_n, &k);
    R_rowidx_data = R_rowidx->data;
    R_colidx_data = R_colidx->data;
    R_d_data = R_d->data;
    k = fixed_constr_ids_tmp->size[0] - 1;
    vlen = 0;
    for (loop_ub = 0; loop_ub <= k; loop_ub++) {
      if (fixed_constr_ids_tmp_data[loop_ub]) {
        vlen++;
      }
    }

    i = r1->size[0];
    r1->size[0] = vlen;
    emxEnsureCapacity_int32_T(r1, i);
    ii_data = r1->data;
    vlen = 0;
    for (loop_ub = 0; loop_ub <= k; loop_ub++) {
      if (fixed_constr_ids_tmp_data[loop_ub]) {
        ii_data[vlen] = loop_ub;
        vlen++;
      }
    }

    if (R_m < (double)b_y + 1.0) {
      b_waypoints->size[1] = 0;
      b_waypoints->size[0] = 1;
    } else {
      k = R_m - nz;
      i = b_waypoints->size[0] * b_waypoints->size[1];
      b_waypoints->size[1] = k;
      b_waypoints->size[0] = 1;
      emxEnsureCapacity_real_T(b_waypoints, i);
      b_waypoints_data = b_waypoints->data;
      for (i = 0; i < k; i++) {
        b_waypoints_data[i] = ((double)nz + 1.0) + (double)i;
      }
    }

    if (R_n < (double)b_y + 1.0) {
      y->size[1] = 0;
      y->size[0] = 1;
    } else {
      k = R_n - nz;
      i = y->size[0] * y->size[1];
      y->size[1] = k;
      y->size[0] = 1;
      emxEnsureCapacity_real_T(y, i);
      b_waypoints_data = y->data;
      for (i = 0; i < k; i++) {
        b_waypoints_data[i] = ((double)nz + 1.0) + (double)i;
      }
    }

    RPP_m = b_sparse_parenReference(R_d, R_colidx, R_rowidx, b_waypoints, y,
      RPP_d, RPP_colidx, RPP_rowidx, &RPP_n, &k);
    RPP_d_data = RPP_d->data;
    if (R_m < (double)b_y + 1.0) {
      b_waypoints->size[1] = 0;
      b_waypoints->size[0] = 1;
    } else {
      k = R_m - nz;
      i = b_waypoints->size[0] * b_waypoints->size[1];
      b_waypoints->size[1] = k;
      b_waypoints->size[0] = 1;
      emxEnsureCapacity_real_T(b_waypoints, i);
      b_waypoints_data = b_waypoints->data;
      for (i = 0; i < k; i++) {
        b_waypoints_data[i] = ((double)nz + 1.0) + (double)i;
      }
    }

    if (nz < 1) {
      y->size[1] = 0;
      y->size[0] = 1;
    } else {
      i = y->size[0] * y->size[1];
      y->size[1] = b_y;
      y->size[0] = 1;
      emxEnsureCapacity_real_T(y, i);
      b_waypoints_data = y->data;
      for (i = 0; i < b_y; i++) {
        b_waypoints_data[i] = (double)i + 1.0;
      }
    }

    RPF_m = b_sparse_parenReference(R_d, R_colidx, R_rowidx, b_waypoints, y,
      RPF_d, RPF_colidx, RPF_rowidx, &RPF_n, &k);
    loop_ub = RPP_d->size[0];
    for (i = 0; i < loop_ub; i++) {
      RPP_d_data[i] = -RPP_d_data[i];
    }

    loop_ub = r1->size[0];
    i = b_this->size[0];
    b_this->size[0] = r1->size[0];
    emxEnsureCapacity_real_T(b_this, i);
    b_waypoints_data = b_this->data;
    for (i = 0; i < loop_ub; i++) {
      b_waypoints_data[i] = this->constraints->data[dimIdx + this->
        constraints->size[1] * ii_data[i]];
    }

    b_sparse_mtimes(RPF_d, RPF_colidx, RPF_rowidx, RPF_m, RPF_n, b_this, r2);
    sparse_mldivide(RPP_d, RPP_colidx, RPP_rowidx, RPP_m, RPP_n, r2, DP);
    DP_data = DP->data;
    loop_ub = r1->size[0];
    i = D->size[0];
    D->size[0] = r1->size[0] + DP->size[0];
    emxEnsureCapacity_real_T(D, i);
    D_data = D->data;
    for (i = 0; i < loop_ub; i++) {
      D_data[i] = this->constraints->data[dimIdx + this->constraints->size[1] *
        ii_data[i]];
    }

    loop_ub = DP->size[0];
    for (i = 0; i < loop_ub; i++) {
      D_data[i + r1->size[0]] = DP_data[i];
    }

    b_sparse_mtimes(M_d, M_colidx, M_rowidx, M_m, M_n, D, r2);
    sparse_mldivide(A_total_sp.d, A_total_sp.colidx, A_total_sp.rowidx,
                    A_total_sp.m, A_total_sp.n, r2, c_p);
    b_p_data = c_p->data;
    i = 0;
    nz = 0;
    expl_temp = d_p->size[0] * d_p->size[1];
    d_p->size[1] = (int)numSegments;
    d_p->size[0] = 10;
    emxEnsureCapacity_real_T(d_p, expl_temp);
    c_p_data = d_p->data;
    for (expl_temp = 0; expl_temp < 10 * unnamed_idx_1; expl_temp++) {
      c_p_data[nz + d_p->size[1] * i] = b_p_data[expl_temp];
      i++;
      if (i > 9) {
        i = 0;
        nz++;
      }
    }

    vlen = d_p->size[1];
    i = e_p->size[0] * e_p->size[1];
    e_p->size[1] = d_p->size[1];
    e_p->size[0] = 10;
    emxEnsureCapacity_real_T(e_p, i);
    b_p_data = e_p->data;
    for (i = 0; i < 10; i++) {
      for (nz = 0; nz < vlen; nz++) {
        b_p_data[nz + e_p->size[1] * i] = c_p_data[nz + d_p->size[1] * (9 - i)];
      }
    }

    i = d_p->size[0] * d_p->size[1];
    d_p->size[1] = e_p->size[1];
    d_p->size[0] = 10;
    emxEnsureCapacity_real_T(d_p, i);
    c_p_data = d_p->data;
    loop_ub = e_p->size[1];
    for (i = 0; i < 10; i++) {
      for (nz = 0; nz < loop_ub; nz++) {
        c_p_data[nz + d_p->size[1] * i] = b_p_data[nz + e_p->size[1] * i];
      }
    }

    i = r->size[0] * r->size[1];
    r->size[1] = 10;
    loop_ub = d_p->size[1];
    r->size[0] = d_p->size[1];
    emxEnsureCapacity_real_T(r, i);
    b_waypoints_data = r->data;
    for (i = 0; i < loop_ub; i++) {
      for (nz = 0; nz < 10; nz++) {
        b_waypoints_data[nz + 10 * i] = c_p_data[i + d_p->size[1] * nz];
      }
    }

    loop_ub = r->size[0];
    for (i = 0; i < loop_ub; i++) {
      for (nz = 0; nz < 10; nz++) {
        p_data[(dimIdx + p->size[2] * nz) + p->size[2] * 10 * i] =
          b_waypoints_data[nz + 10 * i];
      }
    }

    i = b_waypoints->size[0] * b_waypoints->size[1];
    b_waypoints->size[1] = R_n;
    b_waypoints->size[0] = 1;
    emxEnsureCapacity_real_T(b_waypoints, i);
    b_waypoints_data = b_waypoints->data;
    for (i = 0; i < R_n; i++) {
      b_waypoints_data[i] = 0.0;
    }

    if ((D->size[0] != 0) && (R_n != 0) && (R_colidx_data[R_colidx->size[0] - 1]
         - 1 != 0)) {
      for (k = 0; k < R_n; k++) {
        cd = 0.0;
        vlen = R_colidx_data[k + 1] - 1;
        i = R_colidx_data[k];
        for (expl_temp = i; expl_temp <= vlen; expl_temp++) {
          cd += R_d_data[expl_temp - 1] * D_data[R_rowidx_data[expl_temp - 1] -
            1];
        }

        b_waypoints_data[k] = cd;
      }
    }

    cd = 0.0;
    loop_ub = D->size[0];
    for (i = 0; i < loop_ub; i++) {
      cd += D_data[i] * b_waypoints_data[i];
    }

    J += cd;
  }

  emxFree_real_T(&r2);
  emxFree_real_T(&e_p);
  emxFree_real_T(&b_this);
  emxFree_real_T(&b_waypoints);
  emxFreeStruct_sparse(&d_expl_temp);
  emxFreeStruct_sparse1(&c_expl_temp);
  emxFreeStruct_sparse1(&b_expl_temp);
  emxFree_real_T(&y);
  emxFree_int32_T(&jj);
  emxFree_int32_T(&ii);
  emxFree_boolean_T(&fixed_constr_ids_tmp);
  emxFree_int32_T(&r1);
  emxFree_real_T(&d_p);
  emxFree_real_T(&c_p);
  emxFree_real_T(&tmp_i_col);
  emxFree_real_T(&tmp_i_row);
  emxFree_real_T(&D);
  emxFree_real_T(&DP);
  emxFree_int32_T(&RPF_rowidx);
  emxFree_int32_T(&RPF_colidx);
  emxFree_real_T(&RPF_d);
  emxFree_int32_T(&RPP_rowidx);
  emxFree_int32_T(&RPP_colidx);
  emxFree_real_T(&RPP_d);
  emxFree_int32_T(&R_rowidx);
  emxFree_int32_T(&R_colidx);
  emxFree_real_T(&R_d);
  emxFree_int32_T(&M_rowidx);
  emxFree_int32_T(&M_colidx);
  emxFree_real_T(&M_d);
  emxFreeStruct_sparse(&Q_prime_total_sp);
  emxFreeStruct_sparse(&A_total_sp);
  emxFree_real_T(&r);
  return J;
}

static void spalloc(double m, double n, double nzmax, sparse *s)
{
  int i;
  int numalloc;
  int s_tmp_tmp_tmp;
  s->m = (int)m;
  s_tmp_tmp_tmp = (int)n;
  s->n = (int)n;
  numalloc = (int)nzmax;
  if (numalloc < 1) {
    numalloc = 1;
  }

  i = s->d->size[0];
  s->d->size[0] = numalloc;
  emxEnsureCapacity_real_T(s->d, i);
  s->maxnz = numalloc;
  i = s->colidx->size[0];
  s->colidx->size[0] = (int)n + 1;
  emxEnsureCapacity_int32_T(s->colidx, i);
  s->colidx->data[0] = 1;
  i = s->rowidx->size[0];
  s->rowidx->size[0] = numalloc;
  emxEnsureCapacity_int32_T(s->rowidx, i);
  for (i = 0; i < numalloc; i++) {
    s->d->data[i] = 0.0;
    s->rowidx->data[i] = 0;
  }

  for (numalloc = 0; numalloc < s_tmp_tmp_tmp; numalloc++) {
    s->colidx->data[numalloc + 1] = 1;
  }

  sparse_fillIn(s);
}

static void sparse_ctranspose(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx, const emxArray_int32_T *this_rowidx, int this_m,
  int this_n, sparse *y)
{
  emxArray_int32_T *counts;
  const double *this_d_data;
  const int *this_colidx_data;
  const int *this_rowidx_data;
  int c;
  int numalloc;
  int numalloc_tmp;
  int outridx;
  int *counts_data;
  this_rowidx_data = this_rowidx->data;
  this_colidx_data = this_colidx->data;
  this_d_data = this_d->data;
  y->m = this_n;
  y->n = this_m;
  numalloc_tmp = this_colidx_data[this_colidx->size[0] - 1];
  numalloc = numalloc_tmp - 1;
  if (numalloc < 1) {
    numalloc = 1;
  }

  outridx = y->d->size[0];
  y->d->size[0] = numalloc;
  emxEnsureCapacity_real_T(y->d, outridx);
  y->maxnz = numalloc;
  outridx = y->colidx->size[0];
  y->colidx->size[0] = this_m + 1;
  emxEnsureCapacity_int32_T(y->colidx, outridx);
  y->colidx->data[0] = 1;
  outridx = y->rowidx->size[0];
  y->rowidx->size[0] = numalloc;
  emxEnsureCapacity_int32_T(y->rowidx, outridx);
  for (outridx = 0; outridx < numalloc; outridx++) {
    y->d->data[outridx] = 0.0;
    y->rowidx->data[outridx] = 0;
  }

  for (c = 0; c < this_m; c++) {
    y->colidx->data[c + 1] = 1;
  }

  sparse_fillIn(y);
  emxInit_int32_T(&counts, 1);
  if ((this_m != 0) && (this_n != 0)) {
    numalloc = y->colidx->size[0];
    for (outridx = 0; outridx < numalloc; outridx++) {
      y->colidx->data[outridx] = 0;
    }

    for (numalloc = 0; numalloc <= numalloc_tmp - 2; numalloc++) {
      y->colidx->data[this_rowidx_data[numalloc]]++;
    }

    y->colidx->data[0] = 1;
    outridx = this_m + 1;
    for (numalloc = 2; numalloc <= outridx; numalloc++) {
      y->colidx->data[numalloc - 1] += y->colidx->data[numalloc - 2];
    }

    outridx = counts->size[0];
    counts->size[0] = this_m;
    emxEnsureCapacity_int32_T(counts, outridx);
    counts_data = counts->data;
    for (outridx = 0; outridx < this_m; outridx++) {
      counts_data[outridx] = 0;
    }

    for (c = 0; c < this_n; c++) {
      for (numalloc = this_colidx_data[c] - 1; numalloc + 1 < this_colidx_data[c
           + 1]; numalloc++) {
        numalloc_tmp = counts_data[this_rowidx_data[numalloc] - 1];
        outridx = (numalloc_tmp + y->colidx->data[this_rowidx_data[numalloc] - 1])
          - 1;
        y->d->data[outridx] = this_d_data[numalloc];
        y->rowidx->data[outridx] = c + 1;
        counts_data[this_rowidx_data[numalloc] - 1] = numalloc_tmp + 1;
      }
    }
  }

  emxFree_int32_T(&counts);
}

static void sparse_fillIn(sparse *this)
{
  int c;
  int i;
  int idx;
  idx = 1;
  i = this->colidx->size[0];
  for (c = 0; c <= i - 2; c++) {
    int ridx;
    ridx = this->colidx->data[c];
    this->colidx->data[c] = idx;
    int exitg1;
    int i1;
    do {
      exitg1 = 0;
      i1 = this->colidx->data[c + 1];
      if (ridx < i1) {
        double val;
        int currRowIdx;
        val = 0.0;
        currRowIdx = this->rowidx->data[ridx - 1];
        while ((ridx < i1) && (this->rowidx->data[ridx - 1] == currRowIdx)) {
          val += this->d->data[ridx - 1];
          ridx++;
        }

        if (fabs(val) >= DBL_EPSILON) {
          this->d->data[idx - 1] = val;
          this->rowidx->data[idx - 1] = currRowIdx;
          idx++;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  this->colidx->data[this->colidx->size[0] - 1] = idx;
}

static int sparse_locBsearch(const emxArray_int32_T *x, int xi, int xstart, int
  xend, boolean_T *found)
{
  const int *x_data;
  int n;
  x_data = x->data;
  if (xstart < xend) {
    if (xi < x_data[xstart - 1]) {
      n = xstart - 1;
      *found = false;
    } else {
      int high_i;
      int low_ip1;
      high_i = xend;
      n = xstart;
      low_ip1 = xstart;
      while (high_i > low_ip1 + 1) {
        int mid_i;
        mid_i = (n >> 1) + (high_i >> 1);
        if (((n & 1) == 1) && ((high_i & 1) == 1)) {
          mid_i++;
        }

        if (xi >= x_data[mid_i - 1]) {
          n = mid_i;
          low_ip1 = mid_i;
        } else {
          high_i = mid_i;
        }
      }

      *found = (x_data[n - 1] == xi);
    }
  } else if (xstart == xend) {
    n = xstart - 1;
    *found = false;
  } else {
    n = 0;
    *found = false;
  }

  return n;
}

static void sparse_mldivide(const emxArray_real_T *A_d, const emxArray_int32_T
  *A_colidx, const emxArray_int32_T *A_rowidx, int A_m, int A_n, const
  emxArray_real_T *b, emxArray_real_T *y)
{
  sparse expl_temp;
  const double *A_d_data;
  const double *b_data;
  double *y_data;
  const int *A_colidx_data;
  const int *A_rowidx_data;
  int i;
  b_data = b->data;
  A_rowidx_data = A_rowidx->data;
  A_colidx_data = A_colidx->data;
  A_d_data = A_d->data;
  emxInitStruct_sparse(&expl_temp);
  if ((A_m == 0) || (A_n == 0) || (b->size[0] == 0)) {
    i = y->size[0];
    y->size[0] = A_n;
    emxEnsureCapacity_real_T(y, i);
    y_data = y->data;
    for (i = 0; i < A_n; i++) {
      y_data[i] = 0.0;
    }
  } else if (b->size[0] == A_n) {
    cs_di *cxA;
    cs_din *N;
    cs_dis *S;
    if (A_m < A_n) {
      sparse_ctranspose(A_d, A_colidx, A_rowidx, A_m, A_n, &expl_temp);
      cxA = makeCXSparseMatrix(expl_temp.colidx->data[expl_temp.colidx->size[0]
        - 1] - 1, expl_temp.n, expl_temp.m, &expl_temp.colidx->data[0],
        &expl_temp.rowidx->data[0], &expl_temp.d->data[0]);
    } else {
      cxA = makeCXSparseMatrix(A_colidx_data[A_colidx->size[0] - 1] - 1, A_n,
        A_m, &A_colidx_data[0], &A_rowidx_data[0], &A_d_data[0]);
    }

    S = cs_di_sqr(2, cxA, 0);
    N = cs_di_lu(cxA, S, 1);
    cs_di_spfree(cxA);
    if (N == NULL) {
      cs_di_sfree(S);
      cs_di_nfree(N);
      CXSparseAPI_iteratedQR(A_d, A_colidx, A_rowidx, A_m, A_n, b, A_n, y);
    } else {
      int loop_ub;
      i = y->size[0];
      y->size[0] = b->size[0];
      emxEnsureCapacity_real_T(y, i);
      y_data = y->data;
      loop_ub = b->size[0];
      for (i = 0; i < loop_ub; i++) {
        y_data[i] = b_data[i];
      }

      solve_from_lu_di(N, S, (double *)&y_data[0], b->size[0]);
      cs_di_sfree(S);
      cs_di_nfree(N);
    }
  } else {
    CXSparseAPI_iteratedQR(A_d, A_colidx, A_rowidx, A_m, A_n, b, A_n, y);
  }

  emxFreeStruct_sparse(&expl_temp);
}

static int sparse_mtimes(const emxArray_real_T *a_d, const emxArray_int32_T
  *a_colidx, const emxArray_int32_T *a_rowidx, int a_m, const emxArray_real_T
  *b_d, const emxArray_int32_T *b_colidx, const emxArray_int32_T *b_rowidx, int
  b_n, emxArray_real_T *c_d, emxArray_int32_T *c_colidx, emxArray_int32_T
  *c_rowidx, int *c_n, int *c_maxnz)
{
  emxArray_int32_T *flag;
  emxArray_real_T *wd;
  const double *a_d_data;
  const double *b_d_data;
  double bd;
  double *c_d_data;
  double *wd_data;
  const int *a_colidx_data;
  const int *a_rowidx_data;
  const int *b_colidx_data;
  const int *b_rowidx_data;
  int bcidx;
  int blen;
  int c_m;
  int cmax;
  int cnnz;
  int cstart;
  int i;
  int j;
  int pa;
  int paend;
  int pb;
  int *c_colidx_data;
  int *c_rowidx_data;
  int *flag_data;
  b_rowidx_data = b_rowidx->data;
  b_colidx_data = b_colidx->data;
  b_d_data = b_d->data;
  a_rowidx_data = a_rowidx->data;
  a_colidx_data = a_colidx->data;
  a_d_data = a_d->data;
  i = c_colidx->size[0];
  c_colidx->size[0] = b_colidx->size[0];
  emxEnsureCapacity_int32_T(c_colidx, i);
  c_colidx_data = c_colidx->data;
  blen = b_colidx->size[0];
  for (i = 0; i < blen; i++) {
    c_colidx_data[i] = 0;
  }

  emxInit_int32_T(&flag, 1);
  i = flag->size[0];
  flag->size[0] = a_m;
  emxEnsureCapacity_int32_T(flag, i);
  flag_data = flag->data;
  for (i = 0; i < a_m; i++) {
    flag_data[i] = 0;
  }

  *c_maxnz = 0;
  j = 0;
  int exitg1;
  do {
    exitg1 = 0;
    if (j <= b_n - 1) {
      bcidx = b_colidx_data[j];
      cstart = *c_maxnz;
      cmax = *c_maxnz + a_m;
      c_colidx_data[j] = *c_maxnz + 1;
      while ((bcidx < b_colidx_data[j + 1]) && (*c_maxnz <= cmax)) {
        blen = b_rowidx_data[bcidx - 1];
        paend = a_colidx_data[blen] - 1;
        i = a_colidx_data[blen - 1];
        for (pb = i; pb <= paend; pb++) {
          blen = a_rowidx_data[pb - 1] - 1;
          if (flag_data[blen] != j + 1) {
            flag_data[blen] = j + 1;
            (*c_maxnz)++;
          }
        }

        bcidx++;
      }

      if (*c_maxnz < cstart) {
        exitg1 = 1;
      } else {
        j++;
      }
    } else {
      c_colidx_data[b_n] = *c_maxnz + 1;
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  if (*c_maxnz < 1) {
    *c_maxnz = 1;
  }

  i = c_d->size[0];
  c_d->size[0] = *c_maxnz;
  emxEnsureCapacity_real_T(c_d, i);
  c_d_data = c_d->data;
  i = c_rowidx->size[0];
  c_rowidx->size[0] = *c_maxnz;
  emxEnsureCapacity_int32_T(c_rowidx, i);
  c_rowidx_data = c_rowidx->data;
  emxInit_real_T(&wd, 1);
  i = wd->size[0];
  wd->size[0] = a_m;
  emxEnsureCapacity_real_T(wd, i);
  wd_data = wd->data;
  i = flag->size[0];
  flag->size[0] = a_m;
  emxEnsureCapacity_int32_T(flag, i);
  flag_data = flag->data;
  for (i = 0; i < a_m; i++) {
    flag_data[i] = 0;
  }

  pb = 0;
  cnnz = -1;
  for (j = 0; j < b_n; j++) {
    boolean_T needSort;
    needSort = false;
    cmax = cnnz + 2;
    blen = (b_colidx_data[j + 1] - pb) - 1;
    if (blen != 0) {
      if (blen == 1) {
        cstart = a_colidx_data[b_rowidx_data[pb]];
        paend = cstart - 1;
        i = a_colidx_data[b_rowidx_data[pb] - 1];
        for (pa = i; pa <= paend; pa++) {
          blen = a_rowidx_data[pa - 1];
          c_rowidx_data[((cnnz + pa) - i) + 1] = blen;
          wd_data[blen - 1] = a_d_data[pa - 1] * b_d_data[pb];
        }

        cnnz = (cnnz + cstart) - i;
        pb++;
      } else {
        cstart = a_colidx_data[b_rowidx_data[pb]];
        paend = cstart - 1;
        i = a_colidx_data[b_rowidx_data[pb] - 1];
        for (pa = i; pa <= paend; pa++) {
          blen = a_rowidx_data[pa - 1];
          bcidx = (cnnz + pa) - i;
          flag_data[blen - 1] = bcidx + 2;
          c_rowidx_data[bcidx + 1] = blen;
          wd_data[blen - 1] = a_d_data[pa - 1] * b_d_data[pb];
        }

        cnnz = (cnnz + cstart) - i;
        for (pb++; pb + 1 < b_colidx_data[j + 1]; pb++) {
          bd = b_d_data[pb];
          paend = a_colidx_data[b_rowidx_data[pb]] - 1;
          i = a_colidx_data[b_rowidx_data[pb] - 1];
          for (pa = i; pa <= paend; pa++) {
            blen = a_rowidx_data[pa - 1];
            if (flag_data[blen - 1] < cmax) {
              cnnz++;
              flag_data[blen - 1] = cnnz + 1;
              c_rowidx_data[cnnz] = blen;
              wd_data[blen - 1] = a_d_data[pa - 1] * bd;
              needSort = true;
            } else {
              wd_data[blen - 1] += a_d_data[pa - 1] * bd;
            }
          }
        }
      }
    }

    blen = c_colidx_data[j + 1] - 1;
    bcidx = c_colidx_data[j];
    if (needSort) {
      d_introsort(c_rowidx, bcidx, c_colidx_data[j + 1] - 1);
      c_rowidx_data = c_rowidx->data;
    }

    for (cstart = bcidx; cstart <= blen; cstart++) {
      c_d_data[cstart - 1] = wd_data[c_rowidx_data[cstart - 1] - 1];
    }
  }

  emxFree_int32_T(&flag);
  emxFree_real_T(&wd);
  blen = 1;
  i = c_colidx->size[0];
  for (cmax = 0; cmax <= i - 2; cmax++) {
    bcidx = c_colidx_data[cmax];
    c_colidx_data[cmax] = blen;
    while (bcidx < c_colidx_data[cmax + 1]) {
      cstart = c_rowidx_data[bcidx - 1];
      bd = c_d_data[bcidx - 1];
      bcidx++;
      if (fabs(bd) >= DBL_EPSILON) {
        c_d_data[blen - 1] = bd;
        c_rowidx_data[blen - 1] = cstart;
        blen++;
      }
    }
  }

  c_colidx_data[c_colidx->size[0] - 1] = blen;
  c_m = a_m;
  *c_n = b_n;
  return c_m;
}

static void sparse_parenAssign(sparse *this, const emxArray_real_T *rhs_d, const
  emxArray_int32_T *rhs_colidx, const emxArray_int32_T *rhs_rowidx, int rhs_m,
  const emxArray_real_T *varargin_1, const emxArray_real_T *varargin_2)
{
  emxArray_int32_T *rowidxt;
  emxArray_real_T *dt;
  const double *rhs_d_data;
  const double *varargin_1_data;
  const double *varargin_2_data;
  double *dt_data;
  const int *rhs_colidx_data;
  const int *rhs_rowidx_data;
  int cidx;
  int highOrderA;
  int partialResults_idx_1;
  int rhsIter_col;
  int rhsIter_idx;
  int rhsIter_row;
  int ridx;
  int sm;
  int sn;
  int *rowidxt_data;
  boolean_T found;
  varargin_2_data = varargin_2->data;
  varargin_1_data = varargin_1->data;
  rhs_rowidx_data = rhs_rowidx->data;
  rhs_colidx_data = rhs_colidx->data;
  rhs_d_data = rhs_d->data;
  sm = varargin_1->size[0];
  sn = varargin_2->size[0];
  rhsIter_idx = 1;
  rhsIter_col = 1;
  rhsIter_row = 2;
  emxInit_int32_T(&rowidxt, 1);
  emxInit_real_T(&dt, 1);
  for (cidx = 0; cidx < sn; cidx++) {
    double nt;
    nt = varargin_2_data[cidx];
    for (ridx = 0; ridx < sm; ridx++) {
      double rhsv;
      double thisv;
      int i;
      int vidx;
      i = (int)varargin_1_data[ridx];
      vidx = sparse_locBsearch(this->rowidx, i, this->colidx->data[(int)nt - 1],
        this->colidx->data[(int)nt], &found);
      if (found) {
        thisv = this->d->data[vidx - 1];
      } else {
        thisv = 0.0;
      }

      if ((rhsIter_idx < rhs_colidx_data[rhsIter_col]) && (rhsIter_idx <=
           rhs_colidx_data[rhs_colidx->size[0] - 1] - 1) && (rhsIter_row - 1 ==
           rhs_rowidx_data[rhsIter_idx - 1])) {
        rhsv = rhs_d_data[rhsIter_idx - 1];
        rhsIter_idx++;
      } else {
        rhsv = 0.0;
      }

      partialResults_idx_1 = rhsIter_row;
      rhsIter_row++;
      if (partialResults_idx_1 > rhs_m) {
        rhsIter_col++;
        rhsIter_row = 2;
      }

      if ((!(fabs(thisv) < DBL_EPSILON)) || (!(fabs(rhsv) < DBL_EPSILON))) {
        int nz;
        nz = this->colidx->data[this->colidx->size[0] - 1] - 1;
        if ((fabs(thisv) >= DBL_EPSILON) && (fabs(rhsv) >= DBL_EPSILON)) {
          this->d->data[vidx - 1] = rhsv;
        } else if (fabs(thisv) < DBL_EPSILON) {
          int idx;
          idx = vidx + 1;
          if (this->colidx->data[this->colidx->size[0] - 1] - 1 == this->maxnz)
          {
            int highOrderB;
            int lowOrderB;
            int overflow;
            int partialResults_idx_0_tmp;
            int tmp;
            highOrderA = rowidxt->size[0];
            rowidxt->size[0] = this->rowidx->size[0];
            emxEnsureCapacity_int32_T(rowidxt, highOrderA);
            rowidxt_data = rowidxt->data;
            partialResults_idx_1 = this->rowidx->size[0];
            for (highOrderA = 0; highOrderA < partialResults_idx_1; highOrderA++)
            {
              rowidxt_data[highOrderA] = this->rowidx->data[highOrderA];
            }

            highOrderA = dt->size[0];
            dt->size[0] = this->d->size[0];
            emxEnsureCapacity_real_T(dt, highOrderA);
            dt_data = dt->data;
            partialResults_idx_1 = this->d->size[0];
            for (highOrderA = 0; highOrderA < partialResults_idx_1; highOrderA++)
            {
              dt_data[highOrderA] = this->d->data[highOrderA];
            }

            highOrderA = this->m >> 16;
            partialResults_idx_1 = this->m & 65535;
            highOrderB = this->n >> 16;
            lowOrderB = this->n & 65535;
            partialResults_idx_0_tmp = partialResults_idx_1 * lowOrderB;
            tmp = partialResults_idx_1 * highOrderB;
            partialResults_idx_1 = tmp << 16;
            overflow = tmp >> 16;
            if (overflow <= 0) {
              tmp = highOrderA * lowOrderB;
              overflow += tmp >> 16;
              if (overflow <= 0) {
                overflow += highOrderA * highOrderB;
                if (overflow <= 0) {
                  if (partialResults_idx_0_tmp > MAX_int32_T
                      - partialResults_idx_1) {
                    partialResults_idx_1 = (partialResults_idx_0_tmp +
                      partialResults_idx_1) - MAX_int32_T;
                    overflow++;
                  } else {
                    partialResults_idx_1 += partialResults_idx_0_tmp;
                  }

                  if (partialResults_idx_1 > MAX_int32_T - (tmp << 16)) {
                    overflow++;
                  }
                }
              }
            }

            if (overflow == 0) {
              partialResults_idx_1 = this->colidx->data[this->colidx->size[0] -
                1] + 9;
              highOrderA = this->m * this->n;
              if (partialResults_idx_1 <= highOrderA) {
                highOrderA = partialResults_idx_1;
              }

              if (highOrderA <= 1) {
                partialResults_idx_1 = 1;
              } else {
                partialResults_idx_1 = highOrderA;
              }
            } else if (this->colidx->data[this->colidx->size[0] - 1] + 9 <= 1) {
              partialResults_idx_1 = 1;
            } else {
              partialResults_idx_1 = this->colidx->data[this->colidx->size[0] -
                1] + 9;
            }

            highOrderA = this->rowidx->size[0];
            this->rowidx->size[0] = partialResults_idx_1;
            emxEnsureCapacity_int32_T(this->rowidx, highOrderA);
            highOrderA = this->d->size[0];
            this->d->size[0] = partialResults_idx_1;
            emxEnsureCapacity_real_T(this->d, highOrderA);
            for (highOrderA = 0; highOrderA < partialResults_idx_1; highOrderA++)
            {
              this->rowidx->data[highOrderA] = 0;
              this->d->data[highOrderA] = 0.0;
            }

            this->maxnz = partialResults_idx_1;
            for (partialResults_idx_1 = 0; partialResults_idx_1 < vidx;
                 partialResults_idx_1++) {
              this->rowidx->data[partialResults_idx_1] =
                rowidxt_data[partialResults_idx_1];
              this->d->data[partialResults_idx_1] = dt_data[partialResults_idx_1];
            }

            for (partialResults_idx_1 = idx; partialResults_idx_1 <= nz;
                 partialResults_idx_1++) {
              this->rowidx->data[partialResults_idx_1] =
                rowidxt_data[partialResults_idx_1 - 1];
              this->d->data[partialResults_idx_1] = dt_data[partialResults_idx_1
                - 1];
            }

            this->rowidx->data[vidx] = i;
            this->d->data[vidx] = rhsv;
          } else {
            partialResults_idx_1 = (this->colidx->data[this->colidx->size[0] - 1]
              - vidx) - 1;
            if (partialResults_idx_1 > 0) {
              memmove((void *)&this->rowidx->data[vidx + 1], (void *)
                      &this->rowidx->data[vidx], (unsigned int)((size_t)
                       partialResults_idx_1 * sizeof(int)));
              memmove((void *)&this->d->data[vidx + 1], (void *)&this->d->
                      data[vidx], (unsigned int)((size_t)partialResults_idx_1 *
                       sizeof(double)));
            }

            this->d->data[vidx] = rhsv;
            this->rowidx->data[vidx] = i;
          }

          i = (int)nt + 1;
          highOrderA = this->n + 1;
          for (partialResults_idx_1 = i; partialResults_idx_1 <= highOrderA;
               partialResults_idx_1++) {
            this->colidx->data[partialResults_idx_1 - 1]++;
          }
        } else {
          partialResults_idx_1 = (this->colidx->data[this->colidx->size[0] - 1]
            - vidx) - 1;
          if (partialResults_idx_1 > 0) {
            memmove((void *)&this->rowidx->data[vidx - 1], (void *)&this->
                    rowidx->data[vidx], (unsigned int)((size_t)
                     partialResults_idx_1 * sizeof(int)));
            memmove((void *)&this->d->data[vidx - 1], (void *)&this->d->
                    data[vidx], (unsigned int)((size_t)partialResults_idx_1 *
                     sizeof(double)));
          }

          i = (int)nt + 1;
          highOrderA = this->n + 1;
          for (partialResults_idx_1 = i; partialResults_idx_1 <= highOrderA;
               partialResults_idx_1++) {
            this->colidx->data[partialResults_idx_1 - 1]--;
          }
        }
      }
    }
  }

  emxFree_real_T(&dt);
  emxFree_int32_T(&rowidxt);
}

static int sparse_parenReference(const emxArray_real_T *this_d, const
  emxArray_int32_T *this_colidx, const emxArray_int32_T *this_rowidx, const
  emxArray_real_T *varargin_1, const emxArray_real_T *varargin_2,
  emxArray_real_T *s_d, emxArray_int32_T *s_colidx, emxArray_int32_T *s_rowidx,
  int *s_n, int *s_maxnz)
{
  const double *this_d_data;
  const double *varargin_1_data;
  const double *varargin_2_data;
  double *s_d_data;
  const int *this_colidx_data;
  int cidx;
  int colNnz;
  int i;
  int k;
  int ridx;
  int s_m;
  int sm;
  int sn;
  int *s_colidx_data;
  int *s_rowidx_data;
  boolean_T found;
  varargin_2_data = varargin_2->data;
  varargin_1_data = varargin_1->data;
  this_colidx_data = this_colidx->data;
  this_d_data = this_d->data;
  sm = varargin_1->size[0];
  sn = varargin_2->size[0];
  s_d->size[0] = 0;
  s_rowidx->size[0] = 0;
  i = s_colidx->size[0];
  s_colidx->size[0] = varargin_2->size[0] + 1;
  emxEnsureCapacity_int32_T(s_colidx, i);
  s_colidx_data = s_colidx->data;
  colNnz = varargin_2->size[0] + 1;
  for (i = 0; i < colNnz; i++) {
    s_colidx_data[i] = 0;
  }

  s_colidx_data[0] = 1;
  colNnz = 1;
  k = 0;
  for (cidx = 0; cidx < sn; cidx++) {
    double nt;
    nt = varargin_2_data[cidx];
    for (ridx = 0; ridx < sm; ridx++) {
      int idx;
      idx = sparse_locBsearch(this_rowidx, (int)varargin_1_data[ridx],
        this_colidx_data[(int)nt - 1], this_colidx_data[(int)nt], &found);
      if (found) {
        double s_d_tmp;
        int i1;
        i = s_d->size[0];
        i1 = s_d->size[0];
        s_d->size[0]++;
        emxEnsureCapacity_real_T(s_d, i1);
        s_d_data = s_d->data;
        s_d_tmp = this_d_data[idx - 1];
        s_d_data[i] = s_d_tmp;
        i = s_rowidx->size[0];
        i1 = s_rowidx->size[0];
        s_rowidx->size[0]++;
        emxEnsureCapacity_int32_T(s_rowidx, i1);
        s_rowidx_data = s_rowidx->data;
        s_rowidx_data[i] = ridx + 1;
        s_d_data[k] = s_d_tmp;
        s_rowidx_data[k] = ridx + 1;
        k++;
        colNnz++;
      }
    }

    s_colidx_data[cidx + 1] = colNnz;
  }

  *s_maxnz = s_colidx_data[s_colidx->size[0] - 1] - 1;
  if (*s_maxnz == 0) {
    i = s_rowidx->size[0];
    s_rowidx->size[0] = 1;
    emxEnsureCapacity_int32_T(s_rowidx, i);
    s_rowidx_data = s_rowidx->data;
    s_rowidx_data[0] = 1;
    i = s_d->size[0];
    s_d->size[0] = 1;
    emxEnsureCapacity_real_T(s_d, i);
    s_d_data = s_d->data;
    s_d_data[0] = 0.0;
  }

  s_m = varargin_1->size[0];
  *s_n = varargin_2->size[0];
  if (*s_maxnz < 1) {
    *s_maxnz = 1;
  }

  return s_m;
}

static double sum(const emxArray_real_T *x)
{
  const double *x_data;
  double y;
  int ib;
  int k;
  int vlen;
  x_data = x->data;
  vlen = x->size[0];
  if (x->size[0] == 0) {
    y = 0.0;
  } else {
    int firstBlockLength;
    int lastBlockLength;
    int nblocks;
    if (x->size[0] <= 1024) {
      firstBlockLength = x->size[0];
      lastBlockLength = 0;
      nblocks = 1;
    } else {
      firstBlockLength = 1024;
      nblocks = (int)((unsigned int)x->size[0] >> 10);
      lastBlockLength = x->size[0] - (nblocks << 10);
      if (lastBlockLength > 0) {
        nblocks++;
      } else {
        lastBlockLength = 1024;
      }
    }

    y = x_data[0];
    for (k = 2; k <= firstBlockLength; k++) {
      if (vlen >= 2) {
        y += x_data[k - 1];
      }
    }

    for (ib = 2; ib <= nblocks; ib++) {
      double bsum;
      int hi;
      firstBlockLength = (ib - 1) << 10;
      bsum = x_data[firstBlockLength];
      if (ib == nblocks) {
        hi = lastBlockLength;
      } else {
        hi = 1024;
      }

      for (k = 2; k <= hi; k++) {
        if (vlen >= 2) {
          bsum += x_data[(firstBlockLength + k) - 1];
        }
      }

      y += bsum;
    }
  }

  return y;
}

static double tic(double *tstart_tv_nsec)
{
  coderTimespec b_timespec;
  double tstart_tv_sec;
  if (!freq_not_empty) {
    freq_not_empty = true;
    coderInitTimeFunctions(&freq);
  }

  coderTimeClockGettimeMonotonic(&b_timespec, freq);
  tstart_tv_sec = b_timespec.tv_sec;
  *tstart_tv_nsec = b_timespec.tv_nsec;
  return tstart_tv_sec;
}

static double toc(double tstart_tv_sec, double tstart_tv_nsec)
{
  coderTimespec b_timespec;
  if (!freq_not_empty) {
    freq_not_empty = true;
    coderInitTimeFunctions(&freq);
  }

  coderTimeClockGettimeMonotonic(&b_timespec, freq);
  return (b_timespec.tv_sec - tstart_tv_sec) + (b_timespec.tv_nsec -
    tstart_tv_nsec) / 1.0E+9;
}

static void xgeqp3(emxArray_real_T *A, emxArray_real_T *tau, emxArray_int32_T
                   *jpvt)
{
  emxArray_real_T *vn1;
  emxArray_real_T *vn2;
  emxArray_real_T *work;
  double *A_data;
  double *tau_data;
  double *vn1_data;
  double *vn2_data;
  double *work_data;
  int b_i;
  int i;
  int ia;
  int ix;
  int jA;
  int knt;
  int m;
  int n;
  int nmi;
  int u1;
  int *jpvt_data;
  A_data = A->data;
  m = A->size[1];
  n = A->size[0];
  ix = A->size[1];
  u1 = A->size[0];
  if (ix <= u1) {
    u1 = ix;
  }

  i = tau->size[0];
  tau->size[0] = u1;
  emxEnsureCapacity_real_T(tau, i);
  tau_data = tau->data;
  for (i = 0; i < u1; i++) {
    tau_data[i] = 0.0;
  }

  emxInit_real_T(&work, 1);
  emxInit_real_T(&vn1, 1);
  emxInit_real_T(&vn2, 1);
  if ((A->size[1] == 0) || (A->size[0] == 0) || (u1 < 1)) {
    ix = A->size[0];
    i = jpvt->size[0] * jpvt->size[1];
    jpvt->size[1] = 1;
    jpvt->size[0] = ix;
    emxEnsureCapacity_int32_T(jpvt, i);
    jpvt_data = jpvt->data;
    for (nmi = 0; nmi < ix; nmi++) {
      jpvt_data[nmi] = nmi + 1;
    }
  } else {
    double smax;
    int ma;
    ix = A->size[0];
    i = jpvt->size[0] * jpvt->size[1];
    jpvt->size[1] = 1;
    jpvt->size[0] = ix;
    emxEnsureCapacity_int32_T(jpvt, i);
    jpvt_data = jpvt->data;
    for (jA = 0; jA < ix; jA++) {
      jpvt_data[jA] = jA + 1;
    }

    ma = A->size[1];
    ix = A->size[0];
    i = work->size[0];
    work->size[0] = ix;
    emxEnsureCapacity_real_T(work, i);
    work_data = work->data;
    i = vn1->size[0];
    vn1->size[0] = A->size[0];
    emxEnsureCapacity_real_T(vn1, i);
    vn1_data = vn1->data;
    i = vn2->size[0];
    vn2->size[0] = A->size[0];
    emxEnsureCapacity_real_T(vn2, i);
    vn2_data = vn2->data;
    for (nmi = 0; nmi < ix; nmi++) {
      work_data[nmi] = 0.0;
      smax = xnrm2(m, A, nmi * ma + 1);
      vn1_data[nmi] = smax;
      vn2_data[nmi] = smax;
    }

    for (b_i = 0; b_i < u1; b_i++) {
      double s;
      double temp2;
      int ii;
      int ip1;
      int lastc;
      int mmi;
      int pvt;
      ip1 = b_i + 2;
      lastc = b_i * ma;
      ii = lastc + b_i;
      nmi = n - b_i;
      mmi = m - b_i;
      if (nmi < 1) {
        ix = -1;
      } else {
        ix = 0;
        if (nmi > 1) {
          smax = fabs(vn1_data[b_i]);
          for (jA = 2; jA <= nmi; jA++) {
            s = fabs(vn1_data[(b_i + jA) - 1]);
            if (s > smax) {
              ix = jA - 1;
              smax = s;
            }
          }
        }
      }

      pvt = b_i + ix;
      if (pvt + 1 != b_i + 1) {
        ix = pvt * ma;
        for (jA = 0; jA < m; jA++) {
          knt = ix + jA;
          smax = A_data[knt];
          i = lastc + jA;
          A_data[knt] = A_data[i];
          A_data[i] = smax;
        }

        ix = jpvt_data[pvt];
        jpvt_data[pvt] = jpvt_data[b_i];
        jpvt_data[b_i] = ix;
        vn1_data[pvt] = vn1_data[b_i];
        vn2_data[pvt] = vn2_data[b_i];
      }

      if (b_i + 1 < m) {
        temp2 = A_data[ii];
        ix = ii + 2;
        tau_data[b_i] = 0.0;
        if (mmi > 0) {
          smax = xnrm2(mmi - 1, A, ii + 2);
          if (fabs(smax) >= DBL_EPSILON) {
            s = rt_hypotd_snf(A_data[ii], smax);
            if (A_data[ii] >= 0.0) {
              s = -s;
            }

            if (fabs(s) < 1.0020841800044864E-292) {
              knt = 0;
              i = ii + mmi;
              do {
                knt++;
                for (jA = ix; jA <= i; jA++) {
                  A_data[jA - 1] *= 9.9792015476736E+291;
                }

                s *= 9.9792015476736E+291;
                temp2 *= 9.9792015476736E+291;
              } while ((fabs(s) < 1.0020841800044864E-292) && (knt < 20));

              s = rt_hypotd_snf(temp2, xnrm2(mmi - 1, A, ii + 2));
              if (temp2 >= 0.0) {
                s = -s;
              }

              tau_data[b_i] = (s - temp2) / s;
              smax = 1.0 / (temp2 - s);
              for (jA = ix; jA <= i; jA++) {
                A_data[jA - 1] *= smax;
              }

              for (jA = 0; jA < knt; jA++) {
                s *= 1.0020841800044864E-292;
              }

              temp2 = s;
            } else {
              tau_data[b_i] = (s - A_data[ii]) / s;
              smax = 1.0 / (A_data[ii] - s);
              i = ii + mmi;
              for (jA = ix; jA <= i; jA++) {
                A_data[jA - 1] *= smax;
              }

              temp2 = s;
            }
          }
        }

        A_data[ii] = temp2;
      } else {
        tau_data[b_i] = 0.0;
      }

      if (b_i + 1 < n) {
        temp2 = A_data[ii];
        A_data[ii] = 1.0;
        jA = (ii + ma) + 1;
        if (fabs(tau_data[b_i]) >= DBL_EPSILON) {
          boolean_T exitg2;
          pvt = mmi - 1;
          ix = (ii + mmi) - 1;
          while ((pvt + 1 > 0) && (fabs(A_data[ix]) < DBL_EPSILON)) {
            pvt--;
            ix--;
          }

          lastc = nmi - 2;
          exitg2 = false;
          while ((!exitg2) && (lastc + 1 > 0)) {
            int exitg1;
            ix = jA + lastc * ma;
            ia = ix;
            do {
              exitg1 = 0;
              if (ia <= ix + pvt) {
                if (fabs(A_data[ia - 1]) >= DBL_EPSILON) {
                  exitg1 = 1;
                } else {
                  ia++;
                }
              } else {
                lastc--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          pvt = -1;
          lastc = -1;
        }

        if (pvt + 1 > 0) {
          if (lastc + 1 != 0) {
            for (ix = 0; ix <= lastc; ix++) {
              work_data[ix] = 0.0;
            }

            ix = 0;
            i = jA + ma * lastc;
            for (nmi = jA; ma < 0 ? nmi >= i : nmi <= i; nmi += ma) {
              smax = 0.0;
              knt = nmi + pvt;
              for (ia = nmi; ia <= knt; ia++) {
                smax += A_data[ia - 1] * A_data[(ii + ia) - nmi];
              }

              work_data[ix] += smax;
              ix++;
            }
          }

          if (!(fabs(-tau_data[b_i]) < DBL_EPSILON)) {
            for (nmi = 0; nmi <= lastc; nmi++) {
              if (fabs(work_data[nmi]) >= DBL_EPSILON) {
                smax = work_data[nmi] * -tau_data[b_i];
                i = pvt + jA;
                for (knt = jA; knt <= i; knt++) {
                  A_data[knt - 1] += A_data[(ii + knt) - jA] * smax;
                }
              }

              jA += ma;
            }
          }
        }

        A_data[ii] = temp2;
      }

      for (nmi = ip1; nmi <= n; nmi++) {
        ix = b_i + (nmi - 1) * ma;
        smax = vn1_data[nmi - 1];
        if (fabs(smax) >= DBL_EPSILON) {
          s = fabs(A_data[ix]) / smax;
          s = 1.0 - s * s;
          if (s < 0.0) {
            s = 0.0;
          }

          temp2 = smax / vn2_data[nmi - 1];
          temp2 = s * (temp2 * temp2);
          if (temp2 <= 1.4901161193847656E-8) {
            if (b_i + 1 < m) {
              smax = xnrm2(mmi - 1, A, ix + 2);
              vn1_data[nmi - 1] = smax;
              vn2_data[nmi - 1] = smax;
            } else {
              vn1_data[nmi - 1] = 0.0;
              vn2_data[nmi - 1] = 0.0;
            }
          } else {
            vn1_data[nmi - 1] = smax * sqrt(s);
          }
        }
      }
    }
  }

  emxFree_real_T(&vn2);
  emxFree_real_T(&vn1);
  emxFree_real_T(&work);
}

static double xnrm2(int n, const emxArray_real_T *x, int ix0)
{
  const double *x_data;
  double y;
  int k;
  x_data = x->data;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x_data[ix0 - 1]);
    } else {
      double scale;
      int kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        double absxk;
        absxk = fabs(x_data[k - 1]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

static int xzgetrf(int m, int n, emxArray_real_T *A, int lda, emxArray_int32_T
                   *ipiv)
{
  double *A_data;
  int b_j;
  int b_n;
  int i;
  int info;
  int j;
  int k;
  int yk;
  int *ipiv_data;
  A_data = A->data;
  if (m <= n) {
    yk = m;
  } else {
    yk = n;
  }

  if (yk < 1) {
    b_n = 0;
  } else {
    b_n = yk;
  }

  i = ipiv->size[0] * ipiv->size[1];
  ipiv->size[1] = 1;
  ipiv->size[0] = b_n;
  emxEnsureCapacity_int32_T(ipiv, i);
  ipiv_data = ipiv->data;
  if (b_n > 0) {
    ipiv_data[0] = 1;
    yk = 1;
    for (k = 2; k <= b_n; k++) {
      yk++;
      ipiv_data[k - 1] = yk;
    }
  }

  info = 0;
  if ((m >= 1) && (n >= 1)) {
    int u0;
    u0 = m - 1;
    if (u0 > n) {
      u0 = n;
    }

    for (j = 0; j < u0; j++) {
      double smax;
      int b_tmp;
      int ipiv_tmp;
      int jA;
      int mmj;
      mmj = m - j;
      b_tmp = j * (lda + 1);
      b_n = b_tmp + 2;
      if (mmj < 1) {
        yk = -1;
      } else {
        yk = 0;
        if (mmj > 1) {
          smax = fabs(A_data[b_tmp]);
          for (k = 2; k <= mmj; k++) {
            double s;
            s = fabs(A_data[(b_tmp + k) - 1]);
            if (s > smax) {
              yk = k - 1;
              smax = s;
            }
          }
        }
      }

      if (fabs(A_data[b_tmp + yk]) >= DBL_EPSILON) {
        if (yk != 0) {
          ipiv_tmp = j + yk;
          ipiv_data[j] = ipiv_tmp + 1;
          for (k = 0; k < n; k++) {
            yk = k * lda;
            jA = j + yk;
            smax = A_data[jA];
            i = ipiv_tmp + yk;
            A_data[jA] = A_data[i];
            A_data[i] = smax;
          }
        }

        i = b_tmp + mmj;
        for (yk = b_n; yk <= i; yk++) {
          A_data[yk - 1] /= A_data[b_tmp];
        }
      } else {
        info = j + 1;
      }

      b_n = n - j;
      ipiv_tmp = b_tmp + lda;
      jA = ipiv_tmp;
      for (b_j = 0; b_j <= b_n - 2; b_j++) {
        yk = ipiv_tmp + b_j * lda;
        smax = A_data[yk];
        if (fabs(A_data[yk]) >= DBL_EPSILON) {
          i = jA + 2;
          yk = mmj + jA;
          for (k = i; k <= yk; k++) {
            A_data[k - 1] += A_data[((b_tmp + k) - jA) - 1] * -smax;
          }
        }

        jA += lda;
      }
    }

    if ((info == 0) && (m <= n) && (!(fabs(A_data[(m + A->size[1] * (m - 1)) - 1]) >=
          DBL_EPSILON))) {
      info = m;
    }
  }

  return info;
}

void solver_codegen(const double wpts_data[], const int wpts_size[2], double Tf,
                    boolean_T opt_time_allocation_fl, boolean_T ShowDetails,
                    emxArray_real_T *T, emxArray_real_T *pp, double *n_dim,
                    emxArray_real_T *n_dim_ids, double *N_segments,
                    emxArray_real_T *offset, double *Cost, double *ExitFlag,
                    double *Iterations)
{
  c_TrajectoryOptimizer_solver_me sol;
  emxArray_real_T *initialGuess;
  emxArray_real_T *ppMatrix;
  emxArray_real_T *r;
  emxArray_real_T *r2;
  emxArray_real_T *tSegments;
  double J;
  double b_expl_temp;
  double delta1;
  double exitstruct_ExitFlag;
  double exitstruct_Iterations;
  double expl_temp;
  double minsnap_set_N_segments;
  double minsnap_set_n_dim;
  double minsnap_set_n_dim_src;
  double stats_ExitFlag;
  double stats_Iterations;
  double stats_J;
  double unnamed_idx_2;
  double *T_data;
  double *initialGuess_data;
  double *n_dim_ids_data;
  double *offset_data;
  double *ppMatrix_data;
  double *pp_data;
  double *r1;
  double *tSegments_data;
  int expl_temp_size[2];
  int b_i;
  int i;
  int i1;
  int i2;
  int i3;
  int i_coefs;
  int i_dim;
  int i_seg;
  int k;
  int loop_ub;
  char expl_temp_data[14];
  boolean_T is_good;
  boolean_T y;
  if (!isInitialized_solver_codegen) {
    solver_codegen_initialize();
  }

  c_emxInitStruct_TrajectoryOptim(&sol);
  i = sol.timePoints->size[0] * sol.timePoints->size[1];
  sol.timePoints->size[1] = wpts_size[1];
  sol.timePoints->size[0] = 1;
  emxEnsureCapacity_real_T(sol.timePoints, i);
  if (sol.timePoints->size[1] >= 1) {
    sol.timePoints->data[sol.timePoints->size[1] - 1] = Tf;
    if (sol.timePoints->size[1] >= 2) {
      sol.timePoints->data[0] = 0.0;
      if (sol.timePoints->size[1] >= 3) {
        if (fabs(-Tf) < DBL_EPSILON) {
          delta1 = Tf / ((double)sol.timePoints->size[1] - 1.0);
          i = sol.timePoints->size[1] - 1;
          for (k = 2; k <= i; k++) {
            sol.timePoints->data[k - 1] = ((double)((k << 1) -
              sol.timePoints->size[1]) - 1.0) * delta1;
          }

          if ((sol.timePoints->size[1] & 1) == 1) {
            sol.timePoints->data[sol.timePoints->size[1] >> 1] = 0.0;
          }
        } else if ((Tf < 0.0) && (fabs(Tf) > 8.9884656743115785E+307)) {
          delta1 = Tf / ((double)sol.timePoints->size[1] - 1.0);
          i = sol.timePoints->size[1];
          for (k = 0; k <= i - 3; k++) {
            sol.timePoints->data[k + 1] = delta1 * ((double)k + 1.0);
          }
        } else {
          delta1 = Tf / ((double)sol.timePoints->size[1] - 1.0);
          i = sol.timePoints->size[1];
          for (k = 0; k <= i - 3; k++) {
            sol.timePoints->data[k + 1] = ((double)k + 1.0) * delta1;
          }
        }
      }
    }
  }

  i = sol.wptFnc.workspace.wpts->size[0] * sol.wptFnc.workspace.wpts->size[1];
  sol.wptFnc.workspace.wpts->size[1] = wpts_size[1];
  sol.wptFnc.workspace.wpts->size[0] = wpts_size[0];
  emxEnsureCapacity_real_T(sol.wptFnc.workspace.wpts, i);
  loop_ub = wpts_size[0];
  for (i = 0; i < loop_ub; i++) {
    k = wpts_size[1];
    for (i1 = 0; i1 < k; i1++) {
      sol.wptFnc.workspace.wpts->data[i1 + sol.wptFnc.workspace.wpts->size[1] *
        i] = wpts_data[i1 + wpts_size[1] * i];
    }
  }

  sol.nontriv_wpts = 0.0;
  sol.N_segments = 0.0;
  sol.timeOptim = opt_time_allocation_fl;
  sol.print_stats_fl = ShowDetails;
  emxInit_real_T(&r, 2);
  diff(sol.timePoints, r);
  r1 = r->data;
  i = sol.minSegmentTime->size[0] * sol.minSegmentTime->size[1];
  sol.minSegmentTime->size[1] = r->size[1];
  sol.minSegmentTime->size[0] = 1;
  emxEnsureCapacity_real_T(sol.minSegmentTime, i);
  loop_ub = r->size[1];
  for (i = 0; i < loop_ub; i++) {
    sol.minSegmentTime->data[i] = 0.1 * r1[i];
  }

  emxFree_real_T(&r);
  sol.maxSegmentTime = Tf;
  sol.timeWt = 0.0;
  emxInit_real_T(&initialGuess, 2);
  diff(sol.timePoints, initialGuess);
  initialGuess_data = initialGuess->data;
  emxInit_real_T(&ppMatrix, 3);
  emxInit_real_T(&tSegments, 1);
  emxInit_real_T(&r2, 1);

  px4_usleep(PX4_SOLVER_WAIT_TIME_US);// don't update too frequenty

  if (sol.timeOptim && (initialGuess->size[1] > 1)) {
    J = solver_common_optimize(&sol, initialGuess, ppMatrix, tSegments,
      expl_temp_data, expl_temp_size, &exitstruct_Iterations, &expl_temp,
      &b_expl_temp, &exitstruct_ExitFlag);
    tSegments_data = tSegments->data;
    ppMatrix_data = ppMatrix->data;
    sol.Iterations = exitstruct_Iterations;
    if (sol.print_stats_fl) {
      switch ((int)exitstruct_ExitFlag) {
       case 0:
        delta1 = rt_roundd_snf(exitstruct_Iterations);
        if (delta1 < 2.147483648E+9) {
          if (delta1 >= -2.147483648E+9) {
            i = (int)delta1;
          } else {
            i = MIN_int32_T;
          }
        } else if (delta1 >= 2.147483648E+9) {
          i = MAX_int32_T;
        } else {
          i = 0;
        }

        printf("Problem solved. Local minimum found, number of iterations: %i\n",
               i);
        fflush(stdout);
        break;

       case 1:
        printf("Failed to solve the problem: time limit exceeded\n");
        fflush(stdout);
        break;

       case 2:
        delta1 = rt_roundd_snf(exitstruct_Iterations);
        if (delta1 < 2.147483648E+9) {
          if (delta1 >= -2.147483648E+9) {
            i = (int)delta1;
          } else {
            i = MIN_int32_T;
          }
        } else if (delta1 >= 2.147483648E+9) {
          i = MAX_int32_T;
        } else {
          i = 0;
        }

        printf("Local minimum possible: step size below minimum, number of iterations: %i\n",
               i);
        fflush(stdout);
        break;

       case 3:
        printf("Failed to solve the problem: hessian not positive semi-definite\n");
        fflush(stdout);
        break;

       case 4:
        printf("Failed to solve the problem: search direction invalid\n");
        fflush(stdout);
        break;

       default:
        printf("Failed to solve the problem: iteration limit exceeded\n");
        fflush(stdout);
        break;
      }
    }

    sol.ExitFlag = exitstruct_ExitFlag;
    if ((tSegments->size[0] != 1) && (tSegments->size[0] != 0) &&
        (tSegments->size[0] != 1)) {
      i = tSegments->size[0];
      for (k = 0; k <= i - 2; k++) {
        tSegments_data[k + 1] += tSegments_data[k];
      }
    }

    i = sol.timeOfArrival->size[0] * sol.timeOfArrival->size[1];
    sol.timeOfArrival->size[1] = tSegments->size[0] + 1;
    sol.timeOfArrival->size[0] = 1;
    emxEnsureCapacity_real_T(sol.timeOfArrival, i);
    sol.timeOfArrival->data[0] = 0.0;
    loop_ub = tSegments->size[0];
    for (i = 0; i < loop_ub; i++) {
      sol.timeOfArrival->data[i + 1] = tSegments_data[i];
    }
  } else {
    J = b_solver_common_solvePoly(&sol, initialGuess, ppMatrix);
    ppMatrix_data = ppMatrix->data;
    i = r2->size[0];
    r2->size[0] = initialGuess->size[1];
    emxEnsureCapacity_real_T(r2, i);
    r1 = r2->data;
    loop_ub = initialGuess->size[1];
    for (i = 0; i < loop_ub; i++) {
      r1[i] = initialGuess_data[i];
    }

    if ((initialGuess->size[1] != 1) && (initialGuess->size[1] != 0) &&
        (initialGuess->size[1] != 1)) {
      i = initialGuess->size[1];
      for (k = 0; k <= i - 2; k++) {
        r1[k + 1] += r1[k];
      }
    }

    i = sol.timeOfArrival->size[0] * sol.timeOfArrival->size[1];
    sol.timeOfArrival->size[1] = r2->size[0] + 1;
    sol.timeOfArrival->size[0] = 1;
    emxEnsureCapacity_real_T(sol.timeOfArrival, i);
    sol.timeOfArrival->data[0] = 0.0;
    loop_ub = r2->size[0];
    for (i = 0; i < loop_ub; i++) {
      sol.timeOfArrival->data[i + 1] = r1[i];
    }

    sol.Iterations = 0.0;
    if (sol.print_stats_fl) {
      printf("Fixed-time problem solved.\n");
      fflush(stdout);
    }

    sol.ExitFlag = 0.0;
  }

  emxFree_real_T(&r2);
  emxFree_real_T(&tSegments);
  emxFree_real_T(&initialGuess);
  delta1 = sol.N_segments;
  unnamed_idx_2 = sol.n_dim_src;
  loop_ub = (int)unnamed_idx_2;
  i = sol.pp->size[0] * sol.pp->size[1] * sol.pp->size[2];
  sol.pp->size[2] = (int)unnamed_idx_2;
  sol.pp->size[1] = 10;
  k = (int)delta1;
  sol.pp->size[0] = (int)delta1;
  emxEnsureCapacity_real_T(sol.pp, i);
  for (i = 0; i < k; i++) {
    for (i1 = 0; i1 < 10; i1++) {
      for (i2 = 0; i2 < loop_ub; i2++) {
        sol.pp->data[(i2 + sol.pp->size[2] * i1) + sol.pp->size[2] * 10 * i] =
          0.0;
      }
    }
  }

  delta1 = sol.N_segments;
  i = (int)delta1;
  for (b_i = 0; b_i < i; b_i++) {
    delta1 = sol.n_dim;
    i1 = (int)delta1;
    for (k = 0; k < i1; k++) {
      loop_ub = (int)sol.n_dim_ids->data[k];
      for (i2 = 0; i2 < 10; i2++) {
        sol.pp->data[((loop_ub + sol.pp->size[2] * i2) + sol.pp->size[2] * 10 *
                      b_i) - 1] = ppMatrix_data[(k + ppMatrix->size[2] * (9 - i2))
          + ppMatrix->size[2] * 10 * b_i];
      }
    }
  }

  emxFree_real_T(&ppMatrix);
  sol.J = J;
  sol.cost_is_good = true;
  stats_Iterations = sol.Iterations;
  stats_ExitFlag = sol.ExitFlag;
  is_good = false;
  y = sol.cost_is_good;
  if (y && ((fabs(sol.ExitFlag) < DBL_EPSILON) || (fabs(sol.ExitFlag - 2.0) < DBL_EPSILON))) {
    is_good = true;
  }

  if (is_good) {
    stats_J = sol.J;
  } else {
    stats_J = 1.7976931348623157E+308;
  }

  minsnap_set_n_dim = sol.n_dim;
  minsnap_set_n_dim_src = sol.n_dim_src;
  minsnap_set_N_segments = sol.N_segments;
  i = pp->size[0] * pp->size[1] * pp->size[2];
  pp->size[2] = sol.pp->size[2];
  pp->size[1] = 10;
  pp->size[0] = sol.pp->size[0];
  emxEnsureCapacity_real_T(pp, i);
  pp_data = pp->data;
  loop_ub = sol.pp->size[0];
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < 10; i1++) {
      k = sol.pp->size[2];
      for (i2 = 0; i2 < k; i2++) {
        pp_data[(i2 + pp->size[2] * i1) + pp->size[2] * 10 * i] = sol.pp->data
          [(i2 + sol.pp->size[2] * i1) + sol.pp->size[2] * 10 * i];
      }
    }
  }

  i = T->size[0] * T->size[1];
  T->size[1] = sol.timeOfArrival->size[1];
  T->size[0] = 1;
  emxEnsureCapacity_real_T(T, i);
  T_data = T->data;
  loop_ub = sol.timeOfArrival->size[1];
  for (i = 0; i < loop_ub; i++) {
    T_data[i] = sol.timeOfArrival->data[i];
  }

  i = n_dim_ids->size[0];
  n_dim_ids->size[0] = sol.n_dim_ids->size[0];
  emxEnsureCapacity_real_T(n_dim_ids, i);
  n_dim_ids_data = n_dim_ids->data;
  loop_ub = sol.n_dim_ids->size[0];
  for (i = 0; i < loop_ub; i++) {
    n_dim_ids_data[i] = sol.n_dim_ids->data[i];
  }

  i = offset->size[0];
  offset->size[0] = sol.waypoints_offset->size[0];
  emxEnsureCapacity_real_T(offset, i);
  offset_data = offset->data;
  loop_ub = sol.waypoints_offset->size[0];
  for (i = 0; i < loop_ub; i++) {
    offset_data[i] = sol.waypoints_offset->data[i];
  }

  if (ShowDetails) {
    i = (int)minsnap_set_N_segments;
    if ((int)minsnap_set_N_segments - 1 >= 0) {
      i3 = (int)minsnap_set_n_dim;
    }

    for (i_seg = 0; i_seg < i; i_seg++) {
      if ((unsigned int)i_seg + 1U < 2147483648U) {
        i1 = i_seg + 1;
      } else {
        i1 = MAX_int32_T;
      }

      printf("Segment %i:\n", i1);
      fflush(stdout);
      if ((unsigned int)i_seg + 1U < 2147483648U) {
        i1 = i_seg + 1;
      } else {
        i1 = MAX_int32_T;
      }

      if ((unsigned int)i_seg + 2U < 2147483648U) {
        i2 = (int)(((double)i_seg + 1.0) + 1.0);
      } else {
        i2 = MAX_int32_T;
      }

      printf("T[%i]->T[%i] = %f s -> %f s\n", i1, i2, sol.timeOfArrival->
             data[i_seg], sol.timeOfArrival->data[i_seg + 1]);
      fflush(stdout);
      for (i_dim = 0; i_dim < i3; i_dim++) {
        if ((unsigned int)i_dim + 1U < 2147483648U) {
          i1 = i_dim + 1;
        } else {
          i1 = MAX_int32_T;
        }

        printf("pp[dim=%i] = [ ", i1);
        fflush(stdout);
        for (i_coefs = 0; i_coefs < 10; i_coefs++) {
          printf("%f ", sol.pp->data[(i_dim + sol.pp->size[2] * i_coefs) +
                 sol.pp->size[2] * 10 * i_seg]);
          fflush(stdout);
        }

        printf("]\n");
        fflush(stdout);
      }

      printf("\n");
      fflush(stdout);
    }

    i = (int)minsnap_set_n_dim_src;
    for (i_dim = 0; i_dim < i; i_dim++) {
      if ((unsigned int)i_dim + 1U < 2147483648U) {
        i1 = i_dim + 1;
      } else {
        i1 = MAX_int32_T;
      }

      printf("offset[%i] = %f\n", i1, sol.waypoints_offset->data[i_dim]);
      fflush(stdout);
    }
  }

  *n_dim = minsnap_set_n_dim;
  *N_segments = minsnap_set_N_segments;
  *Cost = stats_J;
  *ExitFlag = stats_ExitFlag;
  *Iterations = stats_Iterations;
  c_emxFreeStruct_TrajectoryOptim(&sol);
}

void solver_codegen_initialize(void)
{
  freq_not_empty = false;
  isInitialized_solver_codegen = true;
}

void solver_codegen_terminate(void)
{
  isInitialized_solver_codegen = false;
}
