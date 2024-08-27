/* This file was automatically generated by CasADi 3.6.5.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) quadrotor_cost_y_e_fun_jac_ut_xt_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

static const casadi_int casadi_s0[24] = {20, 1, 0, 20, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
static const casadi_int casadi_s1[3] = {0, 0, 0};
static const casadi_int casadi_s2[43] = {20, 20, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
static const casadi_int casadi_s3[3] = {20, 0, 0};

/* quadrotor_cost_y_e_fun_jac_ut_xt:(i0[20],i1[],i2[],i3[],i4[])->(o0[20],o1[20x20,20nz],o2[20x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real *rr, *ss;
  casadi_real w0, *w1=w+1, *w2=w+21;
  /* #0: @0 = input[0][0] */
  w0 = arg[0] ? arg[0][0] : 0;
  /* #1: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #2: @0 = input[0][1] */
  w0 = arg[0] ? arg[0][1] : 0;
  /* #3: output[0][1] = @0 */
  if (res[0]) res[0][1] = w0;
  /* #4: @0 = input[0][2] */
  w0 = arg[0] ? arg[0][2] : 0;
  /* #5: output[0][2] = @0 */
  if (res[0]) res[0][2] = w0;
  /* #6: @0 = input[0][3] */
  w0 = arg[0] ? arg[0][3] : 0;
  /* #7: output[0][3] = @0 */
  if (res[0]) res[0][3] = w0;
  /* #8: @0 = input[0][4] */
  w0 = arg[0] ? arg[0][4] : 0;
  /* #9: output[0][4] = @0 */
  if (res[0]) res[0][4] = w0;
  /* #10: @0 = input[0][5] */
  w0 = arg[0] ? arg[0][5] : 0;
  /* #11: output[0][5] = @0 */
  if (res[0]) res[0][5] = w0;
  /* #12: @0 = input[0][6] */
  w0 = arg[0] ? arg[0][6] : 0;
  /* #13: output[0][6] = @0 */
  if (res[0]) res[0][6] = w0;
  /* #14: @0 = input[0][7] */
  w0 = arg[0] ? arg[0][7] : 0;
  /* #15: output[0][7] = @0 */
  if (res[0]) res[0][7] = w0;
  /* #16: @0 = input[0][8] */
  w0 = arg[0] ? arg[0][8] : 0;
  /* #17: output[0][8] = @0 */
  if (res[0]) res[0][8] = w0;
  /* #18: @0 = input[0][9] */
  w0 = arg[0] ? arg[0][9] : 0;
  /* #19: output[0][9] = @0 */
  if (res[0]) res[0][9] = w0;
  /* #20: @0 = input[0][10] */
  w0 = arg[0] ? arg[0][10] : 0;
  /* #21: output[0][10] = @0 */
  if (res[0]) res[0][10] = w0;
  /* #22: @0 = input[0][11] */
  w0 = arg[0] ? arg[0][11] : 0;
  /* #23: output[0][11] = @0 */
  if (res[0]) res[0][11] = w0;
  /* #24: @0 = input[0][12] */
  w0 = arg[0] ? arg[0][12] : 0;
  /* #25: output[0][12] = @0 */
  if (res[0]) res[0][12] = w0;
  /* #26: @0 = input[0][13] */
  w0 = arg[0] ? arg[0][13] : 0;
  /* #27: output[0][13] = @0 */
  if (res[0]) res[0][13] = w0;
  /* #28: @0 = input[0][14] */
  w0 = arg[0] ? arg[0][14] : 0;
  /* #29: output[0][14] = @0 */
  if (res[0]) res[0][14] = w0;
  /* #30: @0 = input[0][15] */
  w0 = arg[0] ? arg[0][15] : 0;
  /* #31: output[0][15] = @0 */
  if (res[0]) res[0][15] = w0;
  /* #32: @0 = input[0][16] */
  w0 = arg[0] ? arg[0][16] : 0;
  /* #33: output[0][16] = @0 */
  if (res[0]) res[0][16] = w0;
  /* #34: @0 = input[0][17] */
  w0 = arg[0] ? arg[0][17] : 0;
  /* #35: output[0][17] = @0 */
  if (res[0]) res[0][17] = w0;
  /* #36: @0 = input[0][18] */
  w0 = arg[0] ? arg[0][18] : 0;
  /* #37: output[0][18] = @0 */
  if (res[0]) res[0][18] = w0;
  /* #38: @0 = input[0][19] */
  w0 = arg[0] ? arg[0][19] : 0;
  /* #39: output[0][19] = @0 */
  if (res[0]) res[0][19] = w0;
  /* #40: @1 = zeros(20x20,20nz) */
  casadi_clear(w1, 20);
  /* #41: @2 = ones(20x1) */
  casadi_fill(w2, 20, 1.);
  /* #42: (@1[:20] = @2) */
  for (rr=w1+0, ss=w2; rr!=w1+20; rr+=1) *rr = *ss++;
  /* #43: output[1][0] = @1 */
  casadi_copy(w1, 20, res[1]);
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_y_e_fun_jac_ut_xt(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_y_e_fun_jac_ut_xt_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_y_e_fun_jac_ut_xt_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_y_e_fun_jac_ut_xt_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_y_e_fun_jac_ut_xt_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_y_e_fun_jac_ut_xt_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_y_e_fun_jac_ut_xt_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_cost_y_e_fun_jac_ut_xt_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_y_e_fun_jac_ut_xt_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_cost_y_e_fun_jac_ut_xt_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_cost_y_e_fun_jac_ut_xt_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_y_e_fun_jac_ut_xt_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_cost_y_e_fun_jac_ut_xt_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_y_e_fun_jac_ut_xt_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s1;
    case 4: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_cost_y_e_fun_jac_ut_xt_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s2;
    case 2: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_y_e_fun_jac_ut_xt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 7;
  if (sz_res) *sz_res = 4;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 41;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_cost_y_e_fun_jac_ut_xt_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 7*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 4*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 41*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
