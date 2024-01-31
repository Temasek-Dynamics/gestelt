/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) ACADOS_model_cost_ext_cost_fun_jac_hess_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_sq CASADI_PREFIX(sq)

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

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[21] = {17, 1, 0, 17, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s5[33] = {17, 17, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 10, 10, 10, 11, 12, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 14, 15, 16};
static const casadi_int casadi_s6[20] = {0, 17, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* ACADOS_model_cost_ext_cost_fun_jac_hess:(i0[13],i1[4],i2[],i3[17])->(o0,o1[17],o2[17x17,13nz],o3[],o4[0x17]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a3, a4, a5, a6, a7, a8, a9;
  a0=5.;
  a1=arg[0]? arg[0][0] : 0;
  a2=2.;
  a1=(a1-a2);
  a2=casadi_sq(a1);
  a3=arg[0]? arg[0][1] : 0;
  a4=-1.8000000000000000e+00;
  a3=(a3-a4);
  a4=casadi_sq(a3);
  a2=(a2+a4);
  a4=arg[0]? arg[0][2] : 0;
  a5=1.;
  a4=(a4-a5);
  a5=casadi_sq(a4);
  a2=(a2+a5);
  a5=(a0*a2);
  a6=arg[0]? arg[0][3] : 0;
  a7=casadi_sq(a6);
  a8=arg[0]? arg[0][4] : 0;
  a9=casadi_sq(a8);
  a7=(a7+a9);
  a9=arg[0]? arg[0][5] : 0;
  a10=casadi_sq(a9);
  a7=(a7+a10);
  a10=(a0*a7);
  a5=(a5+a10);
  a10=3.;
  a11=arg[0]? arg[0][10] : 0;
  a12=casadi_sq(a11);
  a13=arg[0]? arg[0][11] : 0;
  a14=casadi_sq(a13);
  a12=(a12+a14);
  a14=arg[0]? arg[0][12] : 0;
  a15=casadi_sq(a14);
  a12=(a12+a15);
  a15=(a10*a12);
  a5=(a5+a15);
  a15=1.0000000000000001e-01;
  a16=arg[1]? arg[1][0] : 0;
  a17=casadi_sq(a16);
  a18=arg[1]? arg[1][1] : 0;
  a19=casadi_sq(a18);
  a17=(a17+a19);
  a19=arg[1]? arg[1][2] : 0;
  a20=casadi_sq(a19);
  a17=(a17+a20);
  a20=arg[1]? arg[1][3] : 0;
  a21=casadi_sq(a20);
  a17=(a17+a21);
  a17=(a15*a17);
  a5=(a5+a17);
  a2=(a0*a2);
  a0=(a0*a7);
  a2=(a2+a0);
  a10=(a10*a12);
  a2=(a2+a10);
  a5=(a5+a2);
  if (res[0]!=0) res[0][0]=a5;
  a16=(a16+a16);
  a16=(a15*a16);
  if (res[1]!=0) res[1][0]=a16;
  a18=(a18+a18);
  a18=(a15*a18);
  if (res[1]!=0) res[1][1]=a18;
  a19=(a19+a19);
  a19=(a15*a19);
  if (res[1]!=0) res[1][2]=a19;
  a20=(a20+a20);
  a15=(a15*a20);
  if (res[1]!=0) res[1][3]=a15;
  a15=10.;
  a1=(a1+a1);
  a1=(a15*a1);
  if (res[1]!=0) res[1][4]=a1;
  a3=(a3+a3);
  a3=(a15*a3);
  if (res[1]!=0) res[1][5]=a3;
  a4=(a4+a4);
  a4=(a15*a4);
  if (res[1]!=0) res[1][6]=a4;
  a6=(a6+a6);
  a6=(a15*a6);
  if (res[1]!=0) res[1][7]=a6;
  a8=(a8+a8);
  a8=(a15*a8);
  if (res[1]!=0) res[1][8]=a8;
  a9=(a9+a9);
  a15=(a15*a9);
  if (res[1]!=0) res[1][9]=a15;
  a15=0.;
  if (res[1]!=0) res[1][10]=a15;
  if (res[1]!=0) res[1][11]=a15;
  if (res[1]!=0) res[1][12]=a15;
  if (res[1]!=0) res[1][13]=a15;
  a15=6.;
  a11=(a11+a11);
  a11=(a15*a11);
  if (res[1]!=0) res[1][14]=a11;
  a13=(a13+a13);
  a13=(a15*a13);
  if (res[1]!=0) res[1][15]=a13;
  a14=(a14+a14);
  a15=(a15*a14);
  if (res[1]!=0) res[1][16]=a15;
  a15=2.0000000000000001e-01;
  if (res[2]!=0) res[2][0]=a15;
  if (res[2]!=0) res[2][1]=a15;
  if (res[2]!=0) res[2][2]=a15;
  if (res[2]!=0) res[2][3]=a15;
  a15=20.;
  if (res[2]!=0) res[2][4]=a15;
  if (res[2]!=0) res[2][5]=a15;
  if (res[2]!=0) res[2][6]=a15;
  if (res[2]!=0) res[2][7]=a15;
  if (res[2]!=0) res[2][8]=a15;
  if (res[2]!=0) res[2][9]=a15;
  a15=12.;
  if (res[2]!=0) res[2][10]=a15;
  if (res[2]!=0) res[2][11]=a15;
  if (res[2]!=0) res[2][12]=a15;
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_jac_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_jac_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_jac_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_fun_jac_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_jac_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_fun_jac_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_fun_jac_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_fun_jac_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_fun_jac_hess_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_fun_jac_hess_n_out(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_real ACADOS_model_cost_ext_cost_fun_jac_hess_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_fun_jac_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_fun_jac_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    case 4: return "o4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_fun_jac_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_fun_jac_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s3;
    case 2: return casadi_s5;
    case 3: return casadi_s2;
    case 4: return casadi_s6;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_fun_jac_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 5;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
