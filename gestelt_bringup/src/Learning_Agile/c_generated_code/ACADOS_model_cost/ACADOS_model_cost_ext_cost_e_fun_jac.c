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
  #define CASADI_PREFIX(ID) ACADOS_model_cost_ext_cost_e_fun_jac_ ## ID
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

static const casadi_int casadi_s0[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s1[3] = {0, 0, 0};
static const casadi_int casadi_s2[28] = {24, 1, 0, 24, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};
static const casadi_int casadi_s3[5] = {1, 1, 0, 1, 0};

/* ACADOS_model_cost_ext_cost_e_fun_jac:(i0[10],i1[],i2[],i3[24])->(o0,o1[10]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a4, a5, a6, a7, a8, a9;
  a0=500.;
  a1=arg[0]? arg[0][0] : 0;
  a2=arg[3]? arg[3][0] : 0;
  a1=(a1-a2);
  a2=casadi_sq(a1);
  a3=arg[0]? arg[0][1] : 0;
  a4=arg[3]? arg[3][1] : 0;
  a3=(a3-a4);
  a4=casadi_sq(a3);
  a2=(a2+a4);
  a4=arg[0]? arg[0][2] : 0;
  a5=arg[3]? arg[3][2] : 0;
  a4=(a4-a5);
  a5=casadi_sq(a4);
  a2=(a2+a5);
  a2=(a0*a2);
  a5=100.;
  a6=arg[0]? arg[0][3] : 0;
  a7=arg[3]? arg[3][3] : 0;
  a6=(a6-a7);
  a7=casadi_sq(a6);
  a8=arg[0]? arg[0][4] : 0;
  a9=arg[3]? arg[3][4] : 0;
  a8=(a8-a9);
  a9=casadi_sq(a8);
  a7=(a7+a9);
  a9=arg[0]? arg[0][5] : 0;
  a10=arg[3]? arg[3][5] : 0;
  a9=(a9-a10);
  a10=casadi_sq(a9);
  a7=(a7+a10);
  a7=(a5*a7);
  a2=(a2+a7);
  a7=1.;
  a10=2.;
  a11=arg[0]? arg[0][8] : 0;
  a12=casadi_sq(a11);
  a13=arg[0]? arg[0][9] : 0;
  a14=casadi_sq(a13);
  a12=(a12+a14);
  a12=(a10*a12);
  a12=(a7-a12);
  a14=arg[3]? arg[3][8] : 0;
  a15=casadi_sq(a14);
  a16=arg[3]? arg[3][9] : 0;
  a17=casadi_sq(a16);
  a15=(a15+a17);
  a15=(a10*a15);
  a15=(a7-a15);
  a12=(a12-a15);
  a15=casadi_sq(a12);
  a17=arg[0]? arg[0][7] : 0;
  a18=(a17*a11);
  a19=arg[0]? arg[0][6] : 0;
  a20=(a19*a13);
  a18=(a18+a20);
  a18=(a10*a18);
  a20=arg[3]? arg[3][7] : 0;
  a21=(a20*a14);
  a22=arg[3]? arg[3][6] : 0;
  a23=(a22*a16);
  a21=(a21+a23);
  a21=(a10*a21);
  a18=(a18-a21);
  a21=casadi_sq(a18);
  a15=(a15+a21);
  a21=(a17*a13);
  a23=(a19*a11);
  a21=(a21-a23);
  a21=(a10*a21);
  a23=(a20*a16);
  a24=(a22*a14);
  a23=(a23-a24);
  a23=(a10*a23);
  a21=(a21-a23);
  a23=casadi_sq(a21);
  a15=(a15+a23);
  a23=(a17*a11);
  a24=(a19*a13);
  a23=(a23-a24);
  a23=(a10*a23);
  a24=(a20*a14);
  a25=(a22*a16);
  a24=(a24-a25);
  a24=(a10*a24);
  a23=(a23-a24);
  a24=casadi_sq(a23);
  a25=casadi_sq(a17);
  a26=casadi_sq(a13);
  a25=(a25+a26);
  a25=(a10*a25);
  a25=(a7-a25);
  a26=casadi_sq(a20);
  a27=casadi_sq(a16);
  a26=(a26+a27);
  a26=(a10*a26);
  a26=(a7-a26);
  a25=(a25-a26);
  a26=casadi_sq(a25);
  a24=(a24+a26);
  a26=(a11*a13);
  a27=(a19*a17);
  a26=(a26+a27);
  a26=(a10*a26);
  a27=(a14*a16);
  a28=(a22*a20);
  a27=(a27+a28);
  a27=(a10*a27);
  a26=(a26-a27);
  a27=casadi_sq(a26);
  a24=(a24+a27);
  a15=(a15+a24);
  a24=(a17*a13);
  a27=(a19*a11);
  a24=(a24+a27);
  a24=(a10*a24);
  a27=(a20*a16);
  a28=(a22*a14);
  a27=(a27+a28);
  a27=(a10*a27);
  a24=(a24-a27);
  a27=casadi_sq(a24);
  a28=(a11*a13);
  a29=(a19*a17);
  a28=(a28-a29);
  a28=(a10*a28);
  a16=(a14*a16);
  a22=(a22*a20);
  a16=(a16-a22);
  a16=(a10*a16);
  a28=(a28-a16);
  a16=casadi_sq(a28);
  a27=(a27+a16);
  a16=casadi_sq(a17);
  a22=casadi_sq(a11);
  a16=(a16+a22);
  a16=(a10*a16);
  a16=(a7-a16);
  a20=casadi_sq(a20);
  a14=casadi_sq(a14);
  a20=(a20+a14);
  a20=(a10*a20);
  a7=(a7-a20);
  a16=(a16-a7);
  a7=casadi_sq(a16);
  a27=(a27+a7);
  a15=(a15+a27);
  a15=(a5*a15);
  a2=(a2+a15);
  if (res[0]!=0) res[0][0]=a2;
  a1=(a1+a1);
  a1=(a0*a1);
  if (res[1]!=0) res[1][0]=a1;
  a3=(a3+a3);
  a3=(a0*a3);
  if (res[1]!=0) res[1][1]=a3;
  a4=(a4+a4);
  a0=(a0*a4);
  if (res[1]!=0) res[1][2]=a0;
  a6=(a6+a6);
  a6=(a5*a6);
  if (res[1]!=0) res[1][3]=a6;
  a8=(a8+a8);
  a8=(a5*a8);
  if (res[1]!=0) res[1][4]=a8;
  a9=(a9+a9);
  a9=(a5*a9);
  if (res[1]!=0) res[1][5]=a9;
  a24=(a24+a24);
  a24=(a5*a24);
  a24=(a10*a24);
  a9=(a11*a24);
  a28=(a28+a28);
  a28=(a5*a28);
  a28=(a10*a28);
  a8=(a17*a28);
  a9=(a9-a8);
  a26=(a26+a26);
  a26=(a5*a26);
  a26=(a10*a26);
  a8=(a17*a26);
  a9=(a9+a8);
  a23=(a23+a23);
  a23=(a5*a23);
  a23=(a10*a23);
  a8=(a13*a23);
  a9=(a9-a8);
  a21=(a21+a21);
  a21=(a5*a21);
  a21=(a10*a21);
  a8=(a11*a21);
  a9=(a9-a8);
  a18=(a18+a18);
  a18=(a5*a18);
  a18=(a10*a18);
  a8=(a13*a18);
  a9=(a9+a8);
  if (res[1]!=0) res[1][6]=a9;
  a9=(a13*a24);
  a8=(a17+a17);
  a16=(a16+a16);
  a16=(a5*a16);
  a16=(a10*a16);
  a8=(a8*a16);
  a6=(a19*a28);
  a8=(a8+a6);
  a9=(a9-a8);
  a8=(a19*a26);
  a9=(a9+a8);
  a8=(a17+a17);
  a25=(a25+a25);
  a25=(a5*a25);
  a25=(a10*a25);
  a8=(a8*a25);
  a9=(a9-a8);
  a8=(a11*a23);
  a9=(a9+a8);
  a8=(a13*a21);
  a9=(a9+a8);
  a8=(a11*a18);
  a9=(a9+a8);
  if (res[1]!=0) res[1][7]=a9;
  a9=(a13*a28);
  a8=(a11+a11);
  a8=(a8*a16);
  a9=(a9-a8);
  a8=(a19*a24);
  a9=(a9+a8);
  a8=(a13*a26);
  a9=(a9+a8);
  a8=(a17*a23);
  a9=(a9+a8);
  a8=(a19*a21);
  a9=(a9-a8);
  a8=(a17*a18);
  a9=(a9+a8);
  a8=(a11+a11);
  a12=(a12+a12);
  a5=(a5*a12);
  a10=(a10*a5);
  a8=(a8*a10);
  a9=(a9-a8);
  if (res[1]!=0) res[1][8]=a9;
  a28=(a11*a28);
  a24=(a17*a24);
  a28=(a28+a24);
  a11=(a11*a26);
  a28=(a28+a11);
  a11=(a13+a13);
  a11=(a11*a25);
  a28=(a28-a11);
  a23=(a19*a23);
  a28=(a28-a23);
  a17=(a17*a21);
  a28=(a28+a17);
  a19=(a19*a18);
  a28=(a28+a19);
  a13=(a13+a13);
  a13=(a13*a10);
  a28=(a28-a13);
  if (res[1]!=0) res[1][9]=a28;
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_e_fun_jac(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_e_fun_jac_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_e_fun_jac_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_e_fun_jac_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_e_fun_jac_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_e_fun_jac_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_e_fun_jac_incref(void) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_e_fun_jac_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_e_fun_jac_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_e_fun_jac_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real ACADOS_model_cost_ext_cost_e_fun_jac_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_e_fun_jac_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_e_fun_jac_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_e_fun_jac_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_e_fun_jac_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_e_fun_jac_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
