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
  #define CASADI_PREFIX(ID) ACADOS_model_cost_ext_cost_0_fun_jac_hess_ ## ID
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
#define casadi_s7 CASADI_PREFIX(s7)
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
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[22] = {18, 1, 0, 18, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
static const casadi_int casadi_s4[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s5[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s6[42] = {14, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 17, 21, 25, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 10, 11, 12, 13, 10, 11, 12, 13, 10, 11, 12, 13};
static const casadi_int casadi_s7[17] = {0, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* ACADOS_model_cost_ext_cost_0_fun_jac_hess:(i0[10],i1[4],i2[],i3[18])->(o0,o1[14],o2[14x14,25nz],o3[],o4[0x14]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a5, a6, a7, a8, a9;
  a0=10.;
  a1=arg[0]? arg[0][0] : 0;
  a2=arg[3]? arg[3][0] : 0;
  a2=(a1-a2);
  a3=casadi_sq(a2);
  a4=arg[0]? arg[0][1] : 0;
  a5=arg[3]? arg[3][1] : 0;
  a5=(a4-a5);
  a6=casadi_sq(a5);
  a3=(a3+a6);
  a6=arg[0]? arg[0][2] : 0;
  a7=arg[3]? arg[3][2] : 0;
  a7=(a6-a7);
  a8=casadi_sq(a7);
  a3=(a3+a8);
  a3=(a0*a3);
  a8=1.0000000000000000e-02;
  a9=arg[0]? arg[0][3] : 0;
  a10=arg[3]? arg[3][3] : 0;
  a9=(a9-a10);
  a10=casadi_sq(a9);
  a11=arg[0]? arg[0][4] : 0;
  a12=arg[3]? arg[3][4] : 0;
  a11=(a11-a12);
  a12=casadi_sq(a11);
  a10=(a10+a12);
  a12=arg[0]? arg[0][5] : 0;
  a13=arg[3]? arg[3][5] : 0;
  a12=(a12-a13);
  a13=casadi_sq(a12);
  a10=(a10+a13);
  a10=(a8*a10);
  a3=(a3+a10);
  a10=1.;
  a13=2.;
  a14=arg[3]? arg[3][8] : 0;
  a15=casadi_sq(a14);
  a16=arg[3]? arg[3][9] : 0;
  a17=casadi_sq(a16);
  a15=(a15+a17);
  a15=(a13*a15);
  a15=(a10-a15);
  a17=arg[0]? arg[0][8] : 0;
  a18=casadi_sq(a17);
  a19=arg[0]? arg[0][9] : 0;
  a20=casadi_sq(a19);
  a18=(a18+a20);
  a18=(a13*a18);
  a18=(a10-a18);
  a18=(a15*a18);
  a20=arg[3]? arg[3][7] : 0;
  a21=(a20*a14);
  a22=arg[3]? arg[3][6] : 0;
  a23=(a22*a16);
  a21=(a21-a23);
  a21=(a13*a21);
  a23=arg[0]? arg[0][7] : 0;
  a24=(a23*a17);
  a25=arg[0]? arg[0][6] : 0;
  a26=(a25*a19);
  a24=(a24-a26);
  a24=(a13*a24);
  a24=(a21*a24);
  a18=(a18+a24);
  a24=(a20*a16);
  a26=(a22*a14);
  a24=(a24+a26);
  a24=(a13*a24);
  a26=(a23*a19);
  a27=(a25*a17);
  a26=(a26+a27);
  a26=(a13*a26);
  a26=(a24*a26);
  a18=(a18+a26);
  a18=(a10-a18);
  a26=(a20*a14);
  a27=(a22*a16);
  a26=(a26+a27);
  a26=(a13*a26);
  a27=(a23*a17);
  a28=(a25*a19);
  a27=(a27+a28);
  a27=(a13*a27);
  a27=(a26*a27);
  a28=casadi_sq(a20);
  a29=casadi_sq(a16);
  a28=(a28+a29);
  a28=(a13*a28);
  a28=(a10-a28);
  a29=casadi_sq(a23);
  a30=casadi_sq(a19);
  a29=(a29+a30);
  a29=(a13*a29);
  a29=(a10-a29);
  a29=(a28*a29);
  a27=(a27+a29);
  a29=(a14*a16);
  a30=(a22*a20);
  a29=(a29-a30);
  a29=(a13*a29);
  a30=(a17*a19);
  a31=(a25*a23);
  a30=(a30-a31);
  a30=(a13*a30);
  a30=(a29*a30);
  a27=(a27+a30);
  a27=(a10-a27);
  a18=(a18+a27);
  a27=(a20*a16);
  a30=(a22*a14);
  a27=(a27-a30);
  a27=(a13*a27);
  a30=(a23*a19);
  a31=(a25*a17);
  a30=(a30-a31);
  a30=(a13*a30);
  a30=(a27*a30);
  a16=(a14*a16);
  a22=(a22*a20);
  a16=(a16+a22);
  a16=(a13*a16);
  a22=(a17*a19);
  a31=(a25*a23);
  a22=(a22+a31);
  a22=(a13*a22);
  a22=(a16*a22);
  a30=(a30+a22);
  a20=casadi_sq(a20);
  a14=casadi_sq(a14);
  a20=(a20+a14);
  a20=(a13*a20);
  a20=(a10-a20);
  a14=casadi_sq(a23);
  a22=casadi_sq(a17);
  a14=(a14+a22);
  a14=(a13*a14);
  a14=(a10-a14);
  a14=(a20*a14);
  a30=(a30+a14);
  a30=(a10-a30);
  a18=(a18+a30);
  a18=(a8*a18);
  a3=(a3+a18);
  a18=100.;
  a30=-10.;
  a14=arg[3]? arg[3][17] : 0;
  a22=arg[3]? arg[3][16] : 0;
  a14=(a14-a22);
  a14=casadi_sq(a14);
  a30=(a30*a14);
  a30=exp(a30);
  a30=(a18*a30);
  a14=5.;
  a22=arg[3]? arg[3][10] : 0;
  a1=(a1-a22);
  a22=casadi_sq(a1);
  a31=arg[3]? arg[3][11] : 0;
  a4=(a4-a31);
  a31=casadi_sq(a4);
  a22=(a22+a31);
  a31=arg[3]? arg[3][12] : 0;
  a6=(a6-a31);
  a31=casadi_sq(a6);
  a22=(a22+a31);
  a22=(a14*a22);
  a31=arg[3]? arg[3][13] : 0;
  a32=casadi_sq(a31);
  a33=arg[3]? arg[3][14] : 0;
  a34=casadi_sq(a33);
  a32=(a32+a34);
  a34=arg[3]? arg[3][15] : 0;
  a35=casadi_sq(a34);
  a32=(a32+a35);
  a32=sqrt(a32);
  a32=atan(a32);
  a35=sin(a32);
  a36=1.0000000000000000e-08;
  a37=(a31+a36);
  a37=casadi_sq(a37);
  a38=casadi_sq(a33);
  a37=(a37+a38);
  a38=casadi_sq(a34);
  a37=(a37+a38);
  a37=sqrt(a37);
  a36=(a36/a37);
  a31=(a31+a36);
  a36=casadi_sq(a31);
  a37=casadi_sq(a33);
  a36=(a36+a37);
  a37=casadi_sq(a34);
  a36=(a36+a37);
  a36=sqrt(a36);
  a33=(a33/a36);
  a33=(a35*a33);
  a37=casadi_sq(a33);
  a34=(a34/a36);
  a34=(a35*a34);
  a38=casadi_sq(a34);
  a37=(a37+a38);
  a37=(a13*a37);
  a37=(a10-a37);
  a38=casadi_sq(a17);
  a39=casadi_sq(a19);
  a38=(a38+a39);
  a38=(a13*a38);
  a38=(a10-a38);
  a38=(a37*a38);
  a31=(a31/a36);
  a35=(a35*a31);
  a31=(a35*a33);
  a32=cos(a32);
  a36=(a32*a34);
  a31=(a31-a36);
  a31=(a13*a31);
  a36=(a23*a17);
  a39=(a25*a19);
  a36=(a36-a39);
  a36=(a13*a36);
  a36=(a31*a36);
  a38=(a38+a36);
  a36=(a35*a34);
  a39=(a32*a33);
  a36=(a36+a39);
  a36=(a13*a36);
  a39=(a23*a19);
  a40=(a25*a17);
  a39=(a39+a40);
  a39=(a13*a39);
  a39=(a36*a39);
  a38=(a38+a39);
  a38=(a10-a38);
  a39=(a35*a33);
  a40=(a32*a34);
  a39=(a39+a40);
  a39=(a13*a39);
  a40=(a23*a17);
  a41=(a25*a19);
  a40=(a40+a41);
  a40=(a13*a40);
  a40=(a39*a40);
  a41=casadi_sq(a35);
  a42=casadi_sq(a34);
  a41=(a41+a42);
  a41=(a13*a41);
  a41=(a10-a41);
  a42=casadi_sq(a23);
  a43=casadi_sq(a19);
  a42=(a42+a43);
  a42=(a13*a42);
  a42=(a10-a42);
  a42=(a41*a42);
  a40=(a40+a42);
  a42=(a33*a34);
  a43=(a32*a35);
  a42=(a42-a43);
  a42=(a13*a42);
  a43=(a17*a19);
  a44=(a25*a23);
  a43=(a43-a44);
  a43=(a13*a43);
  a43=(a42*a43);
  a40=(a40+a43);
  a40=(a10-a40);
  a38=(a38+a40);
  a40=(a35*a34);
  a43=(a32*a33);
  a40=(a40-a43);
  a40=(a13*a40);
  a43=(a23*a19);
  a44=(a25*a17);
  a43=(a43-a44);
  a43=(a13*a43);
  a43=(a40*a43);
  a34=(a33*a34);
  a32=(a32*a35);
  a34=(a34+a32);
  a34=(a13*a34);
  a32=(a17*a19);
  a44=(a25*a23);
  a32=(a32+a44);
  a32=(a13*a32);
  a32=(a34*a32);
  a43=(a43+a32);
  a35=casadi_sq(a35);
  a33=casadi_sq(a33);
  a35=(a35+a33);
  a35=(a13*a35);
  a35=(a10-a35);
  a33=casadi_sq(a23);
  a32=casadi_sq(a17);
  a33=(a33+a32);
  a33=(a13*a33);
  a33=(a10-a33);
  a33=(a35*a33);
  a43=(a43+a33);
  a10=(a10-a43);
  a38=(a38+a10);
  a38=(a0*a38);
  a22=(a22+a38);
  a22=(a30*a22);
  a3=(a3+a22);
  a22=arg[1]? arg[1][0] : 0;
  a38=casadi_sq(a22);
  a10=arg[1]? arg[1][1] : 0;
  a43=casadi_sq(a10);
  a33=arg[1]? arg[1][2] : 0;
  a32=casadi_sq(a33);
  a43=(a43+a32);
  a43=(a14*a43);
  a38=(a38+a43);
  a43=50.;
  a32=arg[1]? arg[1][3] : 0;
  a44=casadi_sq(a32);
  a44=(a43*a44);
  a38=(a38+a44);
  a3=(a3+a38);
  if (res[0]!=0) res[0][0]=a3;
  a22=(a22+a22);
  if (res[1]!=0) res[1][0]=a22;
  a10=(a10+a10);
  a10=(a14*a10);
  if (res[1]!=0) res[1][1]=a10;
  a33=(a33+a33);
  a33=(a14*a33);
  if (res[1]!=0) res[1][2]=a33;
  a32=(a32+a32);
  a43=(a43*a32);
  if (res[1]!=0) res[1][3]=a43;
  a1=(a1+a1);
  a14=(a14*a30);
  a1=(a1*a14);
  a2=(a2+a2);
  a2=(a0*a2);
  a1=(a1+a2);
  if (res[1]!=0) res[1][4]=a1;
  a4=(a4+a4);
  a4=(a4*a14);
  a5=(a5+a5);
  a5=(a0*a5);
  a4=(a4+a5);
  if (res[1]!=0) res[1][5]=a4;
  a6=(a6+a6);
  a6=(a6*a14);
  a7=(a7+a7);
  a7=(a0*a7);
  a6=(a6+a7);
  if (res[1]!=0) res[1][6]=a6;
  a9=(a9+a9);
  a9=(a8*a9);
  if (res[1]!=0) res[1][7]=a9;
  a11=(a11+a11);
  a11=(a8*a11);
  if (res[1]!=0) res[1][8]=a11;
  a12=(a12+a12);
  a8=(a8*a12);
  if (res[1]!=0) res[1][9]=a8;
  a30=(a0*a30);
  a40=(a40*a30);
  a40=(a13*a40);
  a8=(a17*a40);
  a34=(a34*a30);
  a34=(a13*a34);
  a12=(a23*a34);
  a8=(a8-a12);
  a42=(a42*a30);
  a42=(a13*a42);
  a12=(a23*a42);
  a8=(a8+a12);
  a39=(a39*a30);
  a39=(a13*a39);
  a12=(a19*a39);
  a8=(a8-a12);
  a36=(a36*a30);
  a36=(a13*a36);
  a12=(a17*a36);
  a8=(a8-a12);
  a31=(a31*a30);
  a31=(a13*a31);
  a12=(a19*a31);
  a8=(a8+a12);
  a12=-1.0000000000000000e-02;
  a16=(a12*a16);
  a16=(a13*a16);
  a11=(a23*a16);
  a8=(a8+a11);
  a27=(a12*a27);
  a27=(a13*a27);
  a11=(a17*a27);
  a8=(a8-a11);
  a29=(a12*a29);
  a29=(a13*a29);
  a11=(a23*a29);
  a8=(a8-a11);
  a26=(a12*a26);
  a26=(a13*a26);
  a11=(a19*a26);
  a8=(a8+a11);
  a24=(a12*a24);
  a24=(a13*a24);
  a11=(a17*a24);
  a8=(a8+a11);
  a21=(a12*a21);
  a21=(a13*a21);
  a11=(a19*a21);
  a8=(a8-a11);
  if (res[1]!=0) res[1][10]=a8;
  a8=(a23+a23);
  a35=(a35*a30);
  a35=(a13*a35);
  a8=(a8*a35);
  a11=(a25*a34);
  a8=(a8-a11);
  a11=(a19*a40);
  a8=(a8-a11);
  a11=(a25*a42);
  a8=(a8+a11);
  a11=(a23+a23);
  a41=(a41*a30);
  a41=(a13*a41);
  a11=(a11*a41);
  a8=(a8+a11);
  a11=(a17*a39);
  a8=(a8-a11);
  a11=(a19*a36);
  a8=(a8-a11);
  a11=(a17*a31);
  a8=(a8-a11);
  a11=(a23+a23);
  a20=(a12*a20);
  a20=(a13*a20);
  a11=(a11*a20);
  a8=(a8-a11);
  a11=(a25*a16);
  a8=(a8+a11);
  a11=(a19*a27);
  a8=(a8+a11);
  a11=(a25*a29);
  a8=(a8-a11);
  a11=(a23+a23);
  a28=(a12*a28);
  a28=(a13*a28);
  a11=(a11*a28);
  a8=(a8-a11);
  a11=(a17*a26);
  a8=(a8+a11);
  a11=(a19*a24);
  a8=(a8+a11);
  a11=(a17*a21);
  a8=(a8+a11);
  if (res[1]!=0) res[1][11]=a8;
  a8=(a17+a17);
  a8=(a8*a35);
  a11=(a19*a34);
  a8=(a8-a11);
  a11=(a25*a40);
  a8=(a8+a11);
  a11=(a19*a42);
  a8=(a8-a11);
  a11=(a23*a39);
  a8=(a8-a11);
  a11=(a25*a36);
  a8=(a8-a11);
  a11=(a23*a31);
  a8=(a8-a11);
  a11=(a17+a17);
  a37=(a37*a30);
  a37=(a13*a37);
  a11=(a11*a37);
  a8=(a8+a11);
  a11=(a17+a17);
  a11=(a11*a20);
  a8=(a8-a11);
  a11=(a19*a16);
  a8=(a8+a11);
  a11=(a25*a27);
  a8=(a8-a11);
  a11=(a19*a29);
  a8=(a8+a11);
  a11=(a23*a26);
  a8=(a8+a11);
  a11=(a25*a24);
  a8=(a8+a11);
  a11=(a23*a21);
  a8=(a8+a11);
  a11=(a17+a17);
  a12=(a12*a15);
  a12=(a13*a12);
  a11=(a11*a12);
  a8=(a8-a11);
  if (res[1]!=0) res[1][12]=a8;
  a8=(a19+a19);
  a8=(a8*a41);
  a11=(a17*a34);
  a15=(a23*a40);
  a11=(a11+a15);
  a15=(a17*a42);
  a11=(a11+a15);
  a8=(a8-a11);
  a11=(a25*a39);
  a8=(a8-a11);
  a11=(a23*a36);
  a8=(a8-a11);
  a11=(a25*a31);
  a8=(a8+a11);
  a11=(a19+a19);
  a11=(a11*a37);
  a8=(a8+a11);
  a11=(a17*a16);
  a8=(a8+a11);
  a11=(a23*a27);
  a8=(a8+a11);
  a17=(a17*a29);
  a8=(a8+a17);
  a17=(a19+a19);
  a17=(a17*a28);
  a8=(a8-a17);
  a17=(a25*a26);
  a8=(a8+a17);
  a23=(a23*a24);
  a8=(a8+a23);
  a25=(a25*a21);
  a8=(a8-a25);
  a19=(a19+a19);
  a19=(a19*a12);
  a8=(a8-a19);
  if (res[1]!=0) res[1][13]=a8;
  if (res[2]!=0) res[2][0]=a13;
  if (res[2]!=0) res[2][1]=a0;
  if (res[2]!=0) res[2][2]=a0;
  if (res[2]!=0) res[2][3]=a18;
  a18=(a13*a14);
  a0=20.;
  a18=(a18+a0);
  if (res[2]!=0) res[2][4]=a18;
  a18=(a13*a14);
  a18=(a18+a0);
  if (res[2]!=0) res[2][5]=a18;
  a14=(a13*a14);
  a14=(a14+a0);
  if (res[2]!=0) res[2][6]=a14;
  a14=2.0000000000000000e-02;
  if (res[2]!=0) res[2][7]=a14;
  if (res[2]!=0) res[2][8]=a14;
  if (res[2]!=0) res[2][9]=a14;
  a14=(a42-a34);
  a14=(a14+a16);
  a14=(a14-a29);
  if (res[2]!=0) res[2][10]=a14;
  a0=(a40-a36);
  a0=(a0-a27);
  a0=(a0+a24);
  if (res[2]!=0) res[2][11]=a0;
  a18=(a31-a39);
  a18=(a18+a26);
  a18=(a18-a21);
  if (res[2]!=0) res[2][12]=a18;
  if (res[2]!=0) res[2][13]=a14;
  a14=(a13*a35);
  a8=(a13*a41);
  a14=(a14+a8);
  a8=(a13*a20);
  a14=(a14-a8);
  a8=(a13*a28);
  a14=(a14-a8);
  if (res[2]!=0) res[2][14]=a14;
  a39=(a39+a31);
  a26=(a26-a39);
  a26=(a26+a21);
  if (res[2]!=0) res[2][15]=a26;
  a40=(a40+a36);
  a27=(a27-a40);
  a27=(a27+a24);
  if (res[2]!=0) res[2][16]=a27;
  if (res[2]!=0) res[2][17]=a0;
  if (res[2]!=0) res[2][18]=a26;
  a35=(a13*a35);
  a26=(a13*a37);
  a35=(a35+a26);
  a20=(a13*a20);
  a35=(a35-a20);
  a20=(a13*a12);
  a35=(a35-a20);
  if (res[2]!=0) res[2][19]=a35;
  a34=(a34+a42);
  a16=(a16-a34);
  a16=(a16+a29);
  if (res[2]!=0) res[2][20]=a16;
  if (res[2]!=0) res[2][21]=a18;
  if (res[2]!=0) res[2][22]=a27;
  if (res[2]!=0) res[2][23]=a16;
  a41=(a13*a41);
  a37=(a13*a37);
  a41=(a41+a37);
  a28=(a13*a28);
  a41=(a41-a28);
  a13=(a13*a12);
  a41=(a41-a13);
  if (res[2]!=0) res[2][24]=a41;
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_0_fun_jac_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_0_fun_jac_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_0_fun_jac_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_0_fun_jac_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_0_fun_jac_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_0_fun_jac_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_0_fun_jac_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void ACADOS_model_cost_ext_cost_0_fun_jac_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_0_fun_jac_hess_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int ACADOS_model_cost_ext_cost_0_fun_jac_hess_n_out(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_real ACADOS_model_cost_ext_cost_0_fun_jac_hess_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_0_fun_jac_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ACADOS_model_cost_ext_cost_0_fun_jac_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    case 4: return "o4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_0_fun_jac_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ACADOS_model_cost_ext_cost_0_fun_jac_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s5;
    case 2: return casadi_s6;
    case 3: return casadi_s2;
    case 4: return casadi_s7;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ACADOS_model_cost_ext_cost_0_fun_jac_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 5;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
