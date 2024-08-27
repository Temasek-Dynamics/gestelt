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
  #define CASADI_PREFIX(ID) quadrotor_constr_h_fun_jac_uxt_zt_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_c0 CASADI_PREFIX(c0)
#define casadi_c1 CASADI_PREFIX(c1)
#define casadi_c2 CASADI_PREFIX(c2)
#define casadi_c3 CASADI_PREFIX(c3)
#define casadi_c4 CASADI_PREFIX(c4)
#define casadi_c5 CASADI_PREFIX(c5)
#define casadi_c6 CASADI_PREFIX(c6)
#define casadi_c7 CASADI_PREFIX(c7)
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_clear_casadi_int CASADI_PREFIX(clear_casadi_int)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_de_boor CASADI_PREFIX(de_boor)
#define casadi_dot CASADI_PREFIX(dot)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_f2 CASADI_PREFIX(f2)
#define casadi_f3 CASADI_PREFIX(f3)
#define casadi_f4 CASADI_PREFIX(f4)
#define casadi_f5 CASADI_PREFIX(f5)
#define casadi_f6 CASADI_PREFIX(f6)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_fill_casadi_int CASADI_PREFIX(fill_casadi_int)
#define casadi_low CASADI_PREFIX(low)
#define casadi_nd_boor_eval CASADI_PREFIX(nd_boor_eval)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s10 CASADI_PREFIX(s10)
#define casadi_s11 CASADI_PREFIX(s11)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_s8 CASADI_PREFIX(s8)
#define casadi_s9 CASADI_PREFIX(s9)

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

void casadi_de_boor(casadi_real x, const casadi_real* knots, casadi_int n_knots, casadi_int degree, casadi_real* boor) {
  casadi_int d, i;
  for (d=1;d<degree+1;++d) {
    for (i=0;i<n_knots-d-1;++i) {
      casadi_real b, bottom;
      b = 0;
      bottom = knots[i + d] - knots[i];
      if (bottom) b = (x - knots[i]) * boor[i] / bottom;
      bottom = knots[i + d + 1] - knots[i + 1];
      if (bottom) b += (knots[i + d + 1] - x) * boor[i + 1] / bottom;
      boor[i] = b;
    }
  }
}

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

void casadi_fill_casadi_int(casadi_int* x, casadi_int n, casadi_int alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

void casadi_clear_casadi_int(casadi_int* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

casadi_int casadi_low(casadi_real x, const casadi_real* grid, casadi_int ng, casadi_int lookup_mode) {
  switch (lookup_mode) {
    case 1:
      {
        casadi_real g0, dg;
        casadi_int ret;
        g0 = grid[0];
        dg = grid[ng-1]-g0;
        ret = (casadi_int) ((x-g0)*(ng-1)/dg);
        if (ret<0) ret=0;
        if (ret>ng-2) ret=ng-2;
        return ret;
      }
    case 2:
      {
        casadi_int start, stop, pivot;
        if (ng<2 || x<grid[1]) return 0;
        if (x>grid[ng-1]) return ng-2;
        start = 0;
        stop  = ng-1;
        while (1) {
          pivot = (stop+start)/2;
          if (x < grid[pivot]) {
            if (pivot==stop) return pivot;
            stop = pivot;
          } else {
            if (pivot==start) return pivot;
            start = pivot;
          }
        }
      }
    default:
      {
        casadi_int i;
        for (i=0; i<ng-2; ++i) {
          if (x < grid[i+1]) break;
        }
        return i;
      }
  }
}

void casadi_nd_boor_eval(casadi_real* ret, casadi_int n_dims, const casadi_real* all_knots, const casadi_int* offset, const casadi_int* all_degree, const casadi_int* strides, const casadi_real* c, casadi_int m, const casadi_real* all_x, const casadi_int* lookup_mode, casadi_int* iw, casadi_real* w) {
  casadi_int n_iter, k, i, pivot;
  casadi_int *boor_offset, *starts, *index, *coeff_offset;
  casadi_real *cumprod, *all_boor;
  boor_offset = iw; iw+=n_dims+1;
  starts = iw; iw+=n_dims;
  index = iw; iw+=n_dims;
  coeff_offset = iw;
  cumprod = w; w+= n_dims+1;
  all_boor = w;
  boor_offset[0] = 0;
  cumprod[n_dims] = 1;
  coeff_offset[n_dims] = 0;
  n_iter = 1;
  for (k=0;k<n_dims;++k) {
    casadi_real *boor;
    const casadi_real* knots;
    casadi_real x;
    casadi_int degree, n_knots, n_b, L, start;
    boor = all_boor+boor_offset[k];
    degree = all_degree[k];
    knots = all_knots + offset[k];
    n_knots = offset[k+1]-offset[k];
    n_b = n_knots-degree-1;
    x = all_x[k];
    L = casadi_low(x, knots+degree, n_knots-2*degree, lookup_mode[k]);
    start = L;
    if (start>n_b-degree-1) start = n_b-degree-1;
    starts[k] = start;
    casadi_clear(boor, 2*degree+1);
    if (x>=knots[0] && x<=knots[n_knots-1]) {
      if (x==knots[1]) {
        casadi_fill(boor, degree+1, 1.0);
      } else if (x==knots[n_knots-1]) {
        boor[degree] = 1;
      } else if (knots[L+degree]==x) {
        boor[degree-1] = 1;
      } else {
        boor[degree] = 1;
      }
    }
    casadi_de_boor(x, knots+start, 2*degree+2, degree, boor);
    boor+= degree+1;
    n_iter*= degree+1;
    boor_offset[k+1] = boor_offset[k] + degree+1;
  }
  casadi_clear_casadi_int(index, n_dims);
  for (pivot=n_dims-1;pivot>=0;--pivot) {
    cumprod[pivot] = (*(all_boor+boor_offset[pivot]))*cumprod[pivot+1];
    coeff_offset[pivot] = starts[pivot]*strides[pivot]+coeff_offset[pivot+1];
  }
  for (k=0;k<n_iter;++k) {
    casadi_int pivot = 0;
    for (i=0;i<m;++i) ret[i] += c[coeff_offset[0]+i]*cumprod[0];
    index[0]++;
    {
      while (index[pivot]==boor_offset[pivot+1]-boor_offset[pivot]) {
        index[pivot] = 0;
        if (pivot==n_dims-1) break;
        index[++pivot]++;
      }
      while (pivot>0) {
        cumprod[pivot] = (*(all_boor+boor_offset[pivot]+index[pivot]))*cumprod[pivot+1];
        coeff_offset[pivot] = (starts[pivot]+index[pivot])*strides[pivot]+coeff_offset[pivot+1];
        pivot--;
      }
    }
    cumprod[0] = (*(all_boor+index[0]))*cumprod[1];
    coeff_offset[0] = (starts[0]+index[0])*m+coeff_offset[1];
  }
}

casadi_real casadi_dot(casadi_int n, const casadi_real* x, const casadi_real* y) {
  casadi_int i;
  casadi_real r = 0;
  for (i=0; i<n; ++i) r += *x++ * *y++;
  return r;
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

static const casadi_int casadi_s0[2] = {0, 26};
static const casadi_int casadi_s1[1] = {3};
static const casadi_int casadi_s2[1] = {1};
static const casadi_int casadi_s3[1] = {0};
static const casadi_int casadi_s4[2] = {0, 24};
static const casadi_int casadi_s5[1] = {2};
static const casadi_int casadi_s6[24] = {20, 1, 0, 20, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
static const casadi_int casadi_s7[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s8[3] = {0, 0, 0};
static const casadi_int casadi_s9[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s10[6] = {24, 1, 0, 2, 4, 5};
static const casadi_int casadi_s11[3] = {1, 0, 0};

static const casadi_real casadi_c0[26] = {0., 0., 0., 0., 9.0000000000000002e-01, 1.2000000000000002e+00, 1.5000000000000000e+00, 1.8000000000000000e+00, 2.1000000000000001e+00, 2.4000000000000004e+00, 2.7000000000000002e+00, 3., 3.3000000000000003e+00, 3.6000000000000001e+00, 3.9000000000000004e+00, 4.2000000000000002e+00, 4.5000000000000000e+00, 4.8000000000000007e+00, 5.1000000000000005e+00, 5.4000000000000004e+00, 5.7000000000000002e+00, 6., 6.9000000000000004e+00, 6.9000000000000004e+00, 6.9000000000000004e+00, 6.9000000000000004e+00};
static const casadi_real casadi_c1[22] = {5.5333515547317802e-13, -2.1316282072803006e-13, 1.2523315717771766e-13, -2.1804780203638074e-13, 1.7186252421197423e-13, 1.7008616737257398e-13, -1.0080825063596421e-13, -3.3306690738754696e-14, -4.6629367034256575e-14, -7.9936057773011271e-14, 7.1054273576010019e-14, 2.4158453015843406e-13, 8.8817841970012523e-15, -4.1300296516055823e-13, 7.1054273576010019e-14, 2.6645352591003757e-13, -1.0658141036401503e-13, -1.4210854715202004e-13, 1.2434497875801753e-13, 6.1728400169158704e-14, -7.1054273576010019e-14, -5.6843418860808015e-13};
static const casadi_real casadi_c2[22] = {1.7281054460599656e+00, -1.9610802269919962e+00, 1.7546626649672736e+00, -1.5040208673481674e+00, 8.9278637143525970e+00, -1.1292057275181094e+01, 9.6745039731068143e+00, -3.5484255113074283e+00, 5.0740931759725516e+00, -4.6420515212587006e+00, 1.7210825927554827e+00, -8.4077405882549305e-02, 4.2138078078415964e+00, -1.0782656772684220e+01, 2.7026907324296752e+00, -1.4228709773460819e+00, 1.4189920192604641e+00, -2.4716102664124424e+00, 4.9003483340970044e+00, -2.8563999339530968e+00, 1.8462478685389998e+00, -1.1881392367963406e+00};
static const casadi_real casadi_c3[22] = {-1.5789838572446671e-14, 1.7763568394002505e-14, -1.5395092608135503e-14, 1.9737298215558338e-14, 3.7007434154171876e-14, 1.7763568394002511e-14, -2.9605947323337510e-14, -9.4739031434680029e-14, -2.5165055224836877e-14, 9.4739031434680042e-14, 7.8455760406844391e-14, -1.3470706032118565e-13, -2.5165055224836884e-14, 1.5247062871518808e-13, 4.5889218351173141e-14, -1.3618735768735254e-13, -1.0954200509634877e-13, 1.1842378929335009e-13, 3.5527136788005009e-14, 9.4739031434680067e-15, -1.2730557349035127e-13, 1.5789838572446665e-14};
static const casadi_real casadi_c4[24] = {0., 0., 0., 9.0000000000000002e-01, 1.2000000000000002e+00, 1.5000000000000000e+00, 1.8000000000000000e+00, 2.1000000000000001e+00, 2.4000000000000004e+00, 2.7000000000000002e+00, 3., 3.3000000000000003e+00, 3.6000000000000001e+00, 3.9000000000000004e+00, 4.2000000000000002e+00, 4.5000000000000000e+00, 4.8000000000000007e+00, 5.1000000000000005e+00, 5.4000000000000004e+00, 5.7000000000000002e+00, 6., 6.9000000000000004e+00, 6.9000000000000004e+00, 6.9000000000000004e+00};
static const casadi_real casadi_c5[21] = {-2.5549932540040272e-12, 8.4598994476436928e-13, -6.8656191842819680e-13, 1.2997010874945167e-12, -5.9211894646674679e-15, -9.0298139336179365e-13, 2.2500519965736504e-13, -4.4408920985006262e-14, -1.1102230246251565e-13, 5.0330110449673770e-13, 5.6843418860807995e-13, -7.7567581987144274e-13, -1.4062824978585316e-12, 1.6135241291218936e-12, 6.5133084111342490e-13, -1.2434497875801749e-12, -1.1842378929335012e-13, 8.8817841970012584e-13, -1.2523315717771766e-13, -3.3195668436292181e-13, -1.6579330501068999e-12};
static const casadi_real casadi_c6[21] = {-1.2297285576839872e+01, 9.2893572298981741e+00, -6.5173670646308821e+00, 3.4772948605669214e+01, -6.7399736631778978e+01, 6.9888537494292990e+01, -4.4076431614714139e+01, 2.8741728957599932e+01, -3.2387148990770839e+01, 2.1210447046713945e+01, -6.0171999954601052e+00, 1.4326284045747153e+01, -4.9988215268419388e+01, 4.4951158350379629e+01, -1.3751872365919183e+01, 9.4728766553551509e+00, -1.2968674285576363e+01, 2.4573195335031507e+01, -1.5513496536100202e+01, 1.1756619506230241e+01, -1.0114623684451132e+01};
static const casadi_real casadi_c7[21] = {1.1184468988816393e-13, -8.2896652505345017e-14, 7.0264781647387688e-14, 5.7567119795378457e-14, -6.4146219200564543e-14, -1.5789838572446670e-13, -2.1711028037114171e-13, 2.3191325403281056e-13, 3.9968028886505646e-13, -5.4277570092785509e-14, -7.1054273576009998e-13, 3.6514001698782924e-13, 5.9211894646674992e-13, -3.5527136788004969e-13, -6.0692192012841874e-13, 8.8817841970012523e-14, 7.5988598129899673e-13, -2.7632217501781712e-13, -5.2106467289074005e-14, -3.4194869158454816e-13, 4.7698470687599290e-13};

/* fwd1_jac_x_ref:(x,out_f[1x1,0nz],out_jac_f_x[1x1,0nz],fwd_x,fwd_out_f[1x1,0nz])->(fwd_jac_f_x) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1;
  /* #0: @0 = input[0][0] */
  w0 = arg[0] ? arg[0][0] : 0;
  /* #1: @1 = BSpline(@0) */
  casadi_clear((&w1), 1);
  CASADI_PREFIX(nd_boor_eval)((&w1),1,casadi_c0,casadi_s0,casadi_s1,casadi_s2,casadi_c1,1,(&w0),casadi_s3, iw, w);
  /* #2: @0 = input[3][0] */
  w0 = arg[3] ? arg[3][0] : 0;
  /* #3: @1 = (@1*@0) */
  w1 *= w0;
  /* #4: output[0][0] = @1 */
  if (res[0]) res[0][0] = w1;
  return 0;
}

/* fwd1_jac_y_ref:(x,out_f[1x1,0nz],out_jac_f_x[1x1,0nz],fwd_x,fwd_out_f[1x1,0nz])->(fwd_jac_f_x) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1;
  /* #0: @0 = input[0][0] */
  w0 = arg[0] ? arg[0][0] : 0;
  /* #1: @1 = BSpline(@0) */
  casadi_clear((&w1), 1);
  CASADI_PREFIX(nd_boor_eval)((&w1),1,casadi_c0,casadi_s0,casadi_s1,casadi_s2,casadi_c2,1,(&w0),casadi_s3, iw, w);
  /* #2: @0 = input[3][0] */
  w0 = arg[3] ? arg[3][0] : 0;
  /* #3: @1 = (@1*@0) */
  w1 *= w0;
  /* #4: output[0][0] = @1 */
  if (res[0]) res[0][0] = w1;
  return 0;
}

/* fwd1_jac_z_ref:(x,out_f[1x1,0nz],out_jac_f_x[1x1,0nz],fwd_x,fwd_out_f[1x1,0nz])->(fwd_jac_f_x) */
static int casadi_f3(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1;
  /* #0: @0 = input[0][0] */
  w0 = arg[0] ? arg[0][0] : 0;
  /* #1: @1 = BSpline(@0) */
  casadi_clear((&w1), 1);
  CASADI_PREFIX(nd_boor_eval)((&w1),1,casadi_c0,casadi_s0,casadi_s1,casadi_s2,casadi_c3,1,(&w0),casadi_s3, iw, w);
  /* #2: @0 = input[3][0] */
  w0 = arg[3] ? arg[3][0] : 0;
  /* #3: @1 = (@1*@0) */
  w1 *= w0;
  /* #4: output[0][0] = @1 */
  if (res[0]) res[0][0] = w1;
  return 0;
}

/* fwd1_fwd1_jac_x_ref:(x,out_f[1x1,0nz],out_jac_f_x[1x1,0nz],fwd_x,fwd_out_f[1x1,0nz],out_fwd_jac_f_x[1x1,0nz],fwd2_x,fwd2_out_f[1x1,0nz],fwd2_out_jac_f_x[1x1,0nz],fwd2_fwd_x,fwd2_fwd_out_f[1x1,0nz])->(fwd2_fwd_jac_f_x) */
static int casadi_f4(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1, w2, w3;
  /* #0: @0 = input[3][0] */
  w0 = arg[3] ? arg[3][0] : 0;
  /* #1: @1 = input[0][0] */
  w1 = arg[0] ? arg[0][0] : 0;
  /* #2: @2 = BSpline(@1) */
  casadi_clear((&w2), 1);
  CASADI_PREFIX(nd_boor_eval)((&w2),1,casadi_c4,casadi_s4,casadi_s5,casadi_s2,casadi_c5,1,(&w1),casadi_s3, iw, w);
  /* #3: @3 = input[6][0] */
  w3 = arg[6] ? arg[6][0] : 0;
  /* #4: @2 = (@2*@3) */
  w2 *= w3;
  /* #5: @0 = (@0*@2) */
  w0 *= w2;
  /* #6: @2 = BSpline(@1) */
  casadi_clear((&w2), 1);
  CASADI_PREFIX(nd_boor_eval)((&w2),1,casadi_c0,casadi_s0,casadi_s1,casadi_s2,casadi_c1,1,(&w1),casadi_s3, iw, w);
  /* #7: @1 = input[9][0] */
  w1 = arg[9] ? arg[9][0] : 0;
  /* #8: @2 = (@2*@1) */
  w2 *= w1;
  /* #9: @0 = (@0+@2) */
  w0 += w2;
  /* #10: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  return 0;
}

/* fwd1_fwd1_jac_y_ref:(x,out_f[1x1,0nz],out_jac_f_x[1x1,0nz],fwd_x,fwd_out_f[1x1,0nz],out_fwd_jac_f_x[1x1,0nz],fwd2_x,fwd2_out_f[1x1,0nz],fwd2_out_jac_f_x[1x1,0nz],fwd2_fwd_x,fwd2_fwd_out_f[1x1,0nz])->(fwd2_fwd_jac_f_x) */
static int casadi_f5(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1, w2, w3;
  /* #0: @0 = input[3][0] */
  w0 = arg[3] ? arg[3][0] : 0;
  /* #1: @1 = input[0][0] */
  w1 = arg[0] ? arg[0][0] : 0;
  /* #2: @2 = BSpline(@1) */
  casadi_clear((&w2), 1);
  CASADI_PREFIX(nd_boor_eval)((&w2),1,casadi_c4,casadi_s4,casadi_s5,casadi_s2,casadi_c6,1,(&w1),casadi_s3, iw, w);
  /* #3: @3 = input[6][0] */
  w3 = arg[6] ? arg[6][0] : 0;
  /* #4: @2 = (@2*@3) */
  w2 *= w3;
  /* #5: @0 = (@0*@2) */
  w0 *= w2;
  /* #6: @2 = BSpline(@1) */
  casadi_clear((&w2), 1);
  CASADI_PREFIX(nd_boor_eval)((&w2),1,casadi_c0,casadi_s0,casadi_s1,casadi_s2,casadi_c2,1,(&w1),casadi_s3, iw, w);
  /* #7: @1 = input[9][0] */
  w1 = arg[9] ? arg[9][0] : 0;
  /* #8: @2 = (@2*@1) */
  w2 *= w1;
  /* #9: @0 = (@0+@2) */
  w0 += w2;
  /* #10: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  return 0;
}

/* fwd1_fwd1_jac_z_ref:(x,out_f[1x1,0nz],out_jac_f_x[1x1,0nz],fwd_x,fwd_out_f[1x1,0nz],out_fwd_jac_f_x[1x1,0nz],fwd2_x,fwd2_out_f[1x1,0nz],fwd2_out_jac_f_x[1x1,0nz],fwd2_fwd_x,fwd2_fwd_out_f[1x1,0nz])->(fwd2_fwd_jac_f_x) */
static int casadi_f6(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1, w2, w3;
  /* #0: @0 = input[3][0] */
  w0 = arg[3] ? arg[3][0] : 0;
  /* #1: @1 = input[0][0] */
  w1 = arg[0] ? arg[0][0] : 0;
  /* #2: @2 = BSpline(@1) */
  casadi_clear((&w2), 1);
  CASADI_PREFIX(nd_boor_eval)((&w2),1,casadi_c4,casadi_s4,casadi_s5,casadi_s2,casadi_c7,1,(&w1),casadi_s3, iw, w);
  /* #3: @3 = input[6][0] */
  w3 = arg[6] ? arg[6][0] : 0;
  /* #4: @2 = (@2*@3) */
  w2 *= w3;
  /* #5: @0 = (@0*@2) */
  w0 *= w2;
  /* #6: @2 = BSpline(@1) */
  casadi_clear((&w2), 1);
  CASADI_PREFIX(nd_boor_eval)((&w2),1,casadi_c0,casadi_s0,casadi_s1,casadi_s2,casadi_c3,1,(&w1),casadi_s3, iw, w);
  /* #7: @1 = input[9][0] */
  w1 = arg[9] ? arg[9][0] : 0;
  /* #8: @2 = (@2*@1) */
  w2 *= w1;
  /* #9: @0 = (@0+@2) */
  w0 += w2;
  /* #10: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  return 0;
}

/* quadrotor_constr_h_fun_jac_uxt_zt:(i0[20],i1[4],i2[],i3[])->(o0,o1[24x1,2nz],o2[1x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real **res1=res+3, *rr, *ss;
  const casadi_real **arg1=arg+4;
  casadi_real w0, w3, w5, w8, w10, w13, w15, *w16=w+20, *w17=w+23, *w19=w+25, w22, w24, *w25=w+50;
  /* #0: @0 = input[0][0] */
  w0 = arg[0] ? arg[0][0] : 0;
  /* #1: @1 = 00 */
  /* #2: @2 = 00 */
  /* #3: @3 = 1 */
  w3 = 1.;
  /* #4: @4 = 00 */
  /* #5: @5 = fwd1_jac_x_ref(@0, @1, @2, @3, @4) */
  arg1[0]=(&w0);
  arg1[1]=0;
  arg1[2]=0;
  arg1[3]=(&w3);
  arg1[4]=0;
  res1[0]=(&w5);
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #6: @6 = 00 */
  /* #7: @7 = 00 */
  /* #8: @8 = 1 */
  w8 = 1.;
  /* #9: @9 = 00 */
  /* #10: @10 = fwd1_jac_y_ref(@0, @6, @7, @8, @9) */
  arg1[0]=(&w0);
  arg1[1]=0;
  arg1[2]=0;
  arg1[3]=(&w8);
  arg1[4]=0;
  res1[0]=(&w10);
  if (casadi_f2(arg1, res1, iw, w, 0)) return 1;
  /* #11: @11 = 00 */
  /* #12: @12 = 00 */
  /* #13: @13 = 1 */
  w13 = 1.;
  /* #14: @14 = 00 */
  /* #15: @15 = fwd1_jac_z_ref(@0, @11, @12, @13, @14) */
  arg1[0]=(&w0);
  arg1[1]=0;
  arg1[2]=0;
  arg1[3]=(&w13);
  arg1[4]=0;
  res1[0]=(&w15);
  if (casadi_f3(arg1, res1, iw, w, 0)) return 1;
  /* #16: @16 = vertcat(@5, @10, @15) */
  rr=w16;
  *rr++ = w5;
  *rr++ = w10;
  *rr++ = w15;
  /* #17: @5 = ||@16||_F */
  w5 = sqrt(casadi_dot(3, w16, w16));
  /* #18: @10 = input[0][1] */
  w10 = arg[0] ? arg[0][1] : 0;
  /* #19: @15 = (@5*@10) */
  w15  = (w5*w10);
  /* #20: output[0][0] = @15 */
  if (res[0]) res[0][0] = w15;
  /* #21: @17 = zeros(24x1,2nz) */
  casadi_clear(w17, 2);
  /* #22: @18 = 00 */
  /* #23: @19 = ones(24x1,23nz) */
  casadi_fill(w19, 23, 1.);
  /* #24: {NULL, NULL, NULL, NULL, @15, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL} = vertsplit(@19) */
  w15 = w19[4];
  /* #25: @20 = 00 */
  /* #26: @21 = 00 */
  /* #27: @22 = 0 */
  w22 = 0.;
  /* #28: @23 = 00 */
  /* #29: @24 = fwd1_fwd1_jac_x_ref(@0, @1, @2, @3, @4, @18, @15, @20, @21, @22, @23) */
  arg1[0]=(&w0);
  arg1[1]=0;
  arg1[2]=0;
  arg1[3]=(&w3);
  arg1[4]=0;
  arg1[5]=0;
  arg1[6]=(&w15);
  arg1[7]=0;
  arg1[8]=0;
  arg1[9]=(&w22);
  arg1[10]=0;
  res1[0]=(&w24);
  if (casadi_f4(arg1, res1, iw, w, 0)) return 1;
  /* #30: @1 = 00 */
  /* #31: @2 = 00 */
  /* #32: @4 = 00 */
  /* #33: @3 = 0 */
  w3 = 0.;
  /* #34: @18 = 00 */
  /* #35: @22 = fwd1_fwd1_jac_y_ref(@0, @6, @7, @8, @9, @1, @15, @2, @4, @3, @18) */
  arg1[0]=(&w0);
  arg1[1]=0;
  arg1[2]=0;
  arg1[3]=(&w8);
  arg1[4]=0;
  arg1[5]=0;
  arg1[6]=(&w15);
  arg1[7]=0;
  arg1[8]=0;
  arg1[9]=(&w3);
  arg1[10]=0;
  res1[0]=(&w22);
  if (casadi_f5(arg1, res1, iw, w, 0)) return 1;
  /* #36: @6 = 00 */
  /* #37: @7 = 00 */
  /* #38: @9 = 00 */
  /* #39: @8 = 0 */
  w8 = 0.;
  /* #40: @1 = 00 */
  /* #41: @3 = fwd1_fwd1_jac_z_ref(@0, @11, @12, @13, @14, @6, @15, @7, @9, @8, @1) */
  arg1[0]=(&w0);
  arg1[1]=0;
  arg1[2]=0;
  arg1[3]=(&w13);
  arg1[4]=0;
  arg1[5]=0;
  arg1[6]=(&w15);
  arg1[7]=0;
  arg1[8]=0;
  arg1[9]=(&w8);
  arg1[10]=0;
  res1[0]=(&w3);
  if (casadi_f6(arg1, res1, iw, w, 0)) return 1;
  /* #42: @25 = vertcat(@24, @22, @3) */
  rr=w25;
  *rr++ = w24;
  *rr++ = w22;
  *rr++ = w3;
  /* #43: @24 = dot(@16, @25) */
  w24 = casadi_dot(3, w16, w25);
  /* #44: @24 = (@24/@5) */
  w24 /= w5;
  /* #45: @10 = (@10*@24) */
  w10 *= w24;
  /* #46: (@17[0] = @10) */
  for (rr=w17+0, ss=(&w10); rr!=w17+1; rr+=1) *rr = *ss++;
  /* #47: @10 = ones(24x1,1nz) */
  w10 = 1.;
  /* #48: {NULL, NULL, NULL, NULL, NULL, @24, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL} = vertsplit(@10) */
  w24 = w10;
  /* #49: @5 = (@5*@24) */
  w5 *= w24;
  /* #50: (@17[1] = @5) */
  for (rr=w17+1, ss=(&w5); rr!=w17+2; rr+=1) *rr = *ss++;
  /* #51: output[1][0] = @17 */
  casadi_copy(w17, 2, res[1]);
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_jac_uxt_zt(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_jac_uxt_zt_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_jac_uxt_zt_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_fun_jac_uxt_zt_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_jac_uxt_zt_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_fun_jac_uxt_zt_release(int mem) {
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_fun_jac_uxt_zt_incref(void) {
}

CASADI_SYMBOL_EXPORT void quadrotor_constr_h_fun_jac_uxt_zt_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_constr_h_fun_jac_uxt_zt_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int quadrotor_constr_h_fun_jac_uxt_zt_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real quadrotor_constr_h_fun_jac_uxt_zt_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_constr_h_fun_jac_uxt_zt_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quadrotor_constr_h_fun_jac_uxt_zt_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_constr_h_fun_jac_uxt_zt_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s6;
    case 1: return casadi_s7;
    case 2: return casadi_s8;
    case 3: return casadi_s8;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quadrotor_constr_h_fun_jac_uxt_zt_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s9;
    case 1: return casadi_s10;
    case 2: return casadi_s11;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_jac_uxt_zt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 17;
  if (sz_res) *sz_res = 27;
  if (sz_iw) *sz_iw = 6;
  if (sz_w) *sz_w = 53;
  return 0;
}

CASADI_SYMBOL_EXPORT int quadrotor_constr_h_fun_jac_uxt_zt_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 17*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 27*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 6*sizeof(casadi_int);
  if (sz_w) *sz_w = 53*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
