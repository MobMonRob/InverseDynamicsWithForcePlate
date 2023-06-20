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
  #define CASADI_PREFIX(ID) jit_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)

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

static const casadi_int casadi_s0[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};

/* C_bu:(i0[6],i1[6],i2[6],i3[6])->(o0[6]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[3]? arg[3][2] : 0;
  if (res[0]!=0) res[0][0]=a0;
  if (res[0]!=0) res[0][1]=a0;
  a1=-2.0510342851533115e-10;
  a2=(a1*a0);
  a3=arg[0]? arg[0][0] : 0;
  a4=sin(a3);
  a5=-1.2246467991473532e-16;
  a3=cos(a3);
  a6=(a5*a3);
  a6=(a4+a6);
  a7=arg[3]? arg[3][0] : 0;
  a6=(a6*a7);
  a8=1.2246467991473532e-16;
  a9=(a8*a4);
  a9=(a9+a3);
  a10=arg[3]? arg[3][1] : 0;
  a9=(a9*a10);
  a6=(a6-a9);
  a2=(a2-a6);
  if (res[0]!=0) res[0][2]=a2;
  if (res[0]!=0) res[0][3]=a2;
  if (res[0]!=0) res[0][4]=a2;
  a2=(a1*a2);
  a9=arg[0]? arg[0][3] : 0;
  a11=cos(a9);
  a12=arg[0]? arg[0][2] : 0;
  a13=cos(a12);
  a14=arg[0]? arg[0][1] : 0;
  a15=cos(a14);
  a16=(a1*a15);
  a16=(a16*a6);
  a14=sin(a14);
  a5=(a5*a4);
  a5=(a5-a3);
  a5=(a5*a7);
  a8=(a8*a3);
  a8=(a8-a4);
  a8=(a8*a10);
  a5=(a5+a8);
  a8=(a14*a5);
  a16=(a16-a8);
  a8=(a15*a0);
  a16=(a16+a8);
  a8=(a13*a16);
  a12=sin(a12);
  a15=(a15*a5);
  a1=(a1*a14);
  a1=(a1*a6);
  a15=(a15+a1);
  a14=(a14*a0);
  a15=(a15+a14);
  a14=(a12*a15);
  a8=(a8-a14);
  a11=(a11*a8);
  a9=sin(a9);
  a13=(a13*a15);
  a12=(a12*a16);
  a13=(a13+a12);
  a9=(a9*a13);
  a11=(a11-a9);
  a2=(a2-a11);
  if (res[0]!=0) res[0][5]=a2;
  return 0;
}

CASADI_SYMBOL_EXPORT int C_bu(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int C_bu_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int C_bu_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void C_bu_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int C_bu_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void C_bu_release(int mem) {
}

CASADI_SYMBOL_EXPORT void C_bu_incref(void) {
}

CASADI_SYMBOL_EXPORT void C_bu_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int C_bu_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int C_bu_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real C_bu_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* C_bu_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* C_bu_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* C_bu_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s0;
    case 3: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* C_bu_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int C_bu_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif