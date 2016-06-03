
/* This file defines the ANSI C floating-point functions.  It is here
   because:
   1)  we need also the single-precision functions, missing from most
       compilers
   2)  some compilers define some ANSI functions wrong
*/
#ifndef __MATHF_H
#define __MATHF_H
 
#ifndef __C32_LEGACY_LIBC__
#define __C32_LEGACY_LIBC__
#endif

 
float acosf(float);	
float asinf(float);
float atanf(float);
float atan2f(float, float);
float ceilf(float);
float cosf(float);
float coshf(float);
float expf(float);
float fabsf(float);
float floorf(float);
float fmodf(float, float);
float frexpf(float, int *);
float ldexpf(float, int);
float logf(float);
float log10f(float);
float modff(float, float *);
float powf(float, float);
float sinf(float);
float sinhf(float);
float sqrtf(float);
float tanf(float);
float tanhf(float);

long double acos(long double);	
long double asin(long double);
long double atan(long double);
long double atan2(long double, long double);
long double ceil(long double);
long double cos(long double);
long double cosh(long double);
long double exp(long double);
long double fabs(long double);
long double floor(long double);
long double fmod(long double, long double);
long double frexp(long double, int *);
long double ldexp(long double, int);
long double log(long double);
long double log10(long double);
long double modf(long double, long double *);
long double pow(long double, long double);
long double sin(long double);
long double sinh(long double);
long double sqrt(long double);
long double tan(long double);
long double tanh(long double);

#endif
