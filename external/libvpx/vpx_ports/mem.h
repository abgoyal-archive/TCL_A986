


#ifndef VPX_PORTS_MEM_H
#define VPX_PORTS_MEM_H
#include "vpx_config.h"
#include "vpx/vpx_integer.h"

#if defined(__GNUC__) && __GNUC__
#define DECLARE_ALIGNED(n,typ,val)  typ val __attribute__ ((aligned (n)))
#elif defined(_MSC_VER)
#define DECLARE_ALIGNED(n,typ,val)  __declspec(align(n)) typ val
#else
#warning No alignment directives known for this compiler.
#define DECLARE_ALIGNED(n,typ,val)  typ val
#endif
#endif


#define DECLARE_ALIGNED_ARRAY(a,typ,val,n)\
typ val##_[(n)+(a)/sizeof(typ)+1];\
typ *val = (typ*)((((intptr_t)val##_)+(a)-1)&((intptr_t)-(a)))


#if defined(__GNUC__) && __GNUC__
#define UNINITIALIZED_IS_SAFE(x) x=x
#else
#define UNINITIALIZED_IS_SAFE(x) x
#endif
