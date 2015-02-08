



#ifndef __VALGRIND_H
#define __VALGRIND_H

#include <stdarg.h>
#include <stdint.h>


#undef PLAT_x86_linux
#undef PLAT_amd64_linux
#undef PLAT_ppc32_linux
#undef PLAT_ppc64_linux
#undef PLAT_ppc32_aix5
#undef PLAT_ppc64_aix5

#if !defined(_AIX) && defined(__i386__)
#  define PLAT_x86_linux 1
#elif !defined(_AIX) && defined(__x86_64__)
#  define PLAT_amd64_linux 1
#elif !defined(_AIX) && defined(__powerpc__) && !defined(__powerpc64__)
#  define PLAT_ppc32_linux 1
#elif !defined(_AIX) && defined(__powerpc__) && defined(__powerpc64__)
#  define PLAT_ppc64_linux 1
#elif defined(_AIX) && defined(__64BIT__)
#  define PLAT_ppc64_aix5 1
#elif defined(_AIX) && !defined(__64BIT__)
#  define PLAT_ppc32_aix5 1
#endif


#if !defined(PLAT_x86_linux) && !defined(PLAT_amd64_linux) \
    && !defined(PLAT_ppc32_linux) && !defined(PLAT_ppc64_linux) \
    && !defined(PLAT_ppc32_aix5) && !defined(PLAT_ppc64_aix5)
#  if !defined(NVALGRIND)
#    define NVALGRIND 1
#  endif
#endif


/* ------------------------------------------------------------------ */
/* ARCHITECTURE SPECIFICS for SPECIAL INSTRUCTIONS.  There is nothing */
/* in here of use to end-users -- skip to the next section.           */
/* ------------------------------------------------------------------ */

#if defined(NVALGRIND)

#define VALGRIND_DO_CLIENT_REQUEST(                               \
        _zzq_rlval, _zzq_default, _zzq_request,                   \
        _zzq_arg1, _zzq_arg2, _zzq_arg3, _zzq_arg4, _zzq_arg5)    \
   {                                                              \
      (_zzq_rlval) = (_zzq_default);                              \
   }

#else  /* ! NVALGRIND */


/* ------------------------- x86-linux ------------------------- */

#if defined(PLAT_x86_linux)

typedef
   struct { 
      unsigned int nraddr; /* where's the code? */
   }
   OrigFn;

#define __SPECIAL_INSTRUCTION_PREAMBLE                            \
                     "roll $3,  %%edi ; roll $13, %%edi\n\t"      \
                     "roll $29, %%edi ; roll $19, %%edi\n\t"

#define VALGRIND_DO_CLIENT_REQUEST(                               \
        _zzq_rlval, _zzq_default, _zzq_request,                   \
        _zzq_arg1, _zzq_arg2, _zzq_arg3, _zzq_arg4, _zzq_arg5)    \
  { volatile unsigned int _zzq_args[6];                           \
    volatile unsigned int _zzq_result;                            \
    _zzq_args[0] = (unsigned int)(_zzq_request);                  \
    _zzq_args[1] = (unsigned int)(_zzq_arg1);                     \
    _zzq_args[2] = (unsigned int)(_zzq_arg2);                     \
    _zzq_args[3] = (unsigned int)(_zzq_arg3);                     \
    _zzq_args[4] = (unsigned int)(_zzq_arg4);                     \
    _zzq_args[5] = (unsigned int)(_zzq_arg5);                     \
    __asm__ volatile(__SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* %EDX = client_request ( %EAX ) */         \
                     "xchgl %%ebx,%%ebx"                          \
                     : "=d" (_zzq_result)                         \
                     : "a" (&_zzq_args[0]), "0" (_zzq_default)    \
                     : "cc", "memory"                             \
                    );                                            \
    _zzq_rlval = _zzq_result;                                     \
  }

#define VALGRIND_GET_NR_CONTEXT(_zzq_rlval)                       \
  { volatile OrigFn* _zzq_orig = &(_zzq_rlval);                   \
    volatile unsigned int __addr;                                 \
    __asm__ volatile(__SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* %EAX = guest_NRADDR */                    \
                     "xchgl %%ecx,%%ecx"                          \
                     : "=a" (__addr)                              \
                     :                                            \
                     : "cc", "memory"                             \
                    );                                            \
    _zzq_orig->nraddr = __addr;                                   \
  }

#define VALGRIND_CALL_NOREDIR_EAX                                 \
                     __SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* call-noredir *%EAX */                     \
                     "xchgl %%edx,%%edx\n\t"
#endif /* PLAT_x86_linux */

/* ------------------------ amd64-linux ------------------------ */

#if defined(PLAT_amd64_linux)

typedef
   struct { 
      uint64_t nraddr; /* where's the code? */
   }
   OrigFn;

#define __SPECIAL_INSTRUCTION_PREAMBLE                            \
                     "rolq $3,  %%rdi ; rolq $13, %%rdi\n\t"      \
                     "rolq $61, %%rdi ; rolq $51, %%rdi\n\t"

#define VALGRIND_DO_CLIENT_REQUEST(                               \
        _zzq_rlval, _zzq_default, _zzq_request,                   \
        _zzq_arg1, _zzq_arg2, _zzq_arg3, _zzq_arg4, _zzq_arg5)    \
  { volatile uint64_t _zzq_args[6];                 \
    volatile uint64_t _zzq_result;                  \
    _zzq_args[0] = (uint64_t)(_zzq_request);        \
    _zzq_args[1] = (uint64_t)(_zzq_arg1);           \
    _zzq_args[2] = (uint64_t)(_zzq_arg2);           \
    _zzq_args[3] = (uint64_t)(_zzq_arg3);           \
    _zzq_args[4] = (uint64_t)(_zzq_arg4);           \
    _zzq_args[5] = (uint64_t)(_zzq_arg5);           \
    __asm__ volatile(__SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* %RDX = client_request ( %RAX ) */         \
                     "xchgq %%rbx,%%rbx"                          \
                     : "=d" (_zzq_result)                         \
                     : "a" (&_zzq_args[0]), "0" (_zzq_default)    \
                     : "cc", "memory"                             \
                    );                                            \
    _zzq_rlval = _zzq_result;                                     \
  }

#define VALGRIND_GET_NR_CONTEXT(_zzq_rlval)                       \
  { volatile OrigFn* _zzq_orig = &(_zzq_rlval);                   \
    volatile uint64_t __addr;                       \
    __asm__ volatile(__SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* %RAX = guest_NRADDR */                    \
                     "xchgq %%rcx,%%rcx"                          \
                     : "=a" (__addr)                              \
                     :                                            \
                     : "cc", "memory"                             \
                    );                                            \
    _zzq_orig->nraddr = __addr;                                   \
  }

#define VALGRIND_CALL_NOREDIR_RAX                                 \
                     __SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* call-noredir *%RAX */                     \
                     "xchgq %%rdx,%%rdx\n\t"
#endif /* PLAT_amd64_linux */

/* ------------------------ ppc32-linux ------------------------ */

#if defined(PLAT_ppc32_linux)

typedef
   struct { 
      unsigned int nraddr; /* where's the code? */
   }
   OrigFn;

#define __SPECIAL_INSTRUCTION_PREAMBLE                            \
                     "rlwinm 0,0,3,0,0  ; rlwinm 0,0,13,0,0\n\t"  \
                     "rlwinm 0,0,29,0,0 ; rlwinm 0,0,19,0,0\n\t"

#define VALGRIND_DO_CLIENT_REQUEST(                               \
        _zzq_rlval, _zzq_default, _zzq_request,                   \
        _zzq_arg1, _zzq_arg2, _zzq_arg3, _zzq_arg4, _zzq_arg5)    \
                                                                  \
  {          unsigned int  _zzq_args[6];                          \
             unsigned int  _zzq_result;                           \
             unsigned int* _zzq_ptr;                              \
    _zzq_args[0] = (unsigned int)(_zzq_request);                  \
    _zzq_args[1] = (unsigned int)(_zzq_arg1);                     \
    _zzq_args[2] = (unsigned int)(_zzq_arg2);                     \
    _zzq_args[3] = (unsigned int)(_zzq_arg3);                     \
    _zzq_args[4] = (unsigned int)(_zzq_arg4);                     \
    _zzq_args[5] = (unsigned int)(_zzq_arg5);                     \
    _zzq_ptr = _zzq_args;                                         \
    __asm__ volatile("mr 3,%1\n\t" /*default*/                    \
                     "mr 4,%2\n\t" /*ptr*/                        \
                     __SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* %R3 = client_request ( %R4 ) */           \
                     "or 1,1,1\n\t"                               \
                     "mr %0,3"     /*result*/                     \
                     : "=b" (_zzq_result)                         \
                     : "b" (_zzq_default), "b" (_zzq_ptr)         \
                     : "cc", "memory", "r3", "r4");               \
    _zzq_rlval = _zzq_result;                                     \
  }

#define VALGRIND_GET_NR_CONTEXT(_zzq_rlval)                       \
  { volatile OrigFn* _zzq_orig = &(_zzq_rlval);                   \
    unsigned int __addr;                                          \
    __asm__ volatile(__SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* %R3 = guest_NRADDR */                     \
                     "or 2,2,2\n\t"                               \
                     "mr %0,3"                                    \
                     : "=b" (__addr)                              \
                     :                                            \
                     : "cc", "memory", "r3"                       \
                    );                                            \
    _zzq_orig->nraddr = __addr;                                   \
  }

#define VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                   \
                     __SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* branch-and-link-to-noredir *%R11 */       \
                     "or 3,3,3\n\t"
#endif /* PLAT_ppc32_linux */

/* ------------------------ ppc64-linux ------------------------ */

#if defined(PLAT_ppc64_linux)

typedef
   struct { 
      uint64_t nraddr; /* where's the code? */
      uint64_t r2;  /* what tocptr do we need? */
   }
   OrigFn;

#define __SPECIAL_INSTRUCTION_PREAMBLE                            \
                     "rotldi 0,0,3  ; rotldi 0,0,13\n\t"          \
                     "rotldi 0,0,61 ; rotldi 0,0,51\n\t"

#define VALGRIND_DO_CLIENT_REQUEST(                               \
        _zzq_rlval, _zzq_default, _zzq_request,                   \
        _zzq_arg1, _zzq_arg2, _zzq_arg3, _zzq_arg4, _zzq_arg5)    \
                                                                  \
  {          uint64_t  _zzq_args[6];                \
    register uint64_t  _zzq_result __asm__("r3");   \
    register uint64_t* _zzq_ptr __asm__("r4");      \
    _zzq_args[0] = (uint64_t)(_zzq_request);        \
    _zzq_args[1] = (uint64_t)(_zzq_arg1);           \
    _zzq_args[2] = (uint64_t)(_zzq_arg2);           \
    _zzq_args[3] = (uint64_t)(_zzq_arg3);           \
    _zzq_args[4] = (uint64_t)(_zzq_arg4);           \
    _zzq_args[5] = (uint64_t)(_zzq_arg5);           \
    _zzq_ptr = _zzq_args;                                         \
    __asm__ volatile(__SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* %R3 = client_request ( %R4 ) */           \
                     "or 1,1,1"                                   \
                     : "=r" (_zzq_result)                         \
                     : "0" (_zzq_default), "r" (_zzq_ptr)         \
                     : "cc", "memory");                           \
    _zzq_rlval = _zzq_result;                                     \
  }

#define VALGRIND_GET_NR_CONTEXT(_zzq_rlval)                       \
  { volatile OrigFn* _zzq_orig = &(_zzq_rlval);                   \
    register uint64_t __addr __asm__("r3");         \
    __asm__ volatile(__SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* %R3 = guest_NRADDR */                     \
                     "or 2,2,2"                                   \
                     : "=r" (__addr)                              \
                     :                                            \
                     : "cc", "memory"                             \
                    );                                            \
    _zzq_orig->nraddr = __addr;                                   \
    __asm__ volatile(__SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* %R3 = guest_NRADDR_GPR2 */                \
                     "or 4,4,4"                                   \
                     : "=r" (__addr)                              \
                     :                                            \
                     : "cc", "memory"                             \
                    );                                            \
    _zzq_orig->r2 = __addr;                                       \
  }

#define VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                   \
                     __SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* branch-and-link-to-noredir *%R11 */       \
                     "or 3,3,3\n\t"

#endif /* PLAT_ppc64_linux */

/* ------------------------ ppc32-aix5 ------------------------- */

#if defined(PLAT_ppc32_aix5)

typedef
   struct { 
      unsigned int nraddr; /* where's the code? */
      unsigned int r2;  /* what tocptr do we need? */
   }
   OrigFn;

#define __SPECIAL_INSTRUCTION_PREAMBLE                            \
                     "rlwinm 0,0,3,0,0  ; rlwinm 0,0,13,0,0\n\t"  \
                     "rlwinm 0,0,29,0,0 ; rlwinm 0,0,19,0,0\n\t"

#define VALGRIND_DO_CLIENT_REQUEST(                               \
        _zzq_rlval, _zzq_default, _zzq_request,                   \
        _zzq_arg1, _zzq_arg2, _zzq_arg3, _zzq_arg4, _zzq_arg5)    \
                                                                  \
  {          unsigned int  _zzq_args[7];                          \
    register unsigned int  _zzq_result;                           \
    register unsigned int* _zzq_ptr;                              \
    _zzq_args[0] = (unsigned int)(_zzq_request);                  \
    _zzq_args[1] = (unsigned int)(_zzq_arg1);                     \
    _zzq_args[2] = (unsigned int)(_zzq_arg2);                     \
    _zzq_args[3] = (unsigned int)(_zzq_arg3);                     \
    _zzq_args[4] = (unsigned int)(_zzq_arg4);                     \
    _zzq_args[5] = (unsigned int)(_zzq_arg5);                     \
    _zzq_args[6] = (unsigned int)(_zzq_default);                  \
    _zzq_ptr = _zzq_args;                                         \
    __asm__ volatile("mr 4,%1\n\t"                                \
                     "lwz 3, 24(4)\n\t"                           \
                     __SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* %R3 = client_request ( %R4 ) */           \
                     "or 1,1,1\n\t"                               \
                     "mr %0,3"                                    \
                     : "=b" (_zzq_result)                         \
                     : "b" (_zzq_ptr)                             \
                     : "r3", "r4", "cc", "memory");               \
    _zzq_rlval = _zzq_result;                                     \
  }

#define VALGRIND_GET_NR_CONTEXT(_zzq_rlval)                       \
  { volatile OrigFn* _zzq_orig = &(_zzq_rlval);                   \
    register unsigned int __addr;                                 \
    __asm__ volatile(__SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* %R3 = guest_NRADDR */                     \
                     "or 2,2,2\n\t"                               \
                     "mr %0,3"                                    \
                     : "=b" (__addr)                              \
                     :                                            \
                     : "r3", "cc", "memory"                       \
                    );                                            \
    _zzq_orig->nraddr = __addr;                                   \
    __asm__ volatile(__SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* %R3 = guest_NRADDR_GPR2 */                \
                     "or 4,4,4\n\t"                               \
                     "mr %0,3"                                    \
                     : "=b" (__addr)                              \
                     :                                            \
                     : "r3", "cc", "memory"                       \
                    );                                            \
    _zzq_orig->r2 = __addr;                                       \
  }

#define VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                   \
                     __SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* branch-and-link-to-noredir *%R11 */       \
                     "or 3,3,3\n\t"

#endif /* PLAT_ppc32_aix5 */

/* ------------------------ ppc64-aix5 ------------------------- */

#if defined(PLAT_ppc64_aix5)

typedef
   struct { 
      uint64_t nraddr; /* where's the code? */
      uint64_t r2;  /* what tocptr do we need? */
   }
   OrigFn;

#define __SPECIAL_INSTRUCTION_PREAMBLE                            \
                     "rotldi 0,0,3  ; rotldi 0,0,13\n\t"          \
                     "rotldi 0,0,61 ; rotldi 0,0,51\n\t"

#define VALGRIND_DO_CLIENT_REQUEST(                               \
        _zzq_rlval, _zzq_default, _zzq_request,                   \
        _zzq_arg1, _zzq_arg2, _zzq_arg3, _zzq_arg4, _zzq_arg5)    \
                                                                  \
  {          uint64_t  _zzq_args[7];                \
    register uint64_t  _zzq_result;                 \
    register uint64_t* _zzq_ptr;                    \
    _zzq_args[0] = (unsigned int long long)(_zzq_request);        \
    _zzq_args[1] = (unsigned int long long)(_zzq_arg1);           \
    _zzq_args[2] = (unsigned int long long)(_zzq_arg2);           \
    _zzq_args[3] = (unsigned int long long)(_zzq_arg3);           \
    _zzq_args[4] = (unsigned int long long)(_zzq_arg4);           \
    _zzq_args[5] = (unsigned int long long)(_zzq_arg5);           \
    _zzq_args[6] = (unsigned int long long)(_zzq_default);        \
    _zzq_ptr = _zzq_args;                                         \
    __asm__ volatile("mr 4,%1\n\t"                                \
                     "ld 3, 48(4)\n\t"                            \
                     __SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* %R3 = client_request ( %R4 ) */           \
                     "or 1,1,1\n\t"                               \
                     "mr %0,3"                                    \
                     : "=b" (_zzq_result)                         \
                     : "b" (_zzq_ptr)                             \
                     : "r3", "r4", "cc", "memory");               \
    _zzq_rlval = _zzq_result;                                     \
  }

#define VALGRIND_GET_NR_CONTEXT(_zzq_rlval)                       \
  { volatile OrigFn* _zzq_orig = &(_zzq_rlval);                   \
    register uint64_t __addr;                       \
    __asm__ volatile(__SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* %R3 = guest_NRADDR */                     \
                     "or 2,2,2\n\t"                               \
                     "mr %0,3"                                    \
                     : "=b" (__addr)                              \
                     :                                            \
                     : "r3", "cc", "memory"                       \
                    );                                            \
    _zzq_orig->nraddr = __addr;                                   \
    __asm__ volatile(__SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* %R3 = guest_NRADDR_GPR2 */                \
                     "or 4,4,4\n\t"                               \
                     "mr %0,3"                                    \
                     : "=b" (__addr)                              \
                     :                                            \
                     : "r3", "cc", "memory"                       \
                    );                                            \
    _zzq_orig->r2 = __addr;                                       \
  }

#define VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                   \
                     __SPECIAL_INSTRUCTION_PREAMBLE               \
                     /* branch-and-link-to-noredir *%R11 */       \
                     "or 3,3,3\n\t"

#endif /* PLAT_ppc64_aix5 */

/* Insert assembly code for other platforms here... */

#endif /* NVALGRIND */


/* ------------------------------------------------------------------ */
/* PLATFORM SPECIFICS for FUNCTION WRAPPING.  This is all very        */
/* ugly.  It's the least-worst tradeoff I can think of.               */
/* ------------------------------------------------------------------ */



#define I_WRAP_SONAME_FNNAME_ZU(soname,fnname)                    \
   _vgwZU_##soname##_##fnname

#define I_WRAP_SONAME_FNNAME_ZZ(soname,fnname)                    \
   _vgwZZ_##soname##_##fnname

#define VALGRIND_GET_ORIG_FN(_lval)  VALGRIND_GET_NR_CONTEXT(_lval)


#define CALL_FN_v_v(fnptr)                                        \
   do { volatile unsigned long _junk;                             \
        CALL_FN_W_v(_junk,fnptr); } while (0)

#define CALL_FN_v_W(fnptr, arg1)                                  \
   do { volatile unsigned long _junk;                             \
        CALL_FN_W_W(_junk,fnptr,arg1); } while (0)

#define CALL_FN_v_WW(fnptr, arg1,arg2)                            \
   do { volatile unsigned long _junk;                             \
        CALL_FN_W_WW(_junk,fnptr,arg1,arg2); } while (0)

#define CALL_FN_v_WWW(fnptr, arg1,arg2,arg3)                      \
   do { volatile unsigned long _junk;                             \
        CALL_FN_W_WWW(_junk,fnptr,arg1,arg2,arg3); } while (0)

/* ------------------------- x86-linux ------------------------- */

#if defined(PLAT_x86_linux)

#define __CALLER_SAVED_REGS /*"eax"*/ "ecx", "edx"


#define CALL_FN_W_v(lval, orig)                                   \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[1];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      __asm__ volatile(                                           \
         "movl (%%eax), %%eax\n\t"  /* target->%eax */            \
         VALGRIND_CALL_NOREDIR_EAX                                \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_W(lval, orig, arg1)                             \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[2];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      __asm__ volatile(                                           \
         "pushl 4(%%eax)\n\t"                                     \
         "movl (%%eax), %%eax\n\t"  /* target->%eax */            \
         VALGRIND_CALL_NOREDIR_EAX                                \
         "addl $4, %%esp\n"                                       \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WW(lval, orig, arg1,arg2)                       \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      __asm__ volatile(                                           \
         "pushl 8(%%eax)\n\t"                                     \
         "pushl 4(%%eax)\n\t"                                     \
         "movl (%%eax), %%eax\n\t"  /* target->%eax */            \
         VALGRIND_CALL_NOREDIR_EAX                                \
         "addl $8, %%esp\n"                                       \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WWW(lval, orig, arg1,arg2,arg3)                 \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[4];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      __asm__ volatile(                                           \
         "pushl 12(%%eax)\n\t"                                    \
         "pushl 8(%%eax)\n\t"                                     \
         "pushl 4(%%eax)\n\t"                                     \
         "movl (%%eax), %%eax\n\t"  /* target->%eax */            \
         VALGRIND_CALL_NOREDIR_EAX                                \
         "addl $12, %%esp\n"                                      \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WWWW(lval, orig, arg1,arg2,arg3,arg4)           \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[5];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      __asm__ volatile(                                           \
         "pushl 16(%%eax)\n\t"                                    \
         "pushl 12(%%eax)\n\t"                                    \
         "pushl 8(%%eax)\n\t"                                     \
         "pushl 4(%%eax)\n\t"                                     \
         "movl (%%eax), %%eax\n\t"  /* target->%eax */            \
         VALGRIND_CALL_NOREDIR_EAX                                \
         "addl $16, %%esp\n"                                      \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_5W(lval, orig, arg1,arg2,arg3,arg4,arg5)        \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[6];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      _argvec[5] = (unsigned long)(arg5);                         \
      __asm__ volatile(                                           \
         "pushl 20(%%eax)\n\t"                                    \
         "pushl 16(%%eax)\n\t"                                    \
         "pushl 12(%%eax)\n\t"                                    \
         "pushl 8(%%eax)\n\t"                                     \
         "pushl 4(%%eax)\n\t"                                     \
         "movl (%%eax), %%eax\n\t"  /* target->%eax */            \
         VALGRIND_CALL_NOREDIR_EAX                                \
         "addl $20, %%esp\n"                                      \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_6W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6)   \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[7];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      _argvec[5] = (unsigned long)(arg5);                         \
      _argvec[6] = (unsigned long)(arg6);                         \
      __asm__ volatile(                                           \
         "pushl 24(%%eax)\n\t"                                    \
         "pushl 20(%%eax)\n\t"                                    \
         "pushl 16(%%eax)\n\t"                                    \
         "pushl 12(%%eax)\n\t"                                    \
         "pushl 8(%%eax)\n\t"                                     \
         "pushl 4(%%eax)\n\t"                                     \
         "movl (%%eax), %%eax\n\t"  /* target->%eax */            \
         VALGRIND_CALL_NOREDIR_EAX                                \
         "addl $24, %%esp\n"                                      \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_7W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7)                            \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[8];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      _argvec[5] = (unsigned long)(arg5);                         \
      _argvec[6] = (unsigned long)(arg6);                         \
      _argvec[7] = (unsigned long)(arg7);                         \
      __asm__ volatile(                                           \
         "pushl 28(%%eax)\n\t"                                    \
         "pushl 24(%%eax)\n\t"                                    \
         "pushl 20(%%eax)\n\t"                                    \
         "pushl 16(%%eax)\n\t"                                    \
         "pushl 12(%%eax)\n\t"                                    \
         "pushl 8(%%eax)\n\t"                                     \
         "pushl 4(%%eax)\n\t"                                     \
         "movl (%%eax), %%eax\n\t"  /* target->%eax */            \
         VALGRIND_CALL_NOREDIR_EAX                                \
         "addl $28, %%esp\n"                                      \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_8W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7,arg8)                       \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[9];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      _argvec[5] = (unsigned long)(arg5);                         \
      _argvec[6] = (unsigned long)(arg6);                         \
      _argvec[7] = (unsigned long)(arg7);                         \
      _argvec[8] = (unsigned long)(arg8);                         \
      __asm__ volatile(                                           \
         "pushl 32(%%eax)\n\t"                                    \
         "pushl 28(%%eax)\n\t"                                    \
         "pushl 24(%%eax)\n\t"                                    \
         "pushl 20(%%eax)\n\t"                                    \
         "pushl 16(%%eax)\n\t"                                    \
         "pushl 12(%%eax)\n\t"                                    \
         "pushl 8(%%eax)\n\t"                                     \
         "pushl 4(%%eax)\n\t"                                     \
         "movl (%%eax), %%eax\n\t"  /* target->%eax */            \
         VALGRIND_CALL_NOREDIR_EAX                                \
         "addl $32, %%esp\n"                                      \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_9W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7,arg8,arg9)                  \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[10];                         \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      _argvec[5] = (unsigned long)(arg5);                         \
      _argvec[6] = (unsigned long)(arg6);                         \
      _argvec[7] = (unsigned long)(arg7);                         \
      _argvec[8] = (unsigned long)(arg8);                         \
      _argvec[9] = (unsigned long)(arg9);                         \
      __asm__ volatile(                                           \
         "pushl 36(%%eax)\n\t"                                    \
         "pushl 32(%%eax)\n\t"                                    \
         "pushl 28(%%eax)\n\t"                                    \
         "pushl 24(%%eax)\n\t"                                    \
         "pushl 20(%%eax)\n\t"                                    \
         "pushl 16(%%eax)\n\t"                                    \
         "pushl 12(%%eax)\n\t"                                    \
         "pushl 8(%%eax)\n\t"                                     \
         "pushl 4(%%eax)\n\t"                                     \
         "movl (%%eax), %%eax\n\t"  /* target->%eax */            \
         VALGRIND_CALL_NOREDIR_EAX                                \
         "addl $36, %%esp\n"                                      \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_10W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,  \
                                  arg7,arg8,arg9,arg10)           \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[11];                         \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      _argvec[5] = (unsigned long)(arg5);                         \
      _argvec[6] = (unsigned long)(arg6);                         \
      _argvec[7] = (unsigned long)(arg7);                         \
      _argvec[8] = (unsigned long)(arg8);                         \
      _argvec[9] = (unsigned long)(arg9);                         \
      _argvec[10] = (unsigned long)(arg10);                       \
      __asm__ volatile(                                           \
         "pushl 40(%%eax)\n\t"                                    \
         "pushl 36(%%eax)\n\t"                                    \
         "pushl 32(%%eax)\n\t"                                    \
         "pushl 28(%%eax)\n\t"                                    \
         "pushl 24(%%eax)\n\t"                                    \
         "pushl 20(%%eax)\n\t"                                    \
         "pushl 16(%%eax)\n\t"                                    \
         "pushl 12(%%eax)\n\t"                                    \
         "pushl 8(%%eax)\n\t"                                     \
         "pushl 4(%%eax)\n\t"                                     \
         "movl (%%eax), %%eax\n\t"  /* target->%eax */            \
         VALGRIND_CALL_NOREDIR_EAX                                \
         "addl $40, %%esp\n"                                      \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_11W(lval, orig, arg1,arg2,arg3,arg4,arg5,       \
                                  arg6,arg7,arg8,arg9,arg10,      \
                                  arg11)                          \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[12];                         \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      _argvec[5] = (unsigned long)(arg5);                         \
      _argvec[6] = (unsigned long)(arg6);                         \
      _argvec[7] = (unsigned long)(arg7);                         \
      _argvec[8] = (unsigned long)(arg8);                         \
      _argvec[9] = (unsigned long)(arg9);                         \
      _argvec[10] = (unsigned long)(arg10);                       \
      _argvec[11] = (unsigned long)(arg11);                       \
      __asm__ volatile(                                           \
         "pushl 44(%%eax)\n\t"                                    \
         "pushl 40(%%eax)\n\t"                                    \
         "pushl 36(%%eax)\n\t"                                    \
         "pushl 32(%%eax)\n\t"                                    \
         "pushl 28(%%eax)\n\t"                                    \
         "pushl 24(%%eax)\n\t"                                    \
         "pushl 20(%%eax)\n\t"                                    \
         "pushl 16(%%eax)\n\t"                                    \
         "pushl 12(%%eax)\n\t"                                    \
         "pushl 8(%%eax)\n\t"                                     \
         "pushl 4(%%eax)\n\t"                                     \
         "movl (%%eax), %%eax\n\t"  /* target->%eax */            \
         VALGRIND_CALL_NOREDIR_EAX                                \
         "addl $44, %%esp\n"                                      \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_12W(lval, orig, arg1,arg2,arg3,arg4,arg5,       \
                                  arg6,arg7,arg8,arg9,arg10,      \
                                  arg11,arg12)                    \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[13];                         \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      _argvec[5] = (unsigned long)(arg5);                         \
      _argvec[6] = (unsigned long)(arg6);                         \
      _argvec[7] = (unsigned long)(arg7);                         \
      _argvec[8] = (unsigned long)(arg8);                         \
      _argvec[9] = (unsigned long)(arg9);                         \
      _argvec[10] = (unsigned long)(arg10);                       \
      _argvec[11] = (unsigned long)(arg11);                       \
      _argvec[12] = (unsigned long)(arg12);                       \
      __asm__ volatile(                                           \
         "pushl 48(%%eax)\n\t"                                    \
         "pushl 44(%%eax)\n\t"                                    \
         "pushl 40(%%eax)\n\t"                                    \
         "pushl 36(%%eax)\n\t"                                    \
         "pushl 32(%%eax)\n\t"                                    \
         "pushl 28(%%eax)\n\t"                                    \
         "pushl 24(%%eax)\n\t"                                    \
         "pushl 20(%%eax)\n\t"                                    \
         "pushl 16(%%eax)\n\t"                                    \
         "pushl 12(%%eax)\n\t"                                    \
         "pushl 8(%%eax)\n\t"                                     \
         "pushl 4(%%eax)\n\t"                                     \
         "movl (%%eax), %%eax\n\t"  /* target->%eax */            \
         VALGRIND_CALL_NOREDIR_EAX                                \
         "addl $48, %%esp\n"                                      \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#endif /* PLAT_x86_linux */

/* ------------------------ amd64-linux ------------------------ */

#if defined(PLAT_amd64_linux)

/* ARGREGS: rdi rsi rdx rcx r8 r9 (the rest on stack in R-to-L order) */

/* These regs are trashed by the hidden call. */
#define __CALLER_SAVED_REGS /*"rax",*/ "rcx", "rdx", "rsi",       \
                            "rdi", "r8", "r9", "r10", "r11"



#define CALL_FN_W_v(lval, orig)                                   \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[1];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      __asm__ volatile(                                           \
         "subq $128,%%rsp\n\t"                                    \
         "movq (%%rax), %%rax\n\t"  /* target->%rax */            \
         VALGRIND_CALL_NOREDIR_RAX                                \
         "addq $128,%%rsp\n\t"                                    \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_W(lval, orig, arg1)                             \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[2];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      __asm__ volatile(                                           \
         "subq $128,%%rsp\n\t"                                    \
         "movq 8(%%rax), %%rdi\n\t"                               \
         "movq (%%rax), %%rax\n\t"  /* target->%rax */            \
         VALGRIND_CALL_NOREDIR_RAX                                \
         "addq $128,%%rsp\n\t"                                    \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WW(lval, orig, arg1,arg2)                       \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      __asm__ volatile(                                           \
         "subq $128,%%rsp\n\t"                                    \
         "movq 16(%%rax), %%rsi\n\t"                              \
         "movq 8(%%rax), %%rdi\n\t"                               \
         "movq (%%rax), %%rax\n\t"  /* target->%rax */            \
         VALGRIND_CALL_NOREDIR_RAX                                \
         "addq $128,%%rsp\n\t"                                    \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WWW(lval, orig, arg1,arg2,arg3)                 \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[4];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      __asm__ volatile(                                           \
         "subq $128,%%rsp\n\t"                                    \
         "movq 24(%%rax), %%rdx\n\t"                              \
         "movq 16(%%rax), %%rsi\n\t"                              \
         "movq 8(%%rax), %%rdi\n\t"                               \
         "movq (%%rax), %%rax\n\t"  /* target->%rax */            \
         VALGRIND_CALL_NOREDIR_RAX                                \
         "addq $128,%%rsp\n\t"                                    \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WWWW(lval, orig, arg1,arg2,arg3,arg4)           \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[5];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      __asm__ volatile(                                           \
         "subq $128,%%rsp\n\t"                                    \
         "movq 32(%%rax), %%rcx\n\t"                              \
         "movq 24(%%rax), %%rdx\n\t"                              \
         "movq 16(%%rax), %%rsi\n\t"                              \
         "movq 8(%%rax), %%rdi\n\t"                               \
         "movq (%%rax), %%rax\n\t"  /* target->%rax */            \
         VALGRIND_CALL_NOREDIR_RAX                                \
         "addq $128,%%rsp\n\t"                                    \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_5W(lval, orig, arg1,arg2,arg3,arg4,arg5)        \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[6];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      _argvec[5] = (unsigned long)(arg5);                         \
      __asm__ volatile(                                           \
         "subq $128,%%rsp\n\t"                                    \
         "movq 40(%%rax), %%r8\n\t"                               \
         "movq 32(%%rax), %%rcx\n\t"                              \
         "movq 24(%%rax), %%rdx\n\t"                              \
         "movq 16(%%rax), %%rsi\n\t"                              \
         "movq 8(%%rax), %%rdi\n\t"                               \
         "movq (%%rax), %%rax\n\t"  /* target->%rax */            \
         VALGRIND_CALL_NOREDIR_RAX                                \
         "addq $128,%%rsp\n\t"                                    \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_6W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6)   \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[7];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      _argvec[5] = (unsigned long)(arg5);                         \
      _argvec[6] = (unsigned long)(arg6);                         \
      __asm__ volatile(                                           \
         "subq $128,%%rsp\n\t"                                    \
         "movq 48(%%rax), %%r9\n\t"                               \
         "movq 40(%%rax), %%r8\n\t"                               \
         "movq 32(%%rax), %%rcx\n\t"                              \
         "movq 24(%%rax), %%rdx\n\t"                              \
         "movq 16(%%rax), %%rsi\n\t"                              \
         "movq 8(%%rax), %%rdi\n\t"                               \
         "movq (%%rax), %%rax\n\t"  /* target->%rax */            \
         "addq $128,%%rsp\n\t"                                    \
         VALGRIND_CALL_NOREDIR_RAX                                \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_7W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7)                            \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[8];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      _argvec[5] = (unsigned long)(arg5);                         \
      _argvec[6] = (unsigned long)(arg6);                         \
      _argvec[7] = (unsigned long)(arg7);                         \
      __asm__ volatile(                                           \
         "subq $128,%%rsp\n\t"                                    \
         "pushq 56(%%rax)\n\t"                                    \
         "movq 48(%%rax), %%r9\n\t"                               \
         "movq 40(%%rax), %%r8\n\t"                               \
         "movq 32(%%rax), %%rcx\n\t"                              \
         "movq 24(%%rax), %%rdx\n\t"                              \
         "movq 16(%%rax), %%rsi\n\t"                              \
         "movq 8(%%rax), %%rdi\n\t"                               \
         "movq (%%rax), %%rax\n\t"  /* target->%rax */            \
         VALGRIND_CALL_NOREDIR_RAX                                \
         "addq $8, %%rsp\n"                                       \
         "addq $128,%%rsp\n\t"                                    \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_8W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7,arg8)                       \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[9];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      _argvec[5] = (unsigned long)(arg5);                         \
      _argvec[6] = (unsigned long)(arg6);                         \
      _argvec[7] = (unsigned long)(arg7);                         \
      _argvec[8] = (unsigned long)(arg8);                         \
      __asm__ volatile(                                           \
         "subq $128,%%rsp\n\t"                                    \
         "pushq 64(%%rax)\n\t"                                    \
         "pushq 56(%%rax)\n\t"                                    \
         "movq 48(%%rax), %%r9\n\t"                               \
         "movq 40(%%rax), %%r8\n\t"                               \
         "movq 32(%%rax), %%rcx\n\t"                              \
         "movq 24(%%rax), %%rdx\n\t"                              \
         "movq 16(%%rax), %%rsi\n\t"                              \
         "movq 8(%%rax), %%rdi\n\t"                               \
         "movq (%%rax), %%rax\n\t"  /* target->%rax */            \
         VALGRIND_CALL_NOREDIR_RAX                                \
         "addq $16, %%rsp\n"                                      \
         "addq $128,%%rsp\n\t"                                    \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_9W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7,arg8,arg9)                  \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[10];                         \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      _argvec[5] = (unsigned long)(arg5);                         \
      _argvec[6] = (unsigned long)(arg6);                         \
      _argvec[7] = (unsigned long)(arg7);                         \
      _argvec[8] = (unsigned long)(arg8);                         \
      _argvec[9] = (unsigned long)(arg9);                         \
      __asm__ volatile(                                           \
         "subq $128,%%rsp\n\t"                                    \
         "pushq 72(%%rax)\n\t"                                    \
         "pushq 64(%%rax)\n\t"                                    \
         "pushq 56(%%rax)\n\t"                                    \
         "movq 48(%%rax), %%r9\n\t"                               \
         "movq 40(%%rax), %%r8\n\t"                               \
         "movq 32(%%rax), %%rcx\n\t"                              \
         "movq 24(%%rax), %%rdx\n\t"                              \
         "movq 16(%%rax), %%rsi\n\t"                              \
         "movq 8(%%rax), %%rdi\n\t"                               \
         "movq (%%rax), %%rax\n\t"  /* target->%rax */            \
         VALGRIND_CALL_NOREDIR_RAX                                \
         "addq $24, %%rsp\n"                                      \
         "addq $128,%%rsp\n\t"                                    \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_10W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,  \
                                  arg7,arg8,arg9,arg10)           \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[11];                         \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      _argvec[5] = (unsigned long)(arg5);                         \
      _argvec[6] = (unsigned long)(arg6);                         \
      _argvec[7] = (unsigned long)(arg7);                         \
      _argvec[8] = (unsigned long)(arg8);                         \
      _argvec[9] = (unsigned long)(arg9);                         \
      _argvec[10] = (unsigned long)(arg10);                       \
      __asm__ volatile(                                           \
         "subq $128,%%rsp\n\t"                                    \
         "pushq 80(%%rax)\n\t"                                    \
         "pushq 72(%%rax)\n\t"                                    \
         "pushq 64(%%rax)\n\t"                                    \
         "pushq 56(%%rax)\n\t"                                    \
         "movq 48(%%rax), %%r9\n\t"                               \
         "movq 40(%%rax), %%r8\n\t"                               \
         "movq 32(%%rax), %%rcx\n\t"                              \
         "movq 24(%%rax), %%rdx\n\t"                              \
         "movq 16(%%rax), %%rsi\n\t"                              \
         "movq 8(%%rax), %%rdi\n\t"                               \
         "movq (%%rax), %%rax\n\t"  /* target->%rax */            \
         VALGRIND_CALL_NOREDIR_RAX                                \
         "addq $32, %%rsp\n"                                      \
         "addq $128,%%rsp\n\t"                                    \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_11W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,  \
                                  arg7,arg8,arg9,arg10,arg11)     \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[12];                         \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      _argvec[5] = (unsigned long)(arg5);                         \
      _argvec[6] = (unsigned long)(arg6);                         \
      _argvec[7] = (unsigned long)(arg7);                         \
      _argvec[8] = (unsigned long)(arg8);                         \
      _argvec[9] = (unsigned long)(arg9);                         \
      _argvec[10] = (unsigned long)(arg10);                       \
      _argvec[11] = (unsigned long)(arg11);                       \
      __asm__ volatile(                                           \
         "subq $128,%%rsp\n\t"                                    \
         "pushq 88(%%rax)\n\t"                                    \
         "pushq 80(%%rax)\n\t"                                    \
         "pushq 72(%%rax)\n\t"                                    \
         "pushq 64(%%rax)\n\t"                                    \
         "pushq 56(%%rax)\n\t"                                    \
         "movq 48(%%rax), %%r9\n\t"                               \
         "movq 40(%%rax), %%r8\n\t"                               \
         "movq 32(%%rax), %%rcx\n\t"                              \
         "movq 24(%%rax), %%rdx\n\t"                              \
         "movq 16(%%rax), %%rsi\n\t"                              \
         "movq 8(%%rax), %%rdi\n\t"                               \
         "movq (%%rax), %%rax\n\t"  /* target->%rax */            \
         VALGRIND_CALL_NOREDIR_RAX                                \
         "addq $40, %%rsp\n"                                      \
         "addq $128,%%rsp\n\t"                                    \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_12W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,  \
                                arg7,arg8,arg9,arg10,arg11,arg12) \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[13];                         \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)(arg1);                         \
      _argvec[2] = (unsigned long)(arg2);                         \
      _argvec[3] = (unsigned long)(arg3);                         \
      _argvec[4] = (unsigned long)(arg4);                         \
      _argvec[5] = (unsigned long)(arg5);                         \
      _argvec[6] = (unsigned long)(arg6);                         \
      _argvec[7] = (unsigned long)(arg7);                         \
      _argvec[8] = (unsigned long)(arg8);                         \
      _argvec[9] = (unsigned long)(arg9);                         \
      _argvec[10] = (unsigned long)(arg10);                       \
      _argvec[11] = (unsigned long)(arg11);                       \
      _argvec[12] = (unsigned long)(arg12);                       \
      __asm__ volatile(                                           \
         "subq $128,%%rsp\n\t"                                    \
         "pushq 96(%%rax)\n\t"                                    \
         "pushq 88(%%rax)\n\t"                                    \
         "pushq 80(%%rax)\n\t"                                    \
         "pushq 72(%%rax)\n\t"                                    \
         "pushq 64(%%rax)\n\t"                                    \
         "pushq 56(%%rax)\n\t"                                    \
         "movq 48(%%rax), %%r9\n\t"                               \
         "movq 40(%%rax), %%r8\n\t"                               \
         "movq 32(%%rax), %%rcx\n\t"                              \
         "movq 24(%%rax), %%rdx\n\t"                              \
         "movq 16(%%rax), %%rsi\n\t"                              \
         "movq 8(%%rax), %%rdi\n\t"                               \
         "movq (%%rax), %%rax\n\t"  /* target->%rax */            \
         VALGRIND_CALL_NOREDIR_RAX                                \
         "addq $48, %%rsp\n"                                      \
         "addq $128,%%rsp\n\t"                                    \
         : /*out*/   "=a" (_res)                                  \
         : /*in*/    "a" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#endif /* PLAT_amd64_linux */

/* ------------------------ ppc32-linux ------------------------ */

#if defined(PLAT_ppc32_linux)


/* ARGREGS: r3 r4 r5 r6 r7 r8 r9 r10 (the rest on stack somewhere) */

/* These regs are trashed by the hidden call. */
#define __CALLER_SAVED_REGS                                       \
   "lr", "ctr", "xer",                                            \
   "cr0", "cr1", "cr2", "cr3", "cr4", "cr5", "cr6", "cr7",        \
   "r0", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10",   \
   "r11", "r12", "r13"


#define CALL_FN_W_v(lval, orig)                                   \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[1];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "lwz 11,0(11)\n\t"  /* target->r11 */                    \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr %0,3"                                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_W(lval, orig, arg1)                             \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[2];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)arg1;                           \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "lwz 3,4(11)\n\t"   /* arg1->r3 */                       \
         "lwz 11,0(11)\n\t"  /* target->r11 */                    \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr %0,3"                                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WW(lval, orig, arg1,arg2)                       \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)arg1;                           \
      _argvec[2] = (unsigned long)arg2;                           \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "lwz 3,4(11)\n\t"   /* arg1->r3 */                       \
         "lwz 4,8(11)\n\t"                                        \
         "lwz 11,0(11)\n\t"  /* target->r11 */                    \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr %0,3"                                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WWW(lval, orig, arg1,arg2,arg3)                 \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[4];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)arg1;                           \
      _argvec[2] = (unsigned long)arg2;                           \
      _argvec[3] = (unsigned long)arg3;                           \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "lwz 3,4(11)\n\t"   /* arg1->r3 */                       \
         "lwz 4,8(11)\n\t"                                        \
         "lwz 5,12(11)\n\t"                                       \
         "lwz 11,0(11)\n\t"  /* target->r11 */                    \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr %0,3"                                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WWWW(lval, orig, arg1,arg2,arg3,arg4)           \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[5];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)arg1;                           \
      _argvec[2] = (unsigned long)arg2;                           \
      _argvec[3] = (unsigned long)arg3;                           \
      _argvec[4] = (unsigned long)arg4;                           \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "lwz 3,4(11)\n\t"   /* arg1->r3 */                       \
         "lwz 4,8(11)\n\t"                                        \
         "lwz 5,12(11)\n\t"                                       \
         "lwz 6,16(11)\n\t"  /* arg4->r6 */                       \
         "lwz 11,0(11)\n\t"  /* target->r11 */                    \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr %0,3"                                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_5W(lval, orig, arg1,arg2,arg3,arg4,arg5)        \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[6];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)arg1;                           \
      _argvec[2] = (unsigned long)arg2;                           \
      _argvec[3] = (unsigned long)arg3;                           \
      _argvec[4] = (unsigned long)arg4;                           \
      _argvec[5] = (unsigned long)arg5;                           \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "lwz 3,4(11)\n\t"   /* arg1->r3 */                       \
         "lwz 4,8(11)\n\t"                                        \
         "lwz 5,12(11)\n\t"                                       \
         "lwz 6,16(11)\n\t"  /* arg4->r6 */                       \
         "lwz 7,20(11)\n\t"                                       \
         "lwz 11,0(11)\n\t"  /* target->r11 */                    \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr %0,3"                                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_6W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6)   \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[7];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)arg1;                           \
      _argvec[2] = (unsigned long)arg2;                           \
      _argvec[3] = (unsigned long)arg3;                           \
      _argvec[4] = (unsigned long)arg4;                           \
      _argvec[5] = (unsigned long)arg5;                           \
      _argvec[6] = (unsigned long)arg6;                           \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "lwz 3,4(11)\n\t"   /* arg1->r3 */                       \
         "lwz 4,8(11)\n\t"                                        \
         "lwz 5,12(11)\n\t"                                       \
         "lwz 6,16(11)\n\t"  /* arg4->r6 */                       \
         "lwz 7,20(11)\n\t"                                       \
         "lwz 8,24(11)\n\t"                                       \
         "lwz 11,0(11)\n\t"  /* target->r11 */                    \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr %0,3"                                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_7W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7)                            \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[8];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)arg1;                           \
      _argvec[2] = (unsigned long)arg2;                           \
      _argvec[3] = (unsigned long)arg3;                           \
      _argvec[4] = (unsigned long)arg4;                           \
      _argvec[5] = (unsigned long)arg5;                           \
      _argvec[6] = (unsigned long)arg6;                           \
      _argvec[7] = (unsigned long)arg7;                           \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "lwz 3,4(11)\n\t"   /* arg1->r3 */                       \
         "lwz 4,8(11)\n\t"                                        \
         "lwz 5,12(11)\n\t"                                       \
         "lwz 6,16(11)\n\t"  /* arg4->r6 */                       \
         "lwz 7,20(11)\n\t"                                       \
         "lwz 8,24(11)\n\t"                                       \
         "lwz 9,28(11)\n\t"                                       \
         "lwz 11,0(11)\n\t"  /* target->r11 */                    \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr %0,3"                                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_8W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7,arg8)                       \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[9];                          \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)arg1;                           \
      _argvec[2] = (unsigned long)arg2;                           \
      _argvec[3] = (unsigned long)arg3;                           \
      _argvec[4] = (unsigned long)arg4;                           \
      _argvec[5] = (unsigned long)arg5;                           \
      _argvec[6] = (unsigned long)arg6;                           \
      _argvec[7] = (unsigned long)arg7;                           \
      _argvec[8] = (unsigned long)arg8;                           \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "lwz 3,4(11)\n\t"   /* arg1->r3 */                       \
         "lwz 4,8(11)\n\t"                                        \
         "lwz 5,12(11)\n\t"                                       \
         "lwz 6,16(11)\n\t"  /* arg4->r6 */                       \
         "lwz 7,20(11)\n\t"                                       \
         "lwz 8,24(11)\n\t"                                       \
         "lwz 9,28(11)\n\t"                                       \
         "lwz 10,32(11)\n\t" /* arg8->r10 */                      \
         "lwz 11,0(11)\n\t"  /* target->r11 */                    \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr %0,3"                                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_9W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7,arg8,arg9)                  \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[10];                         \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)arg1;                           \
      _argvec[2] = (unsigned long)arg2;                           \
      _argvec[3] = (unsigned long)arg3;                           \
      _argvec[4] = (unsigned long)arg4;                           \
      _argvec[5] = (unsigned long)arg5;                           \
      _argvec[6] = (unsigned long)arg6;                           \
      _argvec[7] = (unsigned long)arg7;                           \
      _argvec[8] = (unsigned long)arg8;                           \
      _argvec[9] = (unsigned long)arg9;                           \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "addi 1,1,-16\n\t"                                       \
         /* arg9 */                                               \
         "lwz 3,36(11)\n\t"                                       \
         "stw 3,8(1)\n\t"                                         \
         /* args1-8 */                                            \
         "lwz 3,4(11)\n\t"   /* arg1->r3 */                       \
         "lwz 4,8(11)\n\t"                                        \
         "lwz 5,12(11)\n\t"                                       \
         "lwz 6,16(11)\n\t"  /* arg4->r6 */                       \
         "lwz 7,20(11)\n\t"                                       \
         "lwz 8,24(11)\n\t"                                       \
         "lwz 9,28(11)\n\t"                                       \
         "lwz 10,32(11)\n\t" /* arg8->r10 */                      \
         "lwz 11,0(11)\n\t"  /* target->r11 */                    \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "addi 1,1,16\n\t"                                        \
         "mr %0,3"                                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_10W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,  \
                                  arg7,arg8,arg9,arg10)           \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[11];                         \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)arg1;                           \
      _argvec[2] = (unsigned long)arg2;                           \
      _argvec[3] = (unsigned long)arg3;                           \
      _argvec[4] = (unsigned long)arg4;                           \
      _argvec[5] = (unsigned long)arg5;                           \
      _argvec[6] = (unsigned long)arg6;                           \
      _argvec[7] = (unsigned long)arg7;                           \
      _argvec[8] = (unsigned long)arg8;                           \
      _argvec[9] = (unsigned long)arg9;                           \
      _argvec[10] = (unsigned long)arg10;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "addi 1,1,-16\n\t"                                       \
         /* arg10 */                                              \
         "lwz 3,40(11)\n\t"                                       \
         "stw 3,12(1)\n\t"                                        \
         /* arg9 */                                               \
         "lwz 3,36(11)\n\t"                                       \
         "stw 3,8(1)\n\t"                                         \
         /* args1-8 */                                            \
         "lwz 3,4(11)\n\t"   /* arg1->r3 */                       \
         "lwz 4,8(11)\n\t"                                        \
         "lwz 5,12(11)\n\t"                                       \
         "lwz 6,16(11)\n\t"  /* arg4->r6 */                       \
         "lwz 7,20(11)\n\t"                                       \
         "lwz 8,24(11)\n\t"                                       \
         "lwz 9,28(11)\n\t"                                       \
         "lwz 10,32(11)\n\t" /* arg8->r10 */                      \
         "lwz 11,0(11)\n\t"  /* target->r11 */                    \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "addi 1,1,16\n\t"                                        \
         "mr %0,3"                                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_11W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,  \
                                  arg7,arg8,arg9,arg10,arg11)     \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[12];                         \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)arg1;                           \
      _argvec[2] = (unsigned long)arg2;                           \
      _argvec[3] = (unsigned long)arg3;                           \
      _argvec[4] = (unsigned long)arg4;                           \
      _argvec[5] = (unsigned long)arg5;                           \
      _argvec[6] = (unsigned long)arg6;                           \
      _argvec[7] = (unsigned long)arg7;                           \
      _argvec[8] = (unsigned long)arg8;                           \
      _argvec[9] = (unsigned long)arg9;                           \
      _argvec[10] = (unsigned long)arg10;                         \
      _argvec[11] = (unsigned long)arg11;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "addi 1,1,-32\n\t"                                       \
         /* arg11 */                                              \
         "lwz 3,44(11)\n\t"                                       \
         "stw 3,16(1)\n\t"                                        \
         /* arg10 */                                              \
         "lwz 3,40(11)\n\t"                                       \
         "stw 3,12(1)\n\t"                                        \
         /* arg9 */                                               \
         "lwz 3,36(11)\n\t"                                       \
         "stw 3,8(1)\n\t"                                         \
         /* args1-8 */                                            \
         "lwz 3,4(11)\n\t"   /* arg1->r3 */                       \
         "lwz 4,8(11)\n\t"                                        \
         "lwz 5,12(11)\n\t"                                       \
         "lwz 6,16(11)\n\t"  /* arg4->r6 */                       \
         "lwz 7,20(11)\n\t"                                       \
         "lwz 8,24(11)\n\t"                                       \
         "lwz 9,28(11)\n\t"                                       \
         "lwz 10,32(11)\n\t" /* arg8->r10 */                      \
         "lwz 11,0(11)\n\t"  /* target->r11 */                    \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "addi 1,1,32\n\t"                                        \
         "mr %0,3"                                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_12W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,  \
                                arg7,arg8,arg9,arg10,arg11,arg12) \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[13];                         \
      volatile unsigned long _res;                                \
      _argvec[0] = (unsigned long)_orig.nraddr;                   \
      _argvec[1] = (unsigned long)arg1;                           \
      _argvec[2] = (unsigned long)arg2;                           \
      _argvec[3] = (unsigned long)arg3;                           \
      _argvec[4] = (unsigned long)arg4;                           \
      _argvec[5] = (unsigned long)arg5;                           \
      _argvec[6] = (unsigned long)arg6;                           \
      _argvec[7] = (unsigned long)arg7;                           \
      _argvec[8] = (unsigned long)arg8;                           \
      _argvec[9] = (unsigned long)arg9;                           \
      _argvec[10] = (unsigned long)arg10;                         \
      _argvec[11] = (unsigned long)arg11;                         \
      _argvec[12] = (unsigned long)arg12;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "addi 1,1,-32\n\t"                                       \
         /* arg12 */                                              \
         "lwz 3,48(11)\n\t"                                       \
         "stw 3,20(1)\n\t"                                        \
         /* arg11 */                                              \
         "lwz 3,44(11)\n\t"                                       \
         "stw 3,16(1)\n\t"                                        \
         /* arg10 */                                              \
         "lwz 3,40(11)\n\t"                                       \
         "stw 3,12(1)\n\t"                                        \
         /* arg9 */                                               \
         "lwz 3,36(11)\n\t"                                       \
         "stw 3,8(1)\n\t"                                         \
         /* args1-8 */                                            \
         "lwz 3,4(11)\n\t"   /* arg1->r3 */                       \
         "lwz 4,8(11)\n\t"                                        \
         "lwz 5,12(11)\n\t"                                       \
         "lwz 6,16(11)\n\t"  /* arg4->r6 */                       \
         "lwz 7,20(11)\n\t"                                       \
         "lwz 8,24(11)\n\t"                                       \
         "lwz 9,28(11)\n\t"                                       \
         "lwz 10,32(11)\n\t" /* arg8->r10 */                      \
         "lwz 11,0(11)\n\t"  /* target->r11 */                    \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "addi 1,1,32\n\t"                                        \
         "mr %0,3"                                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[0])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#endif /* PLAT_ppc32_linux */

/* ------------------------ ppc64-linux ------------------------ */

#if defined(PLAT_ppc64_linux)

/* ARGREGS: r3 r4 r5 r6 r7 r8 r9 r10 (the rest on stack somewhere) */

/* These regs are trashed by the hidden call. */
#define __CALLER_SAVED_REGS                                       \
   "lr", "ctr", "xer",                                            \
   "cr0", "cr1", "cr2", "cr3", "cr4", "cr5", "cr6", "cr7",        \
   "r0", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10",   \
   "r11", "r12", "r13"


#define CALL_FN_W_v(lval, orig)                                   \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+0];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1] = (unsigned long)_orig.r2;                       \
      _argvec[2] = (unsigned long)_orig.nraddr;                   \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "std 2,-16(11)\n\t"  /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld 2,-16(11)" /* restore tocptr */                      \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_W(lval, orig, arg1)                             \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+1];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "std 2,-16(11)\n\t"  /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld 2,-16(11)" /* restore tocptr */                      \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WW(lval, orig, arg1,arg2)                       \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+2];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "std 2,-16(11)\n\t"  /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld 2,-16(11)" /* restore tocptr */                      \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WWW(lval, orig, arg1,arg2,arg3)                 \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+3];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "std 2,-16(11)\n\t"  /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld 2,-16(11)" /* restore tocptr */                      \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WWWW(lval, orig, arg1,arg2,arg3,arg4)           \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+4];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "std 2,-16(11)\n\t"  /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld 2,-16(11)" /* restore tocptr */                      \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_5W(lval, orig, arg1,arg2,arg3,arg4,arg5)        \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+5];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "std 2,-16(11)\n\t"  /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld   7, 40(11)\n\t" /* arg5->r7 */                      \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld 2,-16(11)" /* restore tocptr */                      \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_6W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6)   \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+6];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "std 2,-16(11)\n\t"  /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld   7, 40(11)\n\t" /* arg5->r7 */                      \
         "ld   8, 48(11)\n\t" /* arg6->r8 */                      \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld 2,-16(11)" /* restore tocptr */                      \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_7W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7)                            \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+7];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "std 2,-16(11)\n\t"  /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld   7, 40(11)\n\t" /* arg5->r7 */                      \
         "ld   8, 48(11)\n\t" /* arg6->r8 */                      \
         "ld   9, 56(11)\n\t" /* arg7->r9 */                      \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld 2,-16(11)" /* restore tocptr */                      \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_8W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7,arg8)                       \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+8];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      _argvec[2+8] = (unsigned long)arg8;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "std 2,-16(11)\n\t"  /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld   7, 40(11)\n\t" /* arg5->r7 */                      \
         "ld   8, 48(11)\n\t" /* arg6->r8 */                      \
         "ld   9, 56(11)\n\t" /* arg7->r9 */                      \
         "ld  10, 64(11)\n\t" /* arg8->r10 */                     \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld 2,-16(11)" /* restore tocptr */                      \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_9W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7,arg8,arg9)                  \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+9];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      _argvec[2+8] = (unsigned long)arg8;                         \
      _argvec[2+9] = (unsigned long)arg9;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "std 2,-16(11)\n\t"  /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "addi 1,1,-128\n\t"  /* expand stack frame */            \
         /* arg9 */                                               \
         "ld  3,72(11)\n\t"                                       \
         "std 3,112(1)\n\t"                                       \
         /* args1-8 */                                            \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld   7, 40(11)\n\t" /* arg5->r7 */                      \
         "ld   8, 48(11)\n\t" /* arg6->r8 */                      \
         "ld   9, 56(11)\n\t" /* arg7->r9 */                      \
         "ld  10, 64(11)\n\t" /* arg8->r10 */                     \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld 2,-16(11)\n\t" /* restore tocptr */                  \
         "addi 1,1,128"     /* restore frame */                   \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_10W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,  \
                                  arg7,arg8,arg9,arg10)           \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+10];                       \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      _argvec[2+8] = (unsigned long)arg8;                         \
      _argvec[2+9] = (unsigned long)arg9;                         \
      _argvec[2+10] = (unsigned long)arg10;                       \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "std 2,-16(11)\n\t"  /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "addi 1,1,-128\n\t"  /* expand stack frame */            \
         /* arg10 */                                              \
         "ld  3,80(11)\n\t"                                       \
         "std 3,120(1)\n\t"                                       \
         /* arg9 */                                               \
         "ld  3,72(11)\n\t"                                       \
         "std 3,112(1)\n\t"                                       \
         /* args1-8 */                                            \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld   7, 40(11)\n\t" /* arg5->r7 */                      \
         "ld   8, 48(11)\n\t" /* arg6->r8 */                      \
         "ld   9, 56(11)\n\t" /* arg7->r9 */                      \
         "ld  10, 64(11)\n\t" /* arg8->r10 */                     \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld 2,-16(11)\n\t" /* restore tocptr */                  \
         "addi 1,1,128"     /* restore frame */                   \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_11W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,  \
                                  arg7,arg8,arg9,arg10,arg11)     \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+11];                       \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      _argvec[2+8] = (unsigned long)arg8;                         \
      _argvec[2+9] = (unsigned long)arg9;                         \
      _argvec[2+10] = (unsigned long)arg10;                       \
      _argvec[2+11] = (unsigned long)arg11;                       \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "std 2,-16(11)\n\t"  /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "addi 1,1,-144\n\t"  /* expand stack frame */            \
         /* arg11 */                                              \
         "ld  3,88(11)\n\t"                                       \
         "std 3,128(1)\n\t"                                       \
         /* arg10 */                                              \
         "ld  3,80(11)\n\t"                                       \
         "std 3,120(1)\n\t"                                       \
         /* arg9 */                                               \
         "ld  3,72(11)\n\t"                                       \
         "std 3,112(1)\n\t"                                       \
         /* args1-8 */                                            \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld   7, 40(11)\n\t" /* arg5->r7 */                      \
         "ld   8, 48(11)\n\t" /* arg6->r8 */                      \
         "ld   9, 56(11)\n\t" /* arg7->r9 */                      \
         "ld  10, 64(11)\n\t" /* arg8->r10 */                     \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld 2,-16(11)\n\t" /* restore tocptr */                  \
         "addi 1,1,144"     /* restore frame */                   \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_12W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,  \
                                arg7,arg8,arg9,arg10,arg11,arg12) \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+12];                       \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      _argvec[2+8] = (unsigned long)arg8;                         \
      _argvec[2+9] = (unsigned long)arg9;                         \
      _argvec[2+10] = (unsigned long)arg10;                       \
      _argvec[2+11] = (unsigned long)arg11;                       \
      _argvec[2+12] = (unsigned long)arg12;                       \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         "std 2,-16(11)\n\t"  /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "addi 1,1,-144\n\t"  /* expand stack frame */            \
         /* arg12 */                                              \
         "ld  3,96(11)\n\t"                                       \
         "std 3,136(1)\n\t"                                       \
         /* arg11 */                                              \
         "ld  3,88(11)\n\t"                                       \
         "std 3,128(1)\n\t"                                       \
         /* arg10 */                                              \
         "ld  3,80(11)\n\t"                                       \
         "std 3,120(1)\n\t"                                       \
         /* arg9 */                                               \
         "ld  3,72(11)\n\t"                                       \
         "std 3,112(1)\n\t"                                       \
         /* args1-8 */                                            \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld   7, 40(11)\n\t" /* arg5->r7 */                      \
         "ld   8, 48(11)\n\t" /* arg6->r8 */                      \
         "ld   9, 56(11)\n\t" /* arg7->r9 */                      \
         "ld  10, 64(11)\n\t" /* arg8->r10 */                     \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld 2,-16(11)\n\t" /* restore tocptr */                  \
         "addi 1,1,144"     /* restore frame */                   \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#endif /* PLAT_ppc64_linux */

/* ------------------------ ppc32-aix5 ------------------------- */

#if defined(PLAT_ppc32_aix5)

/* ARGREGS: r3 r4 r5 r6 r7 r8 r9 r10 (the rest on stack somewhere) */

/* These regs are trashed by the hidden call. */
#define __CALLER_SAVED_REGS                                       \
   "lr", "ctr", "xer",                                            \
   "cr0", "cr1", "cr2", "cr3", "cr4", "cr5", "cr6", "cr7",        \
   "r0", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10",   \
   "r11", "r12", "r13"


#define VG_EXPAND_FRAME_BY_trashes_r3(_n_fr)                      \
         "addi 1,1,-" #_n_fr "\n\t"                               \
         "lwz  3," #_n_fr "(1)\n\t"                               \
         "stw  3,0(1)\n\t"

#define VG_CONTRACT_FRAME_BY(_n_fr)                               \
         "addi 1,1," #_n_fr "\n\t"


#define CALL_FN_W_v(lval, orig)                                   \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+0];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1] = (unsigned long)_orig.r2;                       \
      _argvec[2] = (unsigned long)_orig.nraddr;                   \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "stw  2,-8(11)\n\t"  /* save tocptr */                   \
         "lwz  2,-4(11)\n\t"  /* use nraddr's tocptr */           \
         "lwz 11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "lwz 2,-8(11)\n\t" /* restore tocptr */                  \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_W(lval, orig, arg1)                             \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+1];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "stw  2,-8(11)\n\t"  /* save tocptr */                   \
         "lwz  2,-4(11)\n\t"  /* use nraddr's tocptr */           \
         "lwz  3, 4(11)\n\t"  /* arg1->r3 */                      \
         "lwz 11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "lwz 2,-8(11)\n\t" /* restore tocptr */                  \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WW(lval, orig, arg1,arg2)                       \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+2];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "stw  2,-8(11)\n\t"  /* save tocptr */                   \
         "lwz  2,-4(11)\n\t"  /* use nraddr's tocptr */           \
         "lwz  3, 4(11)\n\t"  /* arg1->r3 */                      \
         "lwz  4, 8(11)\n\t"  /* arg2->r4 */                      \
         "lwz 11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "lwz 2,-8(11)\n\t" /* restore tocptr */                  \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WWW(lval, orig, arg1,arg2,arg3)                 \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+3];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "stw  2,-8(11)\n\t"  /* save tocptr */                   \
         "lwz  2,-4(11)\n\t"  /* use nraddr's tocptr */           \
         "lwz  3, 4(11)\n\t"  /* arg1->r3 */                      \
         "lwz  4, 8(11)\n\t"  /* arg2->r4 */                      \
         "lwz  5, 12(11)\n\t" /* arg3->r5 */                      \
         "lwz 11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "lwz 2,-8(11)\n\t" /* restore tocptr */                  \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WWWW(lval, orig, arg1,arg2,arg3,arg4)           \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+4];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "stw  2,-8(11)\n\t"  /* save tocptr */                   \
         "lwz  2,-4(11)\n\t"  /* use nraddr's tocptr */           \
         "lwz  3, 4(11)\n\t"  /* arg1->r3 */                      \
         "lwz  4, 8(11)\n\t"  /* arg2->r4 */                      \
         "lwz  5, 12(11)\n\t" /* arg3->r5 */                      \
         "lwz  6, 16(11)\n\t" /* arg4->r6 */                      \
         "lwz 11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "lwz 2,-8(11)\n\t" /* restore tocptr */                  \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_5W(lval, orig, arg1,arg2,arg3,arg4,arg5)        \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+5];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "stw  2,-8(11)\n\t"  /* save tocptr */                   \
         "lwz  2,-4(11)\n\t"  /* use nraddr's tocptr */           \
         "lwz  3, 4(11)\n\t"  /* arg1->r3 */                      \
         "lwz  4, 8(11)\n\t" /* arg2->r4 */                       \
         "lwz  5, 12(11)\n\t" /* arg3->r5 */                      \
         "lwz  6, 16(11)\n\t" /* arg4->r6 */                      \
         "lwz  7, 20(11)\n\t" /* arg5->r7 */                      \
         "lwz 11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "lwz 2,-8(11)\n\t" /* restore tocptr */                  \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_6W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6)   \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+6];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "stw  2,-8(11)\n\t"  /* save tocptr */                   \
         "lwz  2,-4(11)\n\t"  /* use nraddr's tocptr */           \
         "lwz  3, 4(11)\n\t"  /* arg1->r3 */                      \
         "lwz  4, 8(11)\n\t"  /* arg2->r4 */                      \
         "lwz  5, 12(11)\n\t" /* arg3->r5 */                      \
         "lwz  6, 16(11)\n\t" /* arg4->r6 */                      \
         "lwz  7, 20(11)\n\t" /* arg5->r7 */                      \
         "lwz  8, 24(11)\n\t" /* arg6->r8 */                      \
         "lwz 11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "lwz 2,-8(11)\n\t" /* restore tocptr */                  \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_7W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7)                            \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+7];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "stw  2,-8(11)\n\t"  /* save tocptr */                   \
         "lwz  2,-4(11)\n\t"  /* use nraddr's tocptr */           \
         "lwz  3, 4(11)\n\t"  /* arg1->r3 */                      \
         "lwz  4, 8(11)\n\t"  /* arg2->r4 */                      \
         "lwz  5, 12(11)\n\t" /* arg3->r5 */                      \
         "lwz  6, 16(11)\n\t" /* arg4->r6 */                      \
         "lwz  7, 20(11)\n\t" /* arg5->r7 */                      \
         "lwz  8, 24(11)\n\t" /* arg6->r8 */                      \
         "lwz  9, 28(11)\n\t" /* arg7->r9 */                      \
         "lwz 11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "lwz 2,-8(11)\n\t" /* restore tocptr */                  \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_8W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7,arg8)                       \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+8];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      _argvec[2+8] = (unsigned long)arg8;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "stw  2,-8(11)\n\t"  /* save tocptr */                   \
         "lwz  2,-4(11)\n\t"  /* use nraddr's tocptr */           \
         "lwz  3, 4(11)\n\t"  /* arg1->r3 */                      \
         "lwz  4, 8(11)\n\t"  /* arg2->r4 */                      \
         "lwz  5, 12(11)\n\t" /* arg3->r5 */                      \
         "lwz  6, 16(11)\n\t" /* arg4->r6 */                      \
         "lwz  7, 20(11)\n\t" /* arg5->r7 */                      \
         "lwz  8, 24(11)\n\t" /* arg6->r8 */                      \
         "lwz  9, 28(11)\n\t" /* arg7->r9 */                      \
         "lwz 10, 32(11)\n\t" /* arg8->r10 */                     \
         "lwz 11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "lwz 2,-8(11)\n\t" /* restore tocptr */                  \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_9W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7,arg8,arg9)                  \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+9];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      _argvec[2+8] = (unsigned long)arg8;                         \
      _argvec[2+9] = (unsigned long)arg9;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "stw  2,-8(11)\n\t"  /* save tocptr */                   \
         "lwz  2,-4(11)\n\t"  /* use nraddr's tocptr */           \
         VG_EXPAND_FRAME_BY_trashes_r3(64)                        \
         /* arg9 */                                               \
         "lwz 3,36(11)\n\t"                                       \
         "stw 3,56(1)\n\t"                                        \
         /* args1-8 */                                            \
         "lwz  3, 4(11)\n\t"  /* arg1->r3 */                      \
         "lwz  4, 8(11)\n\t"  /* arg2->r4 */                      \
         "lwz  5, 12(11)\n\t" /* arg3->r5 */                      \
         "lwz  6, 16(11)\n\t" /* arg4->r6 */                      \
         "lwz  7, 20(11)\n\t" /* arg5->r7 */                      \
         "lwz  8, 24(11)\n\t" /* arg6->r8 */                      \
         "lwz  9, 28(11)\n\t" /* arg7->r9 */                      \
         "lwz 10, 32(11)\n\t" /* arg8->r10 */                     \
         "lwz 11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "lwz 2,-8(11)\n\t" /* restore tocptr */                  \
         VG_CONTRACT_FRAME_BY(64)                                 \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_10W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,  \
                                  arg7,arg8,arg9,arg10)           \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+10];                       \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      _argvec[2+8] = (unsigned long)arg8;                         \
      _argvec[2+9] = (unsigned long)arg9;                         \
      _argvec[2+10] = (unsigned long)arg10;                       \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "stw  2,-8(11)\n\t"  /* save tocptr */                   \
         "lwz  2,-4(11)\n\t"  /* use nraddr's tocptr */           \
         VG_EXPAND_FRAME_BY_trashes_r3(64)                        \
         /* arg10 */                                              \
         "lwz 3,40(11)\n\t"                                       \
         "stw 3,60(1)\n\t"                                        \
         /* arg9 */                                               \
         "lwz 3,36(11)\n\t"                                       \
         "stw 3,56(1)\n\t"                                        \
         /* args1-8 */                                            \
         "lwz  3, 4(11)\n\t"  /* arg1->r3 */                      \
         "lwz  4, 8(11)\n\t"  /* arg2->r4 */                      \
         "lwz  5, 12(11)\n\t" /* arg3->r5 */                      \
         "lwz  6, 16(11)\n\t" /* arg4->r6 */                      \
         "lwz  7, 20(11)\n\t" /* arg5->r7 */                      \
         "lwz  8, 24(11)\n\t" /* arg6->r8 */                      \
         "lwz  9, 28(11)\n\t" /* arg7->r9 */                      \
         "lwz 10, 32(11)\n\t" /* arg8->r10 */                     \
         "lwz 11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "lwz 2,-8(11)\n\t" /* restore tocptr */                  \
         VG_CONTRACT_FRAME_BY(64)                                 \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_11W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,  \
                                  arg7,arg8,arg9,arg10,arg11)     \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+11];                       \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      _argvec[2+8] = (unsigned long)arg8;                         \
      _argvec[2+9] = (unsigned long)arg9;                         \
      _argvec[2+10] = (unsigned long)arg10;                       \
      _argvec[2+11] = (unsigned long)arg11;                       \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "stw  2,-8(11)\n\t"  /* save tocptr */                   \
         "lwz  2,-4(11)\n\t"  /* use nraddr's tocptr */           \
         VG_EXPAND_FRAME_BY_trashes_r3(72)                        \
         /* arg11 */                                              \
         "lwz 3,44(11)\n\t"                                       \
         "stw 3,64(1)\n\t"                                        \
         /* arg10 */                                              \
         "lwz 3,40(11)\n\t"                                       \
         "stw 3,60(1)\n\t"                                        \
         /* arg9 */                                               \
         "lwz 3,36(11)\n\t"                                       \
         "stw 3,56(1)\n\t"                                        \
         /* args1-8 */                                            \
         "lwz  3, 4(11)\n\t"  /* arg1->r3 */                      \
         "lwz  4, 8(11)\n\t"  /* arg2->r4 */                      \
         "lwz  5, 12(11)\n\t" /* arg3->r5 */                      \
         "lwz  6, 16(11)\n\t" /* arg4->r6 */                      \
         "lwz  7, 20(11)\n\t" /* arg5->r7 */                      \
         "lwz  8, 24(11)\n\t" /* arg6->r8 */                      \
         "lwz  9, 28(11)\n\t" /* arg7->r9 */                      \
         "lwz 10, 32(11)\n\t" /* arg8->r10 */                     \
         "lwz 11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "lwz 2,-8(11)\n\t" /* restore tocptr */                  \
         VG_CONTRACT_FRAME_BY(72)                                 \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_12W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,  \
                                arg7,arg8,arg9,arg10,arg11,arg12) \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+12];                       \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      _argvec[2+8] = (unsigned long)arg8;                         \
      _argvec[2+9] = (unsigned long)arg9;                         \
      _argvec[2+10] = (unsigned long)arg10;                       \
      _argvec[2+11] = (unsigned long)arg11;                       \
      _argvec[2+12] = (unsigned long)arg12;                       \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "stw  2,-8(11)\n\t"  /* save tocptr */                   \
         "lwz  2,-4(11)\n\t"  /* use nraddr's tocptr */           \
         VG_EXPAND_FRAME_BY_trashes_r3(72)                        \
         /* arg12 */                                              \
         "lwz 3,48(11)\n\t"                                       \
         "stw 3,68(1)\n\t"                                        \
         /* arg11 */                                              \
         "lwz 3,44(11)\n\t"                                       \
         "stw 3,64(1)\n\t"                                        \
         /* arg10 */                                              \
         "lwz 3,40(11)\n\t"                                       \
         "stw 3,60(1)\n\t"                                        \
         /* arg9 */                                               \
         "lwz 3,36(11)\n\t"                                       \
         "stw 3,56(1)\n\t"                                        \
         /* args1-8 */                                            \
         "lwz  3, 4(11)\n\t"  /* arg1->r3 */                      \
         "lwz  4, 8(11)\n\t"  /* arg2->r4 */                      \
         "lwz  5, 12(11)\n\t" /* arg3->r5 */                      \
         "lwz  6, 16(11)\n\t" /* arg4->r6 */                      \
         "lwz  7, 20(11)\n\t" /* arg5->r7 */                      \
         "lwz  8, 24(11)\n\t" /* arg6->r8 */                      \
         "lwz  9, 28(11)\n\t" /* arg7->r9 */                      \
         "lwz 10, 32(11)\n\t" /* arg8->r10 */                     \
         "lwz 11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "lwz 2,-8(11)\n\t" /* restore tocptr */                  \
         VG_CONTRACT_FRAME_BY(72)                                 \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#endif /* PLAT_ppc32_aix5 */

/* ------------------------ ppc64-aix5 ------------------------- */

#if defined(PLAT_ppc64_aix5)

/* ARGREGS: r3 r4 r5 r6 r7 r8 r9 r10 (the rest on stack somewhere) */

/* These regs are trashed by the hidden call. */
#define __CALLER_SAVED_REGS                                       \
   "lr", "ctr", "xer",                                            \
   "cr0", "cr1", "cr2", "cr3", "cr4", "cr5", "cr6", "cr7",        \
   "r0", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10",   \
   "r11", "r12", "r13"


#define VG_EXPAND_FRAME_BY_trashes_r3(_n_fr)                      \
         "addi 1,1,-" #_n_fr "\n\t"                               \
         "ld   3," #_n_fr "(1)\n\t"                               \
         "std  3,0(1)\n\t"

#define VG_CONTRACT_FRAME_BY(_n_fr)                               \
         "addi 1,1," #_n_fr "\n\t"


#define CALL_FN_W_v(lval, orig)                                   \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+0];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1] = (unsigned long)_orig.r2;                       \
      _argvec[2] = (unsigned long)_orig.nraddr;                   \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "std  2,-16(11)\n\t" /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld 2,-16(11)\n\t" /* restore tocptr */                  \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_W(lval, orig, arg1)                             \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+1];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "std  2,-16(11)\n\t" /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld 2,-16(11)\n\t" /* restore tocptr */                  \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WW(lval, orig, arg1,arg2)                       \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+2];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "std  2,-16(11)\n\t" /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld  2,-16(11)\n\t" /* restore tocptr */                 \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WWW(lval, orig, arg1,arg2,arg3)                 \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+3];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "std  2,-16(11)\n\t" /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld  2,-16(11)\n\t" /* restore tocptr */                 \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_WWWW(lval, orig, arg1,arg2,arg3,arg4)           \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+4];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "std  2,-16(11)\n\t" /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld  2,-16(11)\n\t" /* restore tocptr */                 \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_5W(lval, orig, arg1,arg2,arg3,arg4,arg5)        \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+5];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "std  2,-16(11)\n\t" /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld   7, 40(11)\n\t" /* arg5->r7 */                      \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld  2,-16(11)\n\t" /* restore tocptr */                 \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_6W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6)   \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+6];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "std  2,-16(11)\n\t" /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld   7, 40(11)\n\t" /* arg5->r7 */                      \
         "ld   8, 48(11)\n\t" /* arg6->r8 */                      \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld  2,-16(11)\n\t" /* restore tocptr */                 \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_7W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7)                            \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+7];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "std  2,-16(11)\n\t" /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld   7, 40(11)\n\t" /* arg5->r7 */                      \
         "ld   8, 48(11)\n\t" /* arg6->r8 */                      \
         "ld   9, 56(11)\n\t" /* arg7->r9 */                      \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld  2,-16(11)\n\t" /* restore tocptr */                 \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_8W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7,arg8)                       \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+8];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      _argvec[2+8] = (unsigned long)arg8;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "std  2,-16(11)\n\t" /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld   7, 40(11)\n\t" /* arg5->r7 */                      \
         "ld   8, 48(11)\n\t" /* arg6->r8 */                      \
         "ld   9, 56(11)\n\t" /* arg7->r9 */                      \
         "ld  10, 64(11)\n\t" /* arg8->r10 */                     \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld  2,-16(11)\n\t" /* restore tocptr */                 \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_9W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,   \
                                 arg7,arg8,arg9)                  \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+9];                        \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      _argvec[2+8] = (unsigned long)arg8;                         \
      _argvec[2+9] = (unsigned long)arg9;                         \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "std  2,-16(11)\n\t" /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         VG_EXPAND_FRAME_BY_trashes_r3(128)                       \
         /* arg9 */                                               \
         "ld  3,72(11)\n\t"                                       \
         "std 3,112(1)\n\t"                                       \
         /* args1-8 */                                            \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld   7, 40(11)\n\t" /* arg5->r7 */                      \
         "ld   8, 48(11)\n\t" /* arg6->r8 */                      \
         "ld   9, 56(11)\n\t" /* arg7->r9 */                      \
         "ld  10, 64(11)\n\t" /* arg8->r10 */                     \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld  2,-16(11)\n\t" /* restore tocptr */                 \
         VG_CONTRACT_FRAME_BY(128)                                \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_10W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,  \
                                  arg7,arg8,arg9,arg10)           \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+10];                       \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      _argvec[2+8] = (unsigned long)arg8;                         \
      _argvec[2+9] = (unsigned long)arg9;                         \
      _argvec[2+10] = (unsigned long)arg10;                       \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "std  2,-16(11)\n\t" /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         VG_EXPAND_FRAME_BY_trashes_r3(128)                       \
         /* arg10 */                                              \
         "ld  3,80(11)\n\t"                                       \
         "std 3,120(1)\n\t"                                       \
         /* arg9 */                                               \
         "ld  3,72(11)\n\t"                                       \
         "std 3,112(1)\n\t"                                       \
         /* args1-8 */                                            \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld   7, 40(11)\n\t" /* arg5->r7 */                      \
         "ld   8, 48(11)\n\t" /* arg6->r8 */                      \
         "ld   9, 56(11)\n\t" /* arg7->r9 */                      \
         "ld  10, 64(11)\n\t" /* arg8->r10 */                     \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld  2,-16(11)\n\t" /* restore tocptr */                 \
         VG_CONTRACT_FRAME_BY(128)                                \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_11W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,  \
                                  arg7,arg8,arg9,arg10,arg11)     \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+11];                       \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      _argvec[2+8] = (unsigned long)arg8;                         \
      _argvec[2+9] = (unsigned long)arg9;                         \
      _argvec[2+10] = (unsigned long)arg10;                       \
      _argvec[2+11] = (unsigned long)arg11;                       \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "std  2,-16(11)\n\t" /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         VG_EXPAND_FRAME_BY_trashes_r3(144)                       \
         /* arg11 */                                              \
         "ld  3,88(11)\n\t"                                       \
         "std 3,128(1)\n\t"                                       \
         /* arg10 */                                              \
         "ld  3,80(11)\n\t"                                       \
         "std 3,120(1)\n\t"                                       \
         /* arg9 */                                               \
         "ld  3,72(11)\n\t"                                       \
         "std 3,112(1)\n\t"                                       \
         /* args1-8 */                                            \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld   7, 40(11)\n\t" /* arg5->r7 */                      \
         "ld   8, 48(11)\n\t" /* arg6->r8 */                      \
         "ld   9, 56(11)\n\t" /* arg7->r9 */                      \
         "ld  10, 64(11)\n\t" /* arg8->r10 */                     \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld  2,-16(11)\n\t" /* restore tocptr */                 \
         VG_CONTRACT_FRAME_BY(144)                                \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#define CALL_FN_W_12W(lval, orig, arg1,arg2,arg3,arg4,arg5,arg6,  \
                                arg7,arg8,arg9,arg10,arg11,arg12) \
   do {                                                           \
      volatile OrigFn        _orig = (orig);                      \
      volatile unsigned long _argvec[3+12];                       \
      volatile unsigned long _res;                                \
      /* _argvec[0] holds current r2 across the call */           \
      _argvec[1]   = (unsigned long)_orig.r2;                     \
      _argvec[2]   = (unsigned long)_orig.nraddr;                 \
      _argvec[2+1] = (unsigned long)arg1;                         \
      _argvec[2+2] = (unsigned long)arg2;                         \
      _argvec[2+3] = (unsigned long)arg3;                         \
      _argvec[2+4] = (unsigned long)arg4;                         \
      _argvec[2+5] = (unsigned long)arg5;                         \
      _argvec[2+6] = (unsigned long)arg6;                         \
      _argvec[2+7] = (unsigned long)arg7;                         \
      _argvec[2+8] = (unsigned long)arg8;                         \
      _argvec[2+9] = (unsigned long)arg9;                         \
      _argvec[2+10] = (unsigned long)arg10;                       \
      _argvec[2+11] = (unsigned long)arg11;                       \
      _argvec[2+12] = (unsigned long)arg12;                       \
      __asm__ volatile(                                           \
         "mr 11,%1\n\t"                                           \
         VG_EXPAND_FRAME_BY_trashes_r3(512)                       \
         "std  2,-16(11)\n\t" /* save tocptr */                   \
         "ld   2,-8(11)\n\t"  /* use nraddr's tocptr */           \
         VG_EXPAND_FRAME_BY_trashes_r3(144)                       \
         /* arg12 */                                              \
         "ld  3,96(11)\n\t"                                       \
         "std 3,136(1)\n\t"                                       \
         /* arg11 */                                              \
         "ld  3,88(11)\n\t"                                       \
         "std 3,128(1)\n\t"                                       \
         /* arg10 */                                              \
         "ld  3,80(11)\n\t"                                       \
         "std 3,120(1)\n\t"                                       \
         /* arg9 */                                               \
         "ld  3,72(11)\n\t"                                       \
         "std 3,112(1)\n\t"                                       \
         /* args1-8 */                                            \
         "ld   3, 8(11)\n\t"  /* arg1->r3 */                      \
         "ld   4, 16(11)\n\t" /* arg2->r4 */                      \
         "ld   5, 24(11)\n\t" /* arg3->r5 */                      \
         "ld   6, 32(11)\n\t" /* arg4->r6 */                      \
         "ld   7, 40(11)\n\t" /* arg5->r7 */                      \
         "ld   8, 48(11)\n\t" /* arg6->r8 */                      \
         "ld   9, 56(11)\n\t" /* arg7->r9 */                      \
         "ld  10, 64(11)\n\t" /* arg8->r10 */                     \
         "ld  11, 0(11)\n\t"  /* target->r11 */                   \
         VALGRIND_BRANCH_AND_LINK_TO_NOREDIR_R11                  \
         "mr 11,%1\n\t"                                           \
         "mr %0,3\n\t"                                            \
         "ld  2,-16(11)\n\t" /* restore tocptr */                 \
         VG_CONTRACT_FRAME_BY(144)                                \
         VG_CONTRACT_FRAME_BY(512)                                \
         : /*out*/   "=r" (_res)                                  \
         : /*in*/    "r" (&_argvec[2])                            \
         : /*trash*/ "cc", "memory", __CALLER_SAVED_REGS          \
      );                                                          \
      lval = (__typeof__(lval)) _res;                             \
   } while (0)

#endif /* PLAT_ppc64_aix5 */


/* ------------------------------------------------------------------ */
/* ARCHITECTURE INDEPENDENT MACROS for CLIENT REQUESTS.               */
/*                                                                    */
/* ------------------------------------------------------------------ */


#define VG_USERREQ_TOOL_BASE(a,b) \
   ((unsigned int)(((a)&0xff) << 24 | ((b)&0xff) << 16))
#define VG_IS_TOOL_USERREQ(a, b, v) \
   (VG_USERREQ_TOOL_BASE(a,b) == ((v) & 0xffff0000))

typedef
   enum { VG_USERREQ__RUNNING_ON_VALGRIND  = 0x1001,
          VG_USERREQ__DISCARD_TRANSLATIONS = 0x1002,

          /* These allow any function to be called from the simulated
             CPU but run on the real CPU.  Nb: the first arg passed to
             the function is always the ThreadId of the running
             thread!  So CLIENT_CALL0 actually requires a 1 arg
             function, etc. */
          VG_USERREQ__CLIENT_CALL0 = 0x1101,
          VG_USERREQ__CLIENT_CALL1 = 0x1102,
          VG_USERREQ__CLIENT_CALL2 = 0x1103,
          VG_USERREQ__CLIENT_CALL3 = 0x1104,

          /* Can be useful in regression testing suites -- eg. can
             send Valgrind's output to /dev/null and still count
             errors. */
          VG_USERREQ__COUNT_ERRORS = 0x1201,

          /* These are useful and can be interpreted by any tool that
             tracks malloc() et al, by using vg_replace_malloc.c. */
          VG_USERREQ__MALLOCLIKE_BLOCK = 0x1301,
          VG_USERREQ__FREELIKE_BLOCK   = 0x1302,
          /* Memory pool support. */
          VG_USERREQ__CREATE_MEMPOOL   = 0x1303,
          VG_USERREQ__DESTROY_MEMPOOL  = 0x1304,
          VG_USERREQ__MEMPOOL_ALLOC    = 0x1305,
          VG_USERREQ__MEMPOOL_FREE     = 0x1306,
          VG_USERREQ__MEMPOOL_TRIM     = 0x1307,
          VG_USERREQ__MOVE_MEMPOOL     = 0x1308,
          VG_USERREQ__MEMPOOL_CHANGE   = 0x1309,
          VG_USERREQ__MEMPOOL_EXISTS   = 0x130a,

          /* Allow printfs to valgrind log. */
          VG_USERREQ__PRINTF           = 0x1401,
          VG_USERREQ__PRINTF_BACKTRACE = 0x1402,

          /* Stack support. */
          VG_USERREQ__STACK_REGISTER   = 0x1501,
          VG_USERREQ__STACK_DEREGISTER = 0x1502,
          VG_USERREQ__STACK_CHANGE     = 0x1503
   } Vg_ClientRequest;

#if !defined(__GNUC__)
#  define __extension__ /* */
#endif

#define RUNNING_ON_VALGRIND  __extension__                        \
   ({unsigned int _qzz_res;                                       \
    VALGRIND_DO_CLIENT_REQUEST(_qzz_res, 0 /* if not */,          \
                               VG_USERREQ__RUNNING_ON_VALGRIND,   \
                               0, 0, 0, 0, 0);                    \
    _qzz_res;                                                     \
   })


#define VALGRIND_DISCARD_TRANSLATIONS(_qzz_addr,_qzz_len)         \
   {unsigned int _qzz_res;                                        \
    VALGRIND_DO_CLIENT_REQUEST(_qzz_res, 0,                       \
                               VG_USERREQ__DISCARD_TRANSLATIONS,  \
                               _qzz_addr, _qzz_len, 0, 0, 0);     \
   }



#if defined(NVALGRIND)

#  define VALGRIND_PRINTF(...)
#  define VALGRIND_PRINTF_BACKTRACE(...)

#else /* NVALGRIND */

static int VALGRIND_PRINTF(const char *format, ...)
   __attribute__((format(__printf__, 1, 2), __unused__));
static int
VALGRIND_PRINTF(const char *format, ...)
{
   unsigned long _qzz_res;
   va_list vargs;
   va_start(vargs, format);
   VALGRIND_DO_CLIENT_REQUEST(_qzz_res, 0, VG_USERREQ__PRINTF,
                              (unsigned long)format, (unsigned long)vargs, 
                              0, 0, 0);
   va_end(vargs);
   return (int)_qzz_res;
}

static int VALGRIND_PRINTF_BACKTRACE(const char *format, ...)
   __attribute__((format(__printf__, 1, 2), __unused__));
static int
VALGRIND_PRINTF_BACKTRACE(const char *format, ...)
{
   unsigned long _qzz_res;
   va_list vargs;
   va_start(vargs, format);
   VALGRIND_DO_CLIENT_REQUEST(_qzz_res, 0, VG_USERREQ__PRINTF_BACKTRACE,
                              (unsigned long)format, (unsigned long)vargs, 
                              0, 0, 0);
   va_end(vargs);
   return (int)_qzz_res;
}

#endif /* NVALGRIND */


#define VALGRIND_NON_SIMD_CALL0(_qyy_fn)                          \
   __extension__                                                  \
   ({unsigned long _qyy_res;                                      \
    VALGRIND_DO_CLIENT_REQUEST(_qyy_res, 0 /* default return */,  \
                               VG_USERREQ__CLIENT_CALL0,          \
                               _qyy_fn,                           \
                               0, 0, 0, 0);                       \
    _qyy_res;                                                     \
   })

#define VALGRIND_NON_SIMD_CALL1(_qyy_fn, _qyy_arg1)               \
   __extension__                                                  \
   ({unsigned long _qyy_res;                                      \
    VALGRIND_DO_CLIENT_REQUEST(_qyy_res, 0 /* default return */,  \
                               VG_USERREQ__CLIENT_CALL1,          \
                               _qyy_fn,                           \
                               _qyy_arg1, 0, 0, 0);               \
    _qyy_res;                                                     \
   })

#define VALGRIND_NON_SIMD_CALL2(_qyy_fn, _qyy_arg1, _qyy_arg2)    \
   __extension__                                                  \
   ({unsigned long _qyy_res;                                      \
    VALGRIND_DO_CLIENT_REQUEST(_qyy_res, 0 /* default return */,  \
                               VG_USERREQ__CLIENT_CALL2,          \
                               _qyy_fn,                           \
                               _qyy_arg1, _qyy_arg2, 0, 0);       \
    _qyy_res;                                                     \
   })

#define VALGRIND_NON_SIMD_CALL3(_qyy_fn, _qyy_arg1, _qyy_arg2, _qyy_arg3) \
   __extension__                                                  \
   ({unsigned long _qyy_res;                                      \
    VALGRIND_DO_CLIENT_REQUEST(_qyy_res, 0 /* default return */,  \
                               VG_USERREQ__CLIENT_CALL3,          \
                               _qyy_fn,                           \
                               _qyy_arg1, _qyy_arg2,              \
                               _qyy_arg3, 0);                     \
    _qyy_res;                                                     \
   })


#define VALGRIND_COUNT_ERRORS                                     \
   __extension__                                                  \
   ({unsigned int _qyy_res;                                       \
    VALGRIND_DO_CLIENT_REQUEST(_qyy_res, 0 /* default return */,  \
                               VG_USERREQ__COUNT_ERRORS,          \
                               0, 0, 0, 0, 0);                    \
    _qyy_res;                                                     \
   })

#define VALGRIND_MALLOCLIKE_BLOCK(addr, sizeB, rzB, is_zeroed)    \
   {unsigned int _qzz_res;                                        \
    VALGRIND_DO_CLIENT_REQUEST(_qzz_res, 0,                       \
                               VG_USERREQ__MALLOCLIKE_BLOCK,      \
                               addr, sizeB, rzB, is_zeroed, 0);   \
   }

#define VALGRIND_FREELIKE_BLOCK(addr, rzB)                        \
   {unsigned int _qzz_res;                                        \
    VALGRIND_DO_CLIENT_REQUEST(_qzz_res, 0,                       \
                               VG_USERREQ__FREELIKE_BLOCK,        \
                               addr, rzB, 0, 0, 0);               \
   }

/* Create a memory pool. */
#define VALGRIND_CREATE_MEMPOOL(pool, rzB, is_zeroed)             \
   {unsigned int _qzz_res;                                        \
    VALGRIND_DO_CLIENT_REQUEST(_qzz_res, 0,                       \
                               VG_USERREQ__CREATE_MEMPOOL,        \
                               pool, rzB, is_zeroed, 0, 0);       \
   }

/* Destroy a memory pool. */
#define VALGRIND_DESTROY_MEMPOOL(pool)                            \
   {unsigned int _qzz_res;                                        \
    VALGRIND_DO_CLIENT_REQUEST(_qzz_res, 0,                       \
                               VG_USERREQ__DESTROY_MEMPOOL,       \
                               pool, 0, 0, 0, 0);                 \
   }

/* Associate a piece of memory with a memory pool. */
#define VALGRIND_MEMPOOL_ALLOC(pool, addr, size)                  \
   {unsigned int _qzz_res;                                        \
    VALGRIND_DO_CLIENT_REQUEST(_qzz_res, 0,                       \
                               VG_USERREQ__MEMPOOL_ALLOC,         \
                               pool, addr, size, 0, 0);           \
   }

/* Disassociate a piece of memory from a memory pool. */
#define VALGRIND_MEMPOOL_FREE(pool, addr)                         \
   {unsigned int _qzz_res;                                        \
    VALGRIND_DO_CLIENT_REQUEST(_qzz_res, 0,                       \
                               VG_USERREQ__MEMPOOL_FREE,          \
                               pool, addr, 0, 0, 0);              \
   }

/* Disassociate any pieces outside a particular range. */
#define VALGRIND_MEMPOOL_TRIM(pool, addr, size)                   \
   {unsigned int _qzz_res;                                        \
    VALGRIND_DO_CLIENT_REQUEST(_qzz_res, 0,                       \
                               VG_USERREQ__MEMPOOL_TRIM,          \
                               pool, addr, size, 0, 0);           \
   }

/* Resize and/or move a piece associated with a memory pool. */
#define VALGRIND_MOVE_MEMPOOL(poolA, poolB)                       \
   {unsigned int _qzz_res;                                        \
    VALGRIND_DO_CLIENT_REQUEST(_qzz_res, 0,                       \
                               VG_USERREQ__MOVE_MEMPOOL,          \
                               poolA, poolB, 0, 0, 0);            \
   }

/* Resize and/or move a piece associated with a memory pool. */
#define VALGRIND_MEMPOOL_CHANGE(pool, addrA, addrB, size)         \
   {unsigned int _qzz_res;                                        \
    VALGRIND_DO_CLIENT_REQUEST(_qzz_res, 0,                       \
                               VG_USERREQ__MEMPOOL_CHANGE,        \
                               pool, addrA, addrB, size, 0);      \
   }

/* Return 1 if a mempool exists, else 0. */
#define VALGRIND_MEMPOOL_EXISTS(pool)                             \
   ({unsigned int _qzz_res;                                       \
    VALGRIND_DO_CLIENT_REQUEST(_qzz_res, 0,                       \
                               VG_USERREQ__MEMPOOL_EXISTS,        \
                               pool, 0, 0, 0, 0);                 \
    _qzz_res;                                                     \
   })

/* Mark a piece of memory as being a stack. Returns a stack id. */
#define VALGRIND_STACK_REGISTER(start, end)                       \
   ({unsigned int _qzz_res;                                       \
    VALGRIND_DO_CLIENT_REQUEST(_qzz_res, 0,                       \
                               VG_USERREQ__STACK_REGISTER,        \
                               start, end, 0, 0, 0);              \
    _qzz_res;                                                     \
   })

#define VALGRIND_STACK_DEREGISTER(id)                             \
   {unsigned int _qzz_res;                                        \
    VALGRIND_DO_CLIENT_REQUEST(_qzz_res, 0,                       \
                               VG_USERREQ__STACK_DEREGISTER,      \
                               id, 0, 0, 0, 0);                   \
   }

/* Change the start and end address of the stack id. */
#define VALGRIND_STACK_CHANGE(id, start, end)                     \
   {unsigned int _qzz_res;                                        \
    VALGRIND_DO_CLIENT_REQUEST(_qzz_res, 0,                       \
                               VG_USERREQ__STACK_CHANGE,          \
                               id, start, end, 0, 0);             \
   }


#undef PLAT_x86_linux
#undef PLAT_amd64_linux
#undef PLAT_ppc32_linux
#undef PLAT_ppc64_linux
#undef PLAT_ppc32_aix5
#undef PLAT_ppc64_aix5

#endif   /* __VALGRIND_H */
