
#ifndef LIBFFI_TARGET_H
#define LIBFFI_TARGET_H

/* ---- System specific configurations ----------------------------------- */

#if defined (POWERPC) && defined (__powerpc64__)
#define POWERPC64
#endif

#ifndef LIBFFI_ASM
typedef unsigned long          ffi_arg;
typedef signed long            ffi_sarg;

typedef enum ffi_abi {
  FFI_FIRST_ABI = 0,

#ifdef POWERPC
  FFI_SYSV,
  FFI_GCC_SYSV,
  FFI_LINUX64,
  FFI_LINUX,
  FFI_LINUX_SOFT_FLOAT,
# ifdef POWERPC64
  FFI_DEFAULT_ABI = FFI_LINUX64,
# else
#  if (!defined(__NO_FPRS__) && (__LDBL_MANT_DIG__ == 106))
  FFI_DEFAULT_ABI = FFI_LINUX,
#  else
#   ifdef __NO_FPRS__
  FFI_DEFAULT_ABI = FFI_LINUX_SOFT_FLOAT,
#   else
  FFI_DEFAULT_ABI = FFI_GCC_SYSV,
#   endif
#  endif
# endif
#endif

#ifdef POWERPC_AIX
  FFI_AIX,
  FFI_DARWIN,
  FFI_DEFAULT_ABI = FFI_AIX,
#endif

#ifdef POWERPC_DARWIN
  FFI_AIX,
  FFI_DARWIN,
  FFI_DEFAULT_ABI = FFI_DARWIN,
#endif

#ifdef POWERPC_FREEBSD
  FFI_SYSV,
  FFI_GCC_SYSV,
  FFI_LINUX64,
  FFI_DEFAULT_ABI = FFI_SYSV,
#endif

  FFI_LAST_ABI
} ffi_abi;
#endif

/* ---- Definitions for closures ----------------------------------------- */

#define FFI_CLOSURES 1
#define FFI_NATIVE_RAW_API 0


/* Needed for soft-float long-double-128 support.  */
#define FFI_TYPE_UINT128 (FFI_TYPE_LAST + 1)

/* Needed for FFI_SYSV small structure returns.  */
#define FFI_SYSV_TYPE_SMALL_STRUCT (FFI_TYPE_LAST + 2)

#if defined(POWERPC64) || defined(POWERPC_AIX)
#define FFI_TRAMPOLINE_SIZE 24
#else /* POWERPC || POWERPC_AIX */
#define FFI_TRAMPOLINE_SIZE 40
#endif

#ifndef LIBFFI_ASM
#if defined(POWERPC_DARWIN) || defined(POWERPC_AIX)
struct ffi_aix_trampoline_struct {
    void * code_pointer;	/* Pointer to ffi_closure_ASM */
    void * toc;			/* TOC */
    void * static_chain;	/* Pointer to closure */
};
#endif
#endif

#endif
