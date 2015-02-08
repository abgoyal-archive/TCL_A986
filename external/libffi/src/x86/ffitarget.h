
#ifndef LIBFFI_TARGET_H
#define LIBFFI_TARGET_H

/* ---- System specific configurations ----------------------------------- */

#if defined (X86_64) && defined (__i386__)
#undef X86_64
#define X86
#endif

/* ---- Generic type definitions ----------------------------------------- */

#ifndef LIBFFI_ASM
typedef unsigned long          ffi_arg;
typedef signed long            ffi_sarg;

typedef enum ffi_abi {
  FFI_FIRST_ABI = 0,

  /* ---- Intel x86 Win32 ---------- */
#ifdef X86_WIN32
  FFI_SYSV,
  FFI_STDCALL,
  /* TODO: Add fastcall support for the sake of completeness */
  FFI_DEFAULT_ABI = FFI_SYSV,
#endif

  /* ---- Intel x86 and AMD x86-64 - */
#if !defined(X86_WIN32) && (defined(__i386__) || defined(__x86_64__))
  FFI_SYSV,
  FFI_UNIX64,   /* Unix variants all use the same ABI for x86-64  */
#ifdef __i386__
  FFI_DEFAULT_ABI = FFI_SYSV,
#else
  FFI_DEFAULT_ABI = FFI_UNIX64,
#endif
#endif

  FFI_LAST_ABI = FFI_DEFAULT_ABI + 1
} ffi_abi;
#endif

/* ---- Definitions for closures ----------------------------------------- */

#define FFI_CLOSURES 1
#define FFI_TYPE_SMALL_STRUCT_1B (FFI_TYPE_LAST + 1)
#define FFI_TYPE_SMALL_STRUCT_2B (FFI_TYPE_LAST + 2)

#if defined (X86_64) || (defined (__x86_64__) && defined (X86_DARWIN))
#define FFI_TRAMPOLINE_SIZE 24
#define FFI_NATIVE_RAW_API 0
#else
#ifdef X86_WIN32
#define FFI_TRAMPOLINE_SIZE 13
#else
#define FFI_TRAMPOLINE_SIZE 10
#endif
#define FFI_NATIVE_RAW_API 1	/* x86 has native raw api support */
#endif

#endif

