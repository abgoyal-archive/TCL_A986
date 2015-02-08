
#ifndef LIBFFI_TARGET_H
#define LIBFFI_TARGET_H

#ifndef LIBFFI_ASM
typedef unsigned long long          ffi_arg;
typedef signed long long            ffi_sarg;

typedef enum ffi_abi {
  FFI_FIRST_ABI = 0,
  FFI_UNIX,   	/* Linux and all Unix variants use the same conventions	*/
  FFI_DEFAULT_ABI = FFI_UNIX,
  FFI_LAST_ABI = FFI_DEFAULT_ABI + 1
} ffi_abi;
#endif

/* ---- Definitions for closures ----------------------------------------- */

#define FFI_CLOSURES 1
#define FFI_TRAMPOLINE_SIZE 24  /* Really the following struct, which 	*/
				/* can be interpreted as a C function	*/
				/* descriptor:				*/

#endif

