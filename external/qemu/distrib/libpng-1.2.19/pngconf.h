



#ifndef PNGCONF_H
#define PNGCONF_H

#define PNG_1_2_X

#ifdef PNG_USER_CONFIG
#  ifndef PNG_USER_PRIVATEBUILD
#    define PNG_USER_PRIVATEBUILD
#  endif
#include "pngusr.h"
#endif

/* PNG_CONFIGURE_LIBPNG is set by the "configure" script. */
#ifdef PNG_CONFIGURE_LIBPNG
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#endif


#ifdef __STDC__
#ifdef SPECIALBUILD
#  pragma message("PNG_LIBPNG_SPECIALBUILD (and deprecated SPECIALBUILD)\
 are now LIBPNG reserved macros. Use PNG_USER_PRIVATEBUILD instead.")
#endif

#ifdef PRIVATEBUILD
# pragma message("PRIVATEBUILD is deprecated.\
 Use PNG_USER_PRIVATEBUILD instead.")
# define PNG_USER_PRIVATEBUILD PRIVATEBUILD
#endif
#endif /* __STDC__ */

#ifndef PNG_VERSION_INFO_ONLY

/* End of material added to libpng-1.2.8 */

/* Added at libpng-1.2.19 */
#ifndef PNG_NO_WARN_UNINITIALIZED_ROW
#  ifndef PNG_WARN_UNINITIALIZED_ROW
#    define PNG_WARN_UNINITIALIZED_ROW 1 /* 0: warning; 1: error */
#  endif
#endif
/* End of material added at libpng-1.2.19 */


#ifndef PNG_ZBUF_SIZE
#  define PNG_ZBUF_SIZE 8192
#endif

/* Enable if you want a write-only libpng */

#ifndef PNG_NO_READ_SUPPORTED
#  define PNG_READ_SUPPORTED
#endif

/* Enable if you want a read-only libpng */

#ifndef PNG_NO_WRITE_SUPPORTED
#  define PNG_WRITE_SUPPORTED
#endif

#if !defined(PNG_1_0_X) && !defined(PNG_NO_MNG_FEATURES)
#  ifndef PNG_MNG_FEATURES_SUPPORTED
#    define PNG_MNG_FEATURES_SUPPORTED
#  endif
#endif

#ifndef PNG_NO_FLOATING_POINT_SUPPORTED
#  ifndef PNG_FLOATING_POINT_SUPPORTED
#    define PNG_FLOATING_POINT_SUPPORTED
#  endif
#endif

#if defined(MAXSEG_64K) && !defined(PNG_MAX_MALLOC_64K)
#  define PNG_MAX_MALLOC_64K
#endif

#if defined(__CYGWIN__)
#  if defined(ALL_STATIC)
#    if defined(PNG_BUILD_DLL)
#      undef PNG_BUILD_DLL
#    endif
#    if defined(PNG_USE_DLL)
#      undef PNG_USE_DLL
#    endif
#    if defined(PNG_DLL)
#      undef PNG_DLL
#    endif
#    if !defined(PNG_STATIC)
#      define PNG_STATIC
#    endif
#  else
#    if defined (PNG_BUILD_DLL)
#      if defined(PNG_STATIC)
#        undef PNG_STATIC
#      endif
#      if defined(PNG_USE_DLL)
#        undef PNG_USE_DLL
#      endif
#      if !defined(PNG_DLL)
#        define PNG_DLL
#      endif
#    else
#      if defined(PNG_STATIC)
#        if defined(PNG_USE_DLL)
#          undef PNG_USE_DLL
#        endif
#        if defined(PNG_DLL)
#          undef PNG_DLL
#        endif
#      else
#        if !defined(PNG_USE_DLL)
#          define PNG_USE_DLL
#        endif
#        if !defined(PNG_DLL)
#          define PNG_DLL
#        endif
#      endif  
#    endif  
#  endif
#endif


#if defined(_WIN32_WCE)
#  include <windows.h>
   /* Console I/O functions are not supported on WindowsCE */
#  define PNG_NO_CONSOLE_IO
#  ifdef PNG_DEBUG
#    undef PNG_DEBUG
#  endif
#endif

#ifdef PNG_BUILD_DLL
#  ifndef PNG_CONSOLE_IO_SUPPORTED
#    ifndef PNG_NO_CONSOLE_IO
#      define PNG_NO_CONSOLE_IO
#    endif
#  endif
#endif

#  ifdef PNG_NO_STDIO
#    ifndef PNG_NO_CONSOLE_IO
#      define PNG_NO_CONSOLE_IO
#    endif
#    ifdef PNG_DEBUG
#      if (PNG_DEBUG > 0)
#        include <stdio.h>
#      endif
#    endif
#  else
#    if !defined(_WIN32_WCE)
/* "stdio.h" functions are not supported on WindowsCE */
#      include <stdio.h>
#    endif
#  endif

#ifndef PNGARG

#ifdef OF /* zlib prototype munger */
#  define PNGARG(arglist) OF(arglist)
#else

#ifdef _NO_PROTO
#  define PNGARG(arglist) ()
#  ifndef PNG_TYPECAST_NULL
#     define PNG_TYPECAST_NULL
#  endif
#else
#  define PNGARG(arglist) arglist
#endif /* _NO_PROTO */


#endif /* OF */

#endif /* PNGARG */

#ifndef MACOS
#  if (defined(__MWERKS__) && defined(macintosh)) || defined(applec) || \
      defined(THINK_C) || defined(__SC__) || defined(TARGET_OS_MAC)
#    define MACOS
#  endif
#endif

/* enough people need this for various reasons to include it here */
#if !defined(MACOS) && !defined(RISCOS) && !defined(_WIN32_WCE)
#  include <sys/types.h>
#endif

#if !defined(PNG_SETJMP_NOT_SUPPORTED) && !defined(PNG_NO_SETJMP_SUPPORTED)
#  define PNG_SETJMP_SUPPORTED
#endif

#ifdef PNG_SETJMP_SUPPORTED

#  ifdef __linux__
#    ifdef _BSD_SOURCE
#      define PNG_SAVE_BSD_SOURCE
#      undef _BSD_SOURCE
#    endif
#    ifdef _SETJMP_H
     /* If you encounter a compiler error here, see the explanation
      * near the end of INSTALL.
      */
         __png.h__ already includes setjmp.h;
         __dont__ include it again.;
#    endif
#  endif /* __linux__ */

   /* include setjmp.h for error handling */
#  include <setjmp.h>

#  ifdef __linux__
#    ifdef PNG_SAVE_BSD_SOURCE
#      define _BSD_SOURCE
#      undef PNG_SAVE_BSD_SOURCE
#    endif
#  endif /* __linux__ */
#endif /* PNG_SETJMP_SUPPORTED */

#ifdef BSD
#  include <strings.h>
#else
#  include <string.h>
#endif

/* Other defines for things like memory and the like can go here.  */
#ifdef PNG_INTERNAL

#include <stdlib.h>

#define PNG_EXTERN


#if defined(PNG_FLOATING_POINT_SUPPORTED)
#  if defined(MACOS)
     /* We need to check that <math.h> hasn't already been included earlier
      * as it seems it doesn't agree with <fp.h>, yet we should really use
      * <fp.h> if possible.
      */
#    if !defined(__MATH_H__) && !defined(__MATH_H) && !defined(__cmath__)
#      include <fp.h>
#    endif
#  else
#    include <math.h>
#  endif
#  if defined(_AMIGA) && defined(__SASC) && defined(_M68881)
     /* Amiga SAS/C: We must include builtin FPU functions when compiling using
      * MATH=68881
      */
#    include <m68881.h>
#  endif
#endif

/* Codewarrior on NT has linking problems without this. */
#if (defined(__MWERKS__) && defined(WIN32)) || defined(__STDC__)
#  define PNG_ALWAYS_EXTERN
#endif

/* This provides the non-ANSI (far) memory allocation routines. */
#if defined(__TURBOC__) && defined(__MSDOS__)
#  include <mem.h>
#  include <alloc.h>
#endif

/* I have no idea why is this necessary... */
#if defined(_MSC_VER) && (defined(WIN32) || defined(_Windows) || \
    defined(_WINDOWS) || defined(_WIN32) || defined(__WIN32__))
#  include <malloc.h>
#endif

#ifndef PNG_DITHER_RED_BITS
#  define PNG_DITHER_RED_BITS 5
#endif
#ifndef PNG_DITHER_GREEN_BITS
#  define PNG_DITHER_GREEN_BITS 5
#endif
#ifndef PNG_DITHER_BLUE_BITS
#  define PNG_DITHER_BLUE_BITS 5
#endif


#ifndef PNG_MAX_GAMMA_8
#  define PNG_MAX_GAMMA_8 11
#endif

#ifndef PNG_GAMMA_THRESHOLD
#  define PNG_GAMMA_THRESHOLD 0.05
#endif

#endif /* PNG_INTERNAL */


#ifndef PNG_NO_CONST
#  define PNG_CONST const
#else
#  define PNG_CONST
#endif


/* Any features you will not be using can be undef'ed here */



#if defined(PNG_1_0_X) || defined (PNG_1_2_X)
#  ifndef PNG_NO_iTXt_SUPPORTED
#    define PNG_NO_iTXt_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_iTXt
#    define PNG_NO_READ_iTXt
#  endif
#  ifndef PNG_NO_WRITE_iTXt
#    define PNG_NO_WRITE_iTXt
#  endif
#endif

#if !defined(PNG_NO_iTXt_SUPPORTED)
#  if !defined(PNG_READ_iTXt_SUPPORTED) && !defined(PNG_NO_READ_iTXt)
#    define PNG_READ_iTXt
#  endif
#  if !defined(PNG_WRITE_iTXt_SUPPORTED) && !defined(PNG_NO_WRITE_iTXt)
#    define PNG_WRITE_iTXt
#  endif
#endif


#ifdef PNG_LEGACY_SUPPORTED
#  define PNG_NO_FREE_ME
#  define PNG_NO_READ_UNKNOWN_CHUNKS
#  define PNG_NO_WRITE_UNKNOWN_CHUNKS
#  define PNG_NO_READ_USER_CHUNKS
#  define PNG_NO_READ_iCCP
#  define PNG_NO_WRITE_iCCP
#  define PNG_NO_READ_iTXt
#  define PNG_NO_WRITE_iTXt
#  define PNG_NO_READ_sCAL
#  define PNG_NO_WRITE_sCAL
#  define PNG_NO_READ_sPLT
#  define PNG_NO_WRITE_sPLT
#  define PNG_NO_INFO_IMAGE
#  define PNG_NO_READ_RGB_TO_GRAY
#  define PNG_NO_READ_USER_TRANSFORM
#  define PNG_NO_WRITE_USER_TRANSFORM
#  define PNG_NO_USER_MEM
#  define PNG_NO_READ_EMPTY_PLTE
#  define PNG_NO_MNG_FEATURES
#  define PNG_NO_FIXED_POINT_SUPPORTED
#endif

/* Ignore attempt to turn off both floating and fixed point support */
#if !defined(PNG_FLOATING_POINT_SUPPORTED) || \
    !defined(PNG_NO_FIXED_POINT_SUPPORTED)
#  define PNG_FIXED_POINT_SUPPORTED
#endif

#ifndef PNG_NO_FREE_ME
#  define PNG_FREE_ME_SUPPORTED
#endif

#if defined(PNG_READ_SUPPORTED)

#if !defined(PNG_READ_TRANSFORMS_NOT_SUPPORTED) && \
      !defined(PNG_NO_READ_TRANSFORMS)
#  define PNG_READ_TRANSFORMS_SUPPORTED
#endif

#ifdef PNG_READ_TRANSFORMS_SUPPORTED
#  ifndef PNG_NO_READ_EXPAND
#    define PNG_READ_EXPAND_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_SHIFT
#    define PNG_READ_SHIFT_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_PACK
#    define PNG_READ_PACK_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_BGR
#    define PNG_READ_BGR_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_SWAP
#    define PNG_READ_SWAP_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_PACKSWAP
#    define PNG_READ_PACKSWAP_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_INVERT
#    define PNG_READ_INVERT_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_DITHER
#    define PNG_READ_DITHER_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_BACKGROUND
#    define PNG_READ_BACKGROUND_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_16_TO_8
#    define PNG_READ_16_TO_8_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_FILLER
#    define PNG_READ_FILLER_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_GAMMA
#    define PNG_READ_GAMMA_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_GRAY_TO_RGB
#    define PNG_READ_GRAY_TO_RGB_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_SWAP_ALPHA
#    define PNG_READ_SWAP_ALPHA_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_INVERT_ALPHA
#    define PNG_READ_INVERT_ALPHA_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_STRIP_ALPHA
#    define PNG_READ_STRIP_ALPHA_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_USER_TRANSFORM
#    define PNG_READ_USER_TRANSFORM_SUPPORTED
#  endif
#  ifndef PNG_NO_READ_RGB_TO_GRAY
#    define PNG_READ_RGB_TO_GRAY_SUPPORTED
#  endif
#endif /* PNG_READ_TRANSFORMS_SUPPORTED */

#if !defined(PNG_NO_PROGRESSIVE_READ) && \
 !defined(PNG_PROGRESSIVE_READ_NOT_SUPPORTED) /* if you don't do progressive */
#  define PNG_PROGRESSIVE_READ_SUPPORTED    /* reading.  This is not talking */
#endif                              /* about interlacing capability!  You'll */
             /* still have interlacing unless you change the following line: */

#define PNG_READ_INTERLACING_SUPPORTED /* required in PNG-compliant decoders */

#ifndef PNG_NO_READ_COMPOSITE_NODIV
#  ifndef PNG_NO_READ_COMPOSITED_NODIV  /* libpng-1.0.x misspelling */
#    define PNG_READ_COMPOSITE_NODIV_SUPPORTED  /* well tested on Intel, SGI */
#  endif
#endif

#if defined(PNG_1_0_X) || defined (PNG_1_2_X)
#ifndef PNG_NO_READ_EMPTY_PLTE
#  define PNG_READ_EMPTY_PLTE_SUPPORTED
#endif
#endif

#endif /* PNG_READ_SUPPORTED */

#if defined(PNG_WRITE_SUPPORTED)

# if !defined(PNG_WRITE_TRANSFORMS_NOT_SUPPORTED) && \
    !defined(PNG_NO_WRITE_TRANSFORMS)
#  define PNG_WRITE_TRANSFORMS_SUPPORTED
#endif

#ifdef PNG_WRITE_TRANSFORMS_SUPPORTED
#  ifndef PNG_NO_WRITE_SHIFT
#    define PNG_WRITE_SHIFT_SUPPORTED
#  endif
#  ifndef PNG_NO_WRITE_PACK
#    define PNG_WRITE_PACK_SUPPORTED
#  endif
#  ifndef PNG_NO_WRITE_BGR
#    define PNG_WRITE_BGR_SUPPORTED
#  endif
#  ifndef PNG_NO_WRITE_SWAP
#    define PNG_WRITE_SWAP_SUPPORTED
#  endif
#  ifndef PNG_NO_WRITE_PACKSWAP
#    define PNG_WRITE_PACKSWAP_SUPPORTED
#  endif
#  ifndef PNG_NO_WRITE_INVERT
#    define PNG_WRITE_INVERT_SUPPORTED
#  endif
#  ifndef PNG_NO_WRITE_FILLER
#    define PNG_WRITE_FILLER_SUPPORTED   /* same as WRITE_STRIP_ALPHA */
#  endif
#  ifndef PNG_NO_WRITE_SWAP_ALPHA
#    define PNG_WRITE_SWAP_ALPHA_SUPPORTED
#  endif
#  ifndef PNG_NO_WRITE_INVERT_ALPHA
#    define PNG_WRITE_INVERT_ALPHA_SUPPORTED
#  endif
#  ifndef PNG_NO_WRITE_USER_TRANSFORM
#    define PNG_WRITE_USER_TRANSFORM_SUPPORTED
#  endif
#endif /* PNG_WRITE_TRANSFORMS_SUPPORTED */

#if !defined(PNG_NO_WRITE_INTERLACING_SUPPORTED) && \
    !defined(PNG_WRITE_INTERLACING_SUPPORTED)
#define PNG_WRITE_INTERLACING_SUPPORTED  /* not required for PNG-compliant
                                            encoders, but can cause trouble
                                            if left undefined */
#endif

#if !defined(PNG_NO_WRITE_WEIGHTED_FILTER) && \
    !defined(PNG_WRITE_WEIGHTED_FILTER) && \
     defined(PNG_FLOATING_POINT_SUPPORTED)
#  define PNG_WRITE_WEIGHTED_FILTER_SUPPORTED
#endif

#ifndef PNG_NO_WRITE_FLUSH
#  define PNG_WRITE_FLUSH_SUPPORTED
#endif

#if defined(PNG_1_0_X) || defined (PNG_1_2_X)
/* Deprecated, see PNG_MNG_FEATURES_SUPPORTED, above */
#ifndef PNG_NO_WRITE_EMPTY_PLTE
#  define PNG_WRITE_EMPTY_PLTE_SUPPORTED
#endif
#endif

#endif /* PNG_WRITE_SUPPORTED */

#ifndef PNG_1_0_X
#  ifndef PNG_NO_ERROR_NUMBERS
#    define PNG_ERROR_NUMBERS_SUPPORTED
#  endif
#endif /* PNG_1_0_X */

#if defined(PNG_READ_USER_TRANSFORM_SUPPORTED) || \
    defined(PNG_WRITE_USER_TRANSFORM_SUPPORTED)
#  ifndef PNG_NO_USER_TRANSFORM_PTR
#    define PNG_USER_TRANSFORM_PTR_SUPPORTED
#  endif
#endif

#ifndef PNG_NO_STDIO
#  define PNG_TIME_RFC1123_SUPPORTED
#endif

#if !defined(PNG_NO_EASY_ACCESS) && !defined(PNG_EASY_ACCESS_SUPPORTED)
#  define PNG_EASY_ACCESS_SUPPORTED
#endif


#if defined(PNG_READ_SUPPORTED) && !defined(PNG_NO_OPTIMIZED_CODE)
#  ifndef PNG_OPTIMIZED_CODE_SUPPORTED
#    define PNG_OPTIMIZED_CODE_SUPPORTED
#  endif
#endif

#if defined(PNG_READ_SUPPORTED) && !defined(PNG_NO_ASSEMBLER_CODE)
#  ifndef PNG_ASSEMBLER_CODE_SUPPORTED
#    define PNG_ASSEMBLER_CODE_SUPPORTED
#  endif

#  if !defined(PNG_MMX_CODE_SUPPORTED) && !defined(PNG_NO_MMX_CODE)
#    define PNG_MMX_CODE_SUPPORTED
#  endif

#  if !defined(PNG_USE_PNGVCRD) && defined(PNG_MMX_CODE_SUPPORTED) && \
     defined(_MSC_VER)
#    define PNG_USE_PNGVCRD
#  endif

#  if !defined(PNG_USE_PNGGCCRD) && defined(PNG_MMX_CODE_SUPPORTED) && \
     !defined(PNG_USE_PNGVCRD)
#    define PNG_USE_PNGGCCRD
     /* If you are sure that you don't need thread safety and you are compiling
        with PNG_USE_PNGCCRD for an MMX application, you can define this for
        faster execution.  See pnggccrd.c.
#    define PNG_THREAD_UNSAFE_OK
     */
#  endif

#endif

#if !defined(PNG_1_0_X)
#if !defined(PNG_NO_USER_MEM) && !defined(PNG_USER_MEM_SUPPORTED)
#  define PNG_USER_MEM_SUPPORTED
#endif
#endif /* PNG_1_0_X */

/* Added at libpng-1.2.6 */
#if !defined(PNG_1_0_X)
#ifndef PNG_SET_USER_LIMITS_SUPPORTED
#if !defined(PNG_NO_SET_USER_LIMITS) && !defined(PNG_SET_USER_LIMITS_SUPPORTED)
#  define PNG_SET_USER_LIMITS_SUPPORTED
#endif
#endif
#endif /* PNG_1_0_X */

#ifndef PNG_USER_WIDTH_MAX
#  define PNG_USER_WIDTH_MAX 1000000L
#endif
#ifndef PNG_USER_HEIGHT_MAX
#  define PNG_USER_HEIGHT_MAX 1000000L
#endif

/* These are currently experimental features, define them if you want */

/* very little testing */

/* This is only for PowerPC big-endian and 680x0 systems */
/* some testing */

/* Buggy compilers (e.g., gcc 2.7.2.2) need this */

/* These functions are turned off by default, as they will be phased out. */


#if defined(PNG_READ_SUPPORTED) && \
    !defined(PNG_READ_ANCILLARY_CHUNKS_NOT_SUPPORTED) && \
    !defined(PNG_NO_READ_ANCILLARY_CHUNKS)
#  define PNG_READ_ANCILLARY_CHUNKS_SUPPORTED
#endif

#if defined(PNG_WRITE_SUPPORTED) && \
    !defined(PNG_WRITE_ANCILLARY_CHUNKS_NOT_SUPPORTED) && \
    !defined(PNG_NO_WRITE_ANCILLARY_CHUNKS)
#  define PNG_WRITE_ANCILLARY_CHUNKS_SUPPORTED
#endif

#ifdef PNG_READ_ANCILLARY_CHUNKS_SUPPORTED

#ifdef PNG_NO_READ_TEXT
#  define PNG_NO_READ_iTXt
#  define PNG_NO_READ_tEXt
#  define PNG_NO_READ_zTXt
#endif
#ifndef PNG_NO_READ_bKGD
#  define PNG_READ_bKGD_SUPPORTED
#  define PNG_bKGD_SUPPORTED
#endif
#ifndef PNG_NO_READ_cHRM
#  define PNG_READ_cHRM_SUPPORTED
#  define PNG_cHRM_SUPPORTED
#endif
#ifndef PNG_NO_READ_gAMA
#  define PNG_READ_gAMA_SUPPORTED
#  define PNG_gAMA_SUPPORTED
#endif
#ifndef PNG_NO_READ_hIST
#  define PNG_READ_hIST_SUPPORTED
#  define PNG_hIST_SUPPORTED
#endif
#ifndef PNG_NO_READ_iCCP
#  define PNG_READ_iCCP_SUPPORTED
#  define PNG_iCCP_SUPPORTED
#endif
#ifndef PNG_NO_READ_iTXt
#  ifndef PNG_READ_iTXt_SUPPORTED
#    define PNG_READ_iTXt_SUPPORTED
#  endif
#  ifndef PNG_iTXt_SUPPORTED
#    define PNG_iTXt_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_READ_oFFs
#  define PNG_READ_oFFs_SUPPORTED
#  define PNG_oFFs_SUPPORTED
#endif
#ifndef PNG_NO_READ_pCAL
#  define PNG_READ_pCAL_SUPPORTED
#  define PNG_pCAL_SUPPORTED
#endif
#ifndef PNG_NO_READ_sCAL
#  define PNG_READ_sCAL_SUPPORTED
#  define PNG_sCAL_SUPPORTED
#endif
#ifndef PNG_NO_READ_pHYs
#  define PNG_READ_pHYs_SUPPORTED
#  define PNG_pHYs_SUPPORTED
#endif
#ifndef PNG_NO_READ_sBIT
#  define PNG_READ_sBIT_SUPPORTED
#  define PNG_sBIT_SUPPORTED
#endif
#ifndef PNG_NO_READ_sPLT
#  define PNG_READ_sPLT_SUPPORTED
#  define PNG_sPLT_SUPPORTED
#endif
#ifndef PNG_NO_READ_sRGB
#  define PNG_READ_sRGB_SUPPORTED
#  define PNG_sRGB_SUPPORTED
#endif
#ifndef PNG_NO_READ_tEXt
#  define PNG_READ_tEXt_SUPPORTED
#  define PNG_tEXt_SUPPORTED
#endif
#ifndef PNG_NO_READ_tIME
#  define PNG_READ_tIME_SUPPORTED
#  define PNG_tIME_SUPPORTED
#endif
#ifndef PNG_NO_READ_tRNS
#  define PNG_READ_tRNS_SUPPORTED
#  define PNG_tRNS_SUPPORTED
#endif
#ifndef PNG_NO_READ_zTXt
#  define PNG_READ_zTXt_SUPPORTED
#  define PNG_zTXt_SUPPORTED
#endif
#ifndef PNG_NO_READ_UNKNOWN_CHUNKS
#  define PNG_READ_UNKNOWN_CHUNKS_SUPPORTED
#  ifndef PNG_UNKNOWN_CHUNKS_SUPPORTED
#    define PNG_UNKNOWN_CHUNKS_SUPPORTED
#  endif
#  ifndef PNG_NO_HANDLE_AS_UNKNOWN
#    define PNG_HANDLE_AS_UNKNOWN_SUPPORTED
#  endif
#endif
#if !defined(PNG_NO_READ_USER_CHUNKS) && \
     defined(PNG_READ_UNKNOWN_CHUNKS_SUPPORTED)
#  define PNG_READ_USER_CHUNKS_SUPPORTED
#  define PNG_USER_CHUNKS_SUPPORTED
#  ifdef PNG_NO_READ_UNKNOWN_CHUNKS
#    undef PNG_NO_READ_UNKNOWN_CHUNKS
#  endif
#  ifdef PNG_NO_HANDLE_AS_UNKNOWN
#    undef PNG_NO_HANDLE_AS_UNKNOWN
#  endif
#endif
#ifndef PNG_NO_READ_OPT_PLTE
#  define PNG_READ_OPT_PLTE_SUPPORTED /* only affects support of the */
#endif                      /* optional PLTE chunk in RGB and RGBA images */
#if defined(PNG_READ_iTXt_SUPPORTED) || defined(PNG_READ_tEXt_SUPPORTED) || \
    defined(PNG_READ_zTXt_SUPPORTED)
#  define PNG_READ_TEXT_SUPPORTED
#  define PNG_TEXT_SUPPORTED
#endif

#endif /* PNG_READ_ANCILLARY_CHUNKS_SUPPORTED */

#ifdef PNG_WRITE_ANCILLARY_CHUNKS_SUPPORTED

#ifdef PNG_NO_WRITE_TEXT
#  define PNG_NO_WRITE_iTXt
#  define PNG_NO_WRITE_tEXt
#  define PNG_NO_WRITE_zTXt
#endif
#ifndef PNG_NO_WRITE_bKGD
#  define PNG_WRITE_bKGD_SUPPORTED
#  ifndef PNG_bKGD_SUPPORTED
#    define PNG_bKGD_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_WRITE_cHRM
#  define PNG_WRITE_cHRM_SUPPORTED
#  ifndef PNG_cHRM_SUPPORTED
#    define PNG_cHRM_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_WRITE_gAMA
#  define PNG_WRITE_gAMA_SUPPORTED
#  ifndef PNG_gAMA_SUPPORTED
#    define PNG_gAMA_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_WRITE_hIST
#  define PNG_WRITE_hIST_SUPPORTED
#  ifndef PNG_hIST_SUPPORTED
#    define PNG_hIST_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_WRITE_iCCP
#  define PNG_WRITE_iCCP_SUPPORTED
#  ifndef PNG_iCCP_SUPPORTED
#    define PNG_iCCP_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_WRITE_iTXt
#  ifndef PNG_WRITE_iTXt_SUPPORTED
#    define PNG_WRITE_iTXt_SUPPORTED
#  endif
#  ifndef PNG_iTXt_SUPPORTED
#    define PNG_iTXt_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_WRITE_oFFs
#  define PNG_WRITE_oFFs_SUPPORTED
#  ifndef PNG_oFFs_SUPPORTED
#    define PNG_oFFs_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_WRITE_pCAL
#  define PNG_WRITE_pCAL_SUPPORTED
#  ifndef PNG_pCAL_SUPPORTED
#    define PNG_pCAL_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_WRITE_sCAL
#  define PNG_WRITE_sCAL_SUPPORTED
#  ifndef PNG_sCAL_SUPPORTED
#    define PNG_sCAL_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_WRITE_pHYs
#  define PNG_WRITE_pHYs_SUPPORTED
#  ifndef PNG_pHYs_SUPPORTED
#    define PNG_pHYs_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_WRITE_sBIT
#  define PNG_WRITE_sBIT_SUPPORTED
#  ifndef PNG_sBIT_SUPPORTED
#    define PNG_sBIT_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_WRITE_sPLT
#  define PNG_WRITE_sPLT_SUPPORTED
#  ifndef PNG_sPLT_SUPPORTED
#    define PNG_sPLT_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_WRITE_sRGB
#  define PNG_WRITE_sRGB_SUPPORTED
#  ifndef PNG_sRGB_SUPPORTED
#    define PNG_sRGB_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_WRITE_tEXt
#  define PNG_WRITE_tEXt_SUPPORTED
#  ifndef PNG_tEXt_SUPPORTED
#    define PNG_tEXt_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_WRITE_tIME
#  define PNG_WRITE_tIME_SUPPORTED
#  ifndef PNG_tIME_SUPPORTED
#    define PNG_tIME_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_WRITE_tRNS
#  define PNG_WRITE_tRNS_SUPPORTED
#  ifndef PNG_tRNS_SUPPORTED
#    define PNG_tRNS_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_WRITE_zTXt
#  define PNG_WRITE_zTXt_SUPPORTED
#  ifndef PNG_zTXt_SUPPORTED
#    define PNG_zTXt_SUPPORTED
#  endif
#endif
#ifndef PNG_NO_WRITE_UNKNOWN_CHUNKS
#  define PNG_WRITE_UNKNOWN_CHUNKS_SUPPORTED
#  ifndef PNG_UNKNOWN_CHUNKS_SUPPORTED
#    define PNG_UNKNOWN_CHUNKS_SUPPORTED
#  endif
#  ifndef PNG_NO_HANDLE_AS_UNKNOWN
#     ifndef PNG_HANDLE_AS_UNKNOWN_SUPPORTED
#       define PNG_HANDLE_AS_UNKNOWN_SUPPORTED
#     endif
#  endif
#endif
#if defined(PNG_WRITE_iTXt_SUPPORTED) || defined(PNG_WRITE_tEXt_SUPPORTED) || \
    defined(PNG_WRITE_zTXt_SUPPORTED)
#  define PNG_WRITE_TEXT_SUPPORTED
#  ifndef PNG_TEXT_SUPPORTED
#    define PNG_TEXT_SUPPORTED
#  endif
#endif

#endif /* PNG_WRITE_ANCILLARY_CHUNKS_SUPPORTED */

#ifndef PNG_NO_INFO_IMAGE
#  define PNG_INFO_IMAGE_SUPPORTED
#endif

/* need the time information for reading tIME chunks */
#if defined(PNG_tIME_SUPPORTED)
#  if !defined(_WIN32_WCE)
     /* "time.h" functions are not supported on WindowsCE */
#    include <time.h>
#  endif
#endif


typedef unsigned long png_uint_32;
typedef long png_int_32;
typedef unsigned short png_uint_16;
typedef short png_int_16;
typedef unsigned char png_byte;

#ifdef PNG_SIZE_T
   typedef PNG_SIZE_T png_size_t;
#  define png_sizeof(x) png_convert_size(sizeof (x))
#else
   typedef size_t png_size_t;
#  define png_sizeof(x) sizeof (x)
#endif


#ifdef __BORLANDC__
#  if defined(__LARGE__) || defined(__HUGE__) || defined(__COMPACT__)
#    define LDATA 1
#  else
#    define LDATA 0
#  endif
   /* GRR:  why is Cygwin in here?  Cygwin is not Borland C... */
#  if !defined(__WIN32__) && !defined(__FLAT__) && !defined(__CYGWIN__)
#    define PNG_MAX_MALLOC_64K
#    if (LDATA != 1)
#      ifndef FAR
#        define FAR __far
#      endif
#      define USE_FAR_KEYWORD
#    endif   /* LDATA != 1 */
     /* Possibly useful for moving data out of default segment.
      * Uncomment it if you want. Could also define FARDATA as
      * const if your compiler supports it. (SJT)
#    define FARDATA FAR
      */
#  endif  /* __WIN32__, __FLAT__, __CYGWIN__ */
#endif   /* __BORLANDC__ */



/* MSC Medium model */
#if defined(FAR)
#  if defined(M_I86MM)
#    define USE_FAR_KEYWORD
#    define FARDATA FAR
#    include <dos.h>
#  endif
#endif

/* SJT: default case */
#ifndef FAR
#  define FAR
#endif

/* At this point FAR is always defined */
#ifndef FARDATA
#  define FARDATA
#endif

typedef png_int_32 png_fixed_point;

/* Add typedefs for pointers */
typedef void            FAR * png_voidp;
typedef png_byte        FAR * png_bytep;
typedef png_uint_32     FAR * png_uint_32p;
typedef png_int_32      FAR * png_int_32p;
typedef png_uint_16     FAR * png_uint_16p;
typedef png_int_16      FAR * png_int_16p;
typedef PNG_CONST char  FAR * png_const_charp;
typedef char            FAR * png_charp;
typedef png_fixed_point FAR * png_fixed_point_p;

#ifndef PNG_NO_STDIO
#if defined(_WIN32_WCE)
typedef HANDLE                png_FILE_p;
#else
typedef FILE                * png_FILE_p;
#endif
#endif

#ifdef PNG_FLOATING_POINT_SUPPORTED
typedef double          FAR * png_doublep;
#endif

/* Pointers to pointers; i.e. arrays */
typedef png_byte        FAR * FAR * png_bytepp;
typedef png_uint_32     FAR * FAR * png_uint_32pp;
typedef png_int_32      FAR * FAR * png_int_32pp;
typedef png_uint_16     FAR * FAR * png_uint_16pp;
typedef png_int_16      FAR * FAR * png_int_16pp;
typedef PNG_CONST char  FAR * FAR * png_const_charpp;
typedef char            FAR * FAR * png_charpp;
typedef png_fixed_point FAR * FAR * png_fixed_point_pp;
#ifdef PNG_FLOATING_POINT_SUPPORTED
typedef double          FAR * FAR * png_doublepp;
#endif

/* Pointers to pointers to pointers; i.e., pointer to array */
typedef char            FAR * FAR * FAR * png_charppp;

#if defined(PNG_1_0_X) || defined(PNG_1_2_X)
/* SPC -  Is this stuff deprecated? */
/* It'll be removed as of libpng-1.3.0 - GR-P */
typedef charf *         png_zcharp;
typedef charf * FAR *   png_zcharpp;
typedef z_stream FAR *  png_zstreamp;
#endif /* (PNG_1_0_X) || defined(PNG_1_2_X) */


#if !defined(PNG_DLL) && (defined(PNG_BUILD_DLL) || defined(PNG_USE_DLL))
#  define PNG_DLL
#endif
#if defined(__CYGWIN__)
#  if !defined(PNG_STATIC)
#    if defined(PNG_USE_GLOBAL_ARRAYS)
#      undef PNG_USE_GLOBAL_ARRAYS
#    endif
#    if !defined(PNG_USE_LOCAL_ARRAYS)
#      define PNG_USE_LOCAL_ARRAYS
#    endif
#  else
#    if defined(PNG_USE_LOCAL_ARRAYS) || defined(PNG_NO_GLOBAL_ARRAYS)
#      if defined(PNG_USE_GLOBAL_ARRAYS)
#        undef PNG_USE_GLOBAL_ARRAYS
#      endif
#    endif
#  endif
#  if !defined(PNG_USE_LOCAL_ARRAYS) && !defined(PNG_USE_GLOBAL_ARRAYS)
#    define PNG_USE_LOCAL_ARRAYS
#  endif
#endif

#if !defined(PNG_USE_LOCAL_ARRAYS) && !defined(PNG_USE_GLOBAL_ARRAYS)
#  if defined(PNG_NO_GLOBAL_ARRAYS) || \
      (defined(__GNUC__) && defined(PNG_DLL)) || defined(_MSC_VER)
#    define PNG_USE_LOCAL_ARRAYS
#  else
#    define PNG_USE_GLOBAL_ARRAYS
#  endif
#endif

#if defined(__CYGWIN__)
#  undef PNGAPI
#  define PNGAPI __cdecl
#  undef PNG_IMPEXP
#  define PNG_IMPEXP
#endif  


#if defined(__MINGW32__) && !defined(PNG_MODULEDEF)
#  ifndef PNG_NO_MODULEDEF
#    define PNG_NO_MODULEDEF
#  endif
#endif

#if !defined(PNG_IMPEXP) && defined(PNG_BUILD_DLL) && !defined(PNG_NO_MODULEDEF)
#  define PNG_IMPEXP
#endif

#if defined(PNG_DLL) || defined(_DLL) || defined(__DLL__ ) || \
    (( defined(_Windows) || defined(_WINDOWS) || \
       defined(WIN32) || defined(_WIN32) || defined(__WIN32__) ))

#  ifndef PNGAPI
#     if defined(__GNUC__) || (defined (_MSC_VER) && (_MSC_VER >= 800))
#        define PNGAPI __cdecl
#     else
#        define PNGAPI _cdecl
#     endif
#  endif

#  if !defined(PNG_IMPEXP) && (!defined(PNG_DLL) || \
       0 /* WINCOMPILER_WITH_NO_SUPPORT_FOR_DECLIMPEXP */)
#     define PNG_IMPEXP
#  endif

#  if !defined(PNG_IMPEXP)

#     define PNG_EXPORT_TYPE1(type,symbol)  PNG_IMPEXP type PNGAPI symbol
#     define PNG_EXPORT_TYPE2(type,symbol)  type PNG_IMPEXP PNGAPI symbol

      /* Borland/Microsoft */
#     if defined(_MSC_VER) || defined(__BORLANDC__)
#        if (_MSC_VER >= 800) || (__BORLANDC__ >= 0x500)
#           define PNG_EXPORT PNG_EXPORT_TYPE1
#        else
#           define PNG_EXPORT PNG_EXPORT_TYPE2
#           if defined(PNG_BUILD_DLL)
#              define PNG_IMPEXP __export
#           else
#              define PNG_IMPEXP /*__import */ /* doesn't exist AFAIK in
                                                 VC++ */
#           endif                             /* Exists in Borland C++ for
                                                 C++ classes (== huge) */
#        endif
#     endif

#     if !defined(PNG_IMPEXP)
#        if defined(PNG_BUILD_DLL)
#           define PNG_IMPEXP __declspec(dllexport)
#        else
#           define PNG_IMPEXP __declspec(dllimport)
#        endif
#     endif
#  endif  /* PNG_IMPEXP */
#else /* !(DLL || non-cygwin WINDOWS) */
#   if (defined(__IBMC__) || defined(__IBMCPP__)) && defined(__OS2__)
#      ifndef PNGAPI
#         define PNGAPI _System
#      endif
#   else
#      if 0 /* ... other platforms, with other meanings */
#      endif
#   endif
#endif

#ifndef PNGAPI
#  define PNGAPI
#endif
#ifndef PNG_IMPEXP
#  define PNG_IMPEXP
#endif

#ifdef PNG_BUILDSYMS
#  ifndef PNG_EXPORT
#    define PNG_EXPORT(type,symbol) PNG_FUNCTION_EXPORT symbol END
#  endif
#  ifdef PNG_USE_GLOBAL_ARRAYS
#    ifndef PNG_EXPORT_VAR
#      define PNG_EXPORT_VAR(type) PNG_DATA_EXPORT
#    endif
#  endif
#endif

#ifndef PNG_EXPORT
#  define PNG_EXPORT(type,symbol) PNG_IMPEXP type PNGAPI symbol
#endif

#ifdef PNG_USE_GLOBAL_ARRAYS
#  ifndef PNG_EXPORT_VAR
#    define PNG_EXPORT_VAR(type) extern PNG_IMPEXP type
#  endif
#endif


#ifndef PNG_ABORT
#  define PNG_ABORT() abort()
#endif

#ifdef PNG_SETJMP_SUPPORTED
#  define png_jmpbuf(png_ptr) ((png_ptr)->jmpbuf)
#else
#  define png_jmpbuf(png_ptr) \
   (LIBPNG_WAS_COMPILED_WITH__PNG_SETJMP_NOT_SUPPORTED)
#endif

#if defined(USE_FAR_KEYWORD)  /* memory model independent fns */
/* use this to make far-to-near assignments */
#  define CHECK   1
#  define NOCHECK 0
#  define CVT_PTR(ptr) (png_far_to_near(png_ptr,ptr,CHECK))
#  define CVT_PTR_NOCHECK(ptr) (png_far_to_near(png_ptr,ptr,NOCHECK))
#  define png_snprintf _fsnprintf   /* Added to v 1.2.19 */
#  define png_strcpy  _fstrcpy
#  define png_strncpy _fstrncpy   /* Added to v 1.2.6 */
#  define png_strlen  _fstrlen
#  define png_memcmp  _fmemcmp    /* SJT: added */
#  define png_memcpy  _fmemcpy
#  define png_memset  _fmemset
#else /* use the usual functions */
#  define CVT_PTR(ptr)         (ptr)
#  define CVT_PTR_NOCHECK(ptr) (ptr)
#  ifndef PNG_NO_SNPRINTF
#    ifdef _MSC_VER
#      define png_snprintf _snprintf   /* Added to v 1.2.19 */
#      define png_snprintf2 _snprintf
#      define png_snprintf6 _snprintf
#    else
#      define png_snprintf snprintf   /* Added to v 1.2.19 */
#      define png_snprintf2 snprintf
#      define png_snprintf6 snprintf
#    endif
#  else
     /* You don't have or don't want to use snprintf().  Caution: Using
      * sprintf instead of snprintf exposes your application to accidental
      * or malevolent buffer overflows.  If you don't have snprintf()
      * as a general rule you should provide one (you can get one from
      * Portable OpenSSH). */
#    define png_snprintf(s1,n,fmt,x1) sprintf(s1,fmt,x1)
#    define png_snprintf2(s1,n,fmt,x1,x2) sprintf(s1,fmt,x1,x2)
#    define png_snprintf6(s1,n,fmt,x1,x2,x3,x4,x5,x6) \
        sprintf(s1,fmt,x1,x2,x3,x4,x5,x6)
#  endif
#  define png_strcpy  strcpy
#  define png_strncpy strncpy     /* Added to v 1.2.6 */
#  define png_strlen  strlen
#  define png_memcmp  memcmp      /* SJT: added */
#  define png_memcpy  memcpy
#  define png_memset  memset
#endif
/* End of memory model independent support */

#if (PNG_ZBUF_SIZE > 65536L) && defined(PNG_MAX_MALLOC_64K)
#  undef PNG_ZBUF_SIZE
#  define PNG_ZBUF_SIZE 65536L
#endif

#ifdef PNG_READ_SUPPORTED
/* Prior to libpng-1.0.9, this block was in pngasmrd.h */
#if defined(PNG_INTERNAL)

#if defined(PNG_USE_PNGGCCRD) || defined(PNG_USE_PNGVCRD)
  /* Platform must be Pentium.  Makefile must assemble and load
   * pnggccrd.c or  pngvcrd.c. MMX will be detected at run time and
   * used if present.
   */
#  ifndef PNG_NO_MMX_COMBINE_ROW
#    define PNG_HAVE_MMX_COMBINE_ROW
#  endif
#  ifndef PNG_NO_MMX_READ_INTERLACE
#    define PNG_HAVE_MMX_READ_INTERLACE
#  endif
#  ifndef PNG_NO_MMX_READ_FILTER_ROW
#    define PNG_HAVE_MMX_READ_FILTER_ROW
#    ifndef PNG_NO_MMX_FILTER_SUB
#      define PNG_MMX_READ_FILTER_SUB_SUPPORTED
#    endif
#    if !(defined(__GNUC__) && defined(__x86_64__) && (__GNUC__ < 4))
       /* work around 64-bit gcc compiler bugs in gcc-3.x */
#      ifndef PNG_NO_MMX_FILTER_UP
#        define PNG_MMX_READ_FILTER_UP_SUPPORTED
#      endif
#      ifndef PNG_NO_MMX_FILTER_AVG
#        define PNG_MMX_READ_FILTER_AVG_SUPPORTED
#      endif
#      ifndef PNG_NO_MMX_FILTER_PAETH
#        define PNG_MMX_READ_FILTER_PAETH_SUPPORTED
#      endif
#    endif /* !((__x86_64__) && (GNUC < 4)) */
#  endif
  /* These are the default thresholds before the MMX code kicks in; if either
   * rowbytes or bitdepth is below the threshold, plain C code is used.  These
   * can be overridden at runtime via the png_set_mmx_thresholds() call in
   * libpng 1.2.0 and later.  The values below were chosen by Intel.
   */
#  ifndef PNG_MMX_ROWBYTES_THRESHOLD_DEFAULT
#    define PNG_MMX_ROWBYTES_THRESHOLD_DEFAULT  128  /*  >=  */
#  endif
#  ifndef PNG_MMX_BITDEPTH_THRESHOLD_DEFAULT
#    define PNG_MMX_BITDEPTH_THRESHOLD_DEFAULT  9    /*  >=  */   
#  endif
#endif /* PNG_USE_PNGGCCRD || PNG_USE_PNGVCRD */
/* - see pngvcrd.c or pnggccrd.c for info about what is currently enabled */

#endif /* PNG_INTERNAL */
#endif /* PNG_READ_SUPPORTED */

/* Added at libpng-1.2.8 */
#endif /* PNG_VERSION_INFO_ONLY */

#endif /* PNGCONF_H */
