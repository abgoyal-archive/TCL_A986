

#ifndef _SDL_config_macos_h
#define _SDL_config_macos_h

#include "SDL_platform.h"

/* This is a set of defines to configure the SDL features */

#include <MacTypes.h>

typedef SInt8	int8_t;
typedef UInt8	uint8_t;
typedef SInt16	int16_t;
typedef UInt16	uint16_t;
typedef SInt32	int32_t;
typedef UInt32	uint32_t;
typedef SInt64	int64_t;
typedef UInt64	uint64_t;
typedef unsigned long	uintptr_t;

#define SDL_HAS_64BIT_TYPE	1

/* Useful headers */
#define HAVE_STDIO_H	1
#define STDC_HEADERS	1
#define HAVE_STRING_H	1
#define HAVE_CTYPE_H	1
#define HAVE_MATH_H	1
#define HAVE_SIGNAL_H	1

/* C library functions */
#define HAVE_MALLOC	1
#define HAVE_CALLOC	1
#define HAVE_REALLOC	1
#define HAVE_FREE	1
#define HAVE_ALLOCA	1
#define HAVE_ABS	1
#define HAVE_MEMSET	1
#define HAVE_MEMCPY	1
#define HAVE_MEMMOVE	1
#define HAVE_MEMCMP	1
#define HAVE_STRLEN	1
#define HAVE_STRCHR	1
#define HAVE_STRRCHR	1
#define HAVE_STRSTR	1
#define HAVE_ITOA	1
#define HAVE_STRTOL	1
#define HAVE_STRTOD	1
#define HAVE_ATOI	1
#define HAVE_ATOF	1
#define HAVE_STRCMP	1
#define HAVE_STRNCMP	1
#define HAVE_SSCANF	1

/* Enable various audio drivers */
#define SDL_AUDIO_DRIVER_SNDMGR	1
#define SDL_AUDIO_DRIVER_DISK	1
#define SDL_AUDIO_DRIVER_DUMMY	1

/* Enable various cdrom drivers */
#if TARGET_API_MAC_CARBON
#define SDL_CDROM_DUMMY		1
#else
#define SDL_CDROM_MACOS		1
#endif

/* Enable various input drivers */
#if TARGET_API_MAC_CARBON
#define SDL_JOYSTICK_DUMMY	1
#else
#define SDL_JOYSTICK_MACOS	1
#endif

/* Enable various shared object loading systems */
#define SDL_LOADSO_MACOS	1

/* Enable various threading systems */
#define SDL_THREADS_DISABLED	1

/* Enable various timer systems */
#define SDL_TIMER_MACOS	1

/* Enable various video drivers */
#define SDL_VIDEO_DRIVER_DUMMY	1
#define SDL_VIDEO_DRIVER_DRAWSPROCKET	1
#define SDL_VIDEO_DRIVER_TOOLBOX	1

/* Enable OpenGL support */
#define SDL_VIDEO_OPENGL	1

#endif /* _SDL_config_macos_h */
