

#ifndef _BC_DTS_TYPES_H_
#define _BC_DTS_TYPES_H_

#ifdef __LINUX_USER__  /* Don't include these for KERNEL.. */
#include <stdint.h>
#endif

#ifndef PVOID
typedef void	*PVOID;
#endif

#ifndef BOOL
typedef int	BOOL;
#endif

#if defined(__KERNEL__) || defined(__LINUX_USER__)

#ifdef __LINUX_USER__	/* Don't include these for KERNEL */
typedef uint32_t	ULONG;
typedef int32_t		LONG;
typedef void		*HANDLE;
#ifndef VOID
typedef void		VOID;
#endif
typedef void		*LPVOID;
typedef uint32_t	DWORD;
typedef uint32_t	UINT32;
typedef uint32_t	*LPDWORD;
typedef unsigned char	*PUCHAR;

#ifndef TRUE
	#define TRUE		1
#endif

#ifndef FALSE
	#define FALSE		0
#endif

#define TEXT

#else

/* For Kernel usage.. */
typedef bool	bc_bool_t;
#endif

#else

#ifndef uint64_t
typedef struct _uint64_t {
	uint32_t low_dw;
	uint32_t hi_dw;
} uint64_t;
#endif

#ifndef int32_t
typedef signed long		int32_t;
#endif

#ifndef uint32_t
typedef unsigned long	uint32_t;
#endif

#ifndef uint16_t
typedef unsigned short	uint16_t;
#endif

#ifndef uint8_t
typedef unsigned char	uint8_t;
#endif
#endif

#endif

