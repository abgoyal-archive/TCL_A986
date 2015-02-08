
/*	BSDI osd_defs.h,v 1.4 1998/06/03 19:14:58 karels Exp	*/

#ifndef		_OSD_DEFS_H
#define		_OSD_DEFS_H



/*Definitions - Defines & Constants ----------------------------------------- */

  /* Define the operating system */
#if (defined(__linux__))
# define _DPT_LINUX
#elif (defined(__bsdi__))
# define _DPT_BSDI
#elif (defined(__FreeBSD__))
# define _DPT_FREE_BSD
#else
# define _DPT_SCO
#endif

#if defined (ZIL_CURSES)
#define		_DPT_CURSES
#else
#define         _DPT_MOTIF
#endif

  /* Redefine 'far' to nothing - no far pointer type required in UNIX */
#define		far

  /* Define the mutually exclusive semaphore type */
#define		SEMAPHORE_T	unsigned int *
  /* Define a handle to a DLL */
#define		DLL_HANDLE_T	unsigned int *

#endif
