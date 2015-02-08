
#include "SDL_config.h"

/* This is the system specific header for the SDL CD-ROM API */

/* Structure of CD audio control functions */
extern struct CDcaps {
	/* Get the name of the specified drive */
	const char *(*Name)(int drive);

	/* Open the specified drive, returning a drive id, or -1 on error */
	int (*Open)(int drive);

	/* Get table-of-contents (number of tracks + track info) for disk.
	   The TOC information should be stored in the cdrom structure.
	   This function should return 0 on success, or -1 on error.
	 */
	int (*GetTOC)(SDL_CD *cdrom);

	/* Return the current status and play position, in frames, of the
	   drive.  'position' may be NULL, and if so, should be ignored.
	 */
	CDstatus (*Status)(SDL_CD *cdrom, int *position);

	/* Play from frame 'start' to 'start+len' */
	int (*Play)(SDL_CD *cdrom, int start, int len); 

	/* Pause play */
	int (*Pause)(SDL_CD *cdrom);

	/* Resume play */
	int (*Resume)(SDL_CD *cdrom);

	/* Stop play */
	int (*Stop)(SDL_CD *cdrom);

	/* Eject the current disk */
	int (*Eject)(SDL_CD *cdrom);

	/* Close the specified drive */
	void (*Close)(SDL_CD *cdrom);
} SDL_CDcaps;

/* The number of available CD-ROM drives on the system */
extern int SDL_numcds;

extern int  SDL_SYS_CDInit(void);

/* Function to perform any system-specific CD-ROM related cleanup */
extern void SDL_SYS_CDQuit(void);

