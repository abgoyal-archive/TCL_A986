

/* This is the CD-audio control API for Simple DirectMedia Layer */

#ifndef _SDL_cdrom_h
#define _SDL_cdrom_h

#include "SDL_stdinc.h"
#include "SDL_error.h"

#include "begin_code.h"
/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif


/* The maximum number of CD-ROM tracks on a disk */
#define SDL_MAX_TRACKS	99

/* The types of CD-ROM track possible */
#define SDL_AUDIO_TRACK	0x00
#define SDL_DATA_TRACK	0x04

/* The possible states which a CD-ROM drive can be in. */
typedef enum {
	CD_TRAYEMPTY,
	CD_STOPPED,
	CD_PLAYING,
	CD_PAUSED,
	CD_ERROR = -1
} CDstatus;

/* Given a status, returns true if there's a disk in the drive */
#define CD_INDRIVE(status)	((int)(status) > 0)

typedef struct SDL_CDtrack {
	Uint8 id;		/* Track number */
	Uint8 type;		/* Data or audio track */
	Uint16 unused;
	Uint32 length;		/* Length, in frames, of this track */
	Uint32 offset;		/* Offset, in frames, from start of disk */
} SDL_CDtrack;

/* This structure is only current as of the last call to SDL_CDStatus() */
typedef struct SDL_CD {
	int id;			/* Private drive identifier */
	CDstatus status;	/* Current drive status */

	/* The rest of this structure is only valid if there's a CD in drive */
	int numtracks;		/* Number of tracks on disk */
	int cur_track;		/* Current track position */
	int cur_frame;		/* Current frame offset within current track */
	SDL_CDtrack track[SDL_MAX_TRACKS+1];
} SDL_CD;

/* Conversion functions from frames to Minute/Second/Frames and vice versa */
#define CD_FPS	75
#define FRAMES_TO_MSF(f, M,S,F)	{					\
	int value = f;							\
	*(F) = value%CD_FPS;						\
	value /= CD_FPS;						\
	*(S) = value%60;						\
	value /= 60;							\
	*(M) = value;							\
}
#define MSF_TO_FRAMES(M, S, F)	((M)*60*CD_FPS+(S)*CD_FPS+(F))

/* CD-audio API functions: */

extern DECLSPEC int SDLCALL SDL_CDNumDrives(void);

extern DECLSPEC const char * SDLCALL SDL_CDName(int drive);

extern DECLSPEC SDL_CD * SDLCALL SDL_CDOpen(int drive);

extern DECLSPEC CDstatus SDLCALL SDL_CDStatus(SDL_CD *cdrom);

extern DECLSPEC int SDLCALL SDL_CDPlayTracks(SDL_CD *cdrom,
		int start_track, int start_frame, int ntracks, int nframes);

extern DECLSPEC int SDLCALL SDL_CDPlay(SDL_CD *cdrom, int start, int length);

/* Pause play -- returns 0, or -1 on error */
extern DECLSPEC int SDLCALL SDL_CDPause(SDL_CD *cdrom);

/* Resume play -- returns 0, or -1 on error */
extern DECLSPEC int SDLCALL SDL_CDResume(SDL_CD *cdrom);

/* Stop play -- returns 0, or -1 on error */
extern DECLSPEC int SDLCALL SDL_CDStop(SDL_CD *cdrom);

/* Eject CD-ROM -- returns 0, or -1 on error */
extern DECLSPEC int SDLCALL SDL_CDEject(SDL_CD *cdrom);

/* Closes the handle for the CD-ROM drive */
extern DECLSPEC void SDLCALL SDL_CDClose(SDL_CD *cdrom);


/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif
#include "close_code.h"

#endif /* _SDL_video_h */
