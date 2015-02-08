
#include "SDL_config.h"

#ifndef __CDPlayer__H__
#define __CDPlayer__H__ 1

#include <string.h>

#include <Carbon/Carbon.h>
#include <CoreFoundation/CoreFoundation.h>
#include <AudioUnit/AudioUnit.h>

#include "SDL.h"
#include "SDL_thread.h"
#include "SDL_mutex.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*CDPlayerCompletionProc)(SDL_CD *cdrom) ;

void     Lock ();

void     Unlock();

int      LoadFile (const FSRef *ref, int startFrame, int endFrame); /* pass -1 to do nothing */

int      ReleaseFile ();

int      PlayFile  ();

int      PauseFile ();

void     SetCompletionProc (CDPlayerCompletionProc proc, SDL_CD *cdrom);

int      ReadTOCData (FSVolumeRefNum theVolume, SDL_CD *theCD);

int      ListTrackFiles (FSVolumeRefNum theVolume, FSRef *trackFiles, int numTracks);

int      DetectAudioCDVolumes (FSVolumeRefNum *volumes, int numVolumes);

int      GetCurrentFrame ();

#ifdef __cplusplus
};
#endif

#endif /* __CD_Player__H__ */
