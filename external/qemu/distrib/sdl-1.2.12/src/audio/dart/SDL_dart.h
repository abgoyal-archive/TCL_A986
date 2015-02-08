
#include "SDL_config.h"

#ifndef _SDL_lowaudio_h
#define _SDL_lowaudio_h

#define INCL_TYPES
#define INCL_DOSSEMAPHORES
#define INCL_DOSRESOURCES
#define INCL_DOSMISC
#define INCL_DOSERRORS

#define INCL_OS2MM
#define INCL_MMIOOS2
#define INCL_MCIOS2
#include <os2.h>
#include <os2me.h>     // DART stuff and MMIO stuff

#include "../SDL_sysaudio.h"

/* Hidden "this" pointer for the audio functions */
#define _THIS	SDL_AudioDevice *_this

/* The DirectSound objects */
struct SDL_PrivateAudioData
{
  int iCurrDeviceOrd;
  int iCurrFreq;
  int iCurrBits;
  int iCurrChannels;
  int iCurrNumBufs;
  int iCurrBufSize;

  int iLastPlayedBuf;
  int iNextFreeBuffer;

  MCI_BUFFER_PARMS BufferParms;     // Sound buffer parameters
  MCI_MIX_BUFFER *pMixBuffers;      // Sound buffers
  MCI_MIXSETUP_PARMS MixSetupParms; // Mixer setup parameters
  HEV hevAudioBufferPlayed;         // Event semaphore to indicate that an audio buffer has been played by DART
};

#endif /* _SDL_lowaudio_h */
