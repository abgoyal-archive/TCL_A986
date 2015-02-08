
#include "SDL_config.h"

#ifndef __SDL_PH_EVENTS_H__
#define __SDL_PH_EVENTS_H__

#include "SDL_ph_video.h"

#define PH_SDL_MAX_RECTS 256
#define PH_EVENT_SAFETY_POOL 512
#define EVENT_SIZE (sizeof(PhEvent_t) + 1000 + PH_EVENT_SAFETY_POOL)

/* Functions to be exported */
extern void ph_InitOSKeymap(_THIS);
extern void ph_PumpEvents(_THIS);

#endif /* __SDL_PH_EVENTS_H__ */
