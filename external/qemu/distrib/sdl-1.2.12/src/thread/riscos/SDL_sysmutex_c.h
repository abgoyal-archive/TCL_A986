
#include "SDL_config.h"

#ifndef _SDL_mutex_c_h
#define _SDL_mutex_c_h

#if !SDL_THREADS_DISABLED
struct SDL_mutex {
	pthread_mutex_t id;
};
#endif


#endif /* _SDL_mutex_c_h */
