
#include "SDL_config.h"

#ifndef _SDL_mutex_c_h
#define _SDL_mutex_c_h

struct SDL_mutex {
	pthread_mutex_t id;
};

#endif /* _SDL_mutex_c_h */
