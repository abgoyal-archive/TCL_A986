
#include "SDL_config.h"

#if SDL_THREADS_DISABLED

typedef int SYS_ThreadHandle;

#else

#include <pthread.h>

typedef pthread_t SYS_ThreadHandle;

#endif
