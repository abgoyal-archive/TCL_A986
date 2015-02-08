
#include "SDL_config.h"

#if defined(SDL_LOADSO_DUMMY) || defined(SDL_LOADSO_DISABLED)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* System dependent library loading routines                           */

#include "SDL_loadso.h"

void *SDL_LoadObject(const char *sofile)
{
	const char *loaderror = "SDL_LoadObject() not implemented";
	SDL_SetError("Failed loading %s: %s", sofile, loaderror);
	return(NULL);
}

void *SDL_LoadFunction(void *handle, const char *name)
{
	const char *loaderror = "SDL_LoadFunction() not implemented";
	SDL_SetError("Failed loading %s: %s", name, loaderror);
	return(NULL);
}

void SDL_UnloadObject(void *handle)
{
    /* no-op. */
}

#endif /* SDL_LOADSO_DUMMY || SDL_LOADSO_DISABLED */
