
#include "SDL_config.h"

#ifdef SDL_LOADSO_LDG

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* System dependent library loading routines                           */

#include <stdio.h>
#include <gem.h>
#include <ldg.h>

#include "SDL_loadso.h"

void *SDL_LoadObject(const char *sofile)
{
	const char *loaderror = "Unknown error";
	void *handle = (void *)ldg_open((char *)sofile, ldg_global);
	if ( handle == NULL ) {
		SDL_SetError("Failed loading %s: %s", sofile, loaderror);
	}
	return(handle);
}

void *SDL_LoadFunction(void *handle, const char *name)
{
	const char *loaderror = "Unknown error";
	void *symbol = (void *)ldg_find((char *)name, (LDG *)handle);
	if ( symbol == NULL ) {
		SDL_SetError("Failed loading %s: %s", name, loaderror);
	}
	return(symbol);
}

void SDL_UnloadObject(void *handle)
{
	if ( handle != NULL ) {
		ldg_close((LDG *)handle, ldg_global);
	}
}

#endif /* SDL_LOADSO_LDG */
