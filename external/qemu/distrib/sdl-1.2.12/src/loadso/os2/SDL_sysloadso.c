
#include "SDL_config.h"

#ifdef SDL_LOADSO_OS2

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* System dependent library loading routines                           */

#include <stdio.h>
#define INCL_DOSERRORS
#define INCL_DOSMODULEMGR
#include <os2.h>

#include "SDL_loadso.h"

void *SDL_LoadObject(const char *sofile)
{
    HMODULE handle = NULL;
    char buf[512];
    APIRET ulrc = DosLoadModule(buf, sizeof (buf), (char *) sofile, &handle);

    /* Generate an error message if all loads failed */
    if ((ulrc != NO_ERROR) || (handle == NULL))
        SDL_SetError("Failed loading %s: %s", sofile, buf);

    return((void *) handle);
}

void *SDL_LoadFunction(void *handle, const char *name)
{
    const char *loaderror = "Unknown error";
    void *symbol = NULL;
    APIRET ulrc = DosQueryProcAddr((HMODULE)handle, 0, (char *)name, &symbol);
    if (ulrc == ERROR_INVALID_HANDLE)
        loaderror = "Invalid module handle";
    else if (ulrc == ERROR_INVALID_NAME)
        loaderror = "Symbol not found";

    if (symbol == NULL)
        SDL_SetError("Failed loading %s: %s", name, loaderror);

    return(symbol);
}

void SDL_UnloadObject(void *handle)
{
    if ( handle != NULL )
        DosFreeModule((HMODULE) handle);
}

#endif /* SDL_LOADSO_OS2 */
