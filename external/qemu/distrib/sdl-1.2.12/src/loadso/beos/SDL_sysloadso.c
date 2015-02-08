
#include "SDL_config.h"

#ifdef SDL_LOADSO_BEOS

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* System dependent library loading routines                           */

#include <stdio.h>
#include <be/kernel/image.h>

#include "SDL_loadso.h"

void *
SDL_LoadObject(const char *sofile)
{
    void *handle = NULL;
    image_id library_id = load_add_on(sofile);
    if (library_id < 0) {
        SDL_SetError(strerror((int) library_id));
    } else {
        handle = (void *) (library_id);
    }
    return (handle);
}

void *
SDL_LoadFunction(void *handle, const char *name)
{
    void *sym = NULL;
    image_id library_id = (image_id) handle;
    status_t rc = get_image_symbol(library_id, name, B_SYMBOL_TYPE_TEXT, &sym);
    if (rc != B_NO_ERROR) {
        SDL_SetError(strerror(rc));
    }
    return (sym);
}

void
SDL_UnloadObject(void *handle)
{
    image_id library_id;
    if (handle != NULL) {
        library_id = (image_id) handle;
        unload_add_on(library_id);
    }
}

#endif /* SDL_LOADSO_BEOS */

/* vi: set ts=4 sw=4 expandtab: */

