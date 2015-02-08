
#include "SDL_config.h"

#ifdef SDL_LOADSO_MACOS

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* System dependent library loading routines                           */

#include <stdio.h>
#include <string.h>
#define OLDP2C 1
#include <Strings.h>
#include <CodeFragments.h>
#include <Errors.h>

#include "SDL_loadso.h"

void *SDL_LoadObject(const char *sofile)
{
	void *handle = NULL;
	const char *loaderror = NULL;
	CFragConnectionID library_id;
	Ptr mainAddr;
	Str255 errName;
	OSErr error;
	char psofile[512];

	SDL_strlcpy(psofile, sofile, SDL_arraysize(psofile));
	error = GetSharedLibrary(C2PStr(psofile), kCompiledCFragArch,
			kLoadCFrag, &library_id, &mainAddr, errName);
	switch (error) {
		case noErr:
			loaderror = NULL;
			break;
		case cfragNoLibraryErr:
			loaderror = "Library not found";
			break;
		case cfragUnresolvedErr:
			loaderror = "Unabled to resolve symbols";
			break;
		case cfragNoPrivateMemErr:
		case cfragNoClientMemErr:
			loaderror = "Out of memory";
			break;
		default:
			loaderror = "Unknown Code Fragment Manager error";
			break;
	}
	if ( loaderror == NULL ) {
		handle = (void *)(library_id);
	} else {
		SDL_SetError("Failed loading %s: %s", sofile, loaderror);
	}
	return(handle);
}

void *SDL_LoadFunction(void *handle, const char *name)
{
	void *symbol = NULL;
	const char *loaderror = NULL;
	CFragSymbolClass class;
	CFragConnectionID library_id = (CFragConnectionID)handle;
	char pname[512];

	SDL_strlcpy(pname, name, SDL_arraysize(pname));
	if ( FindSymbol(library_id, C2PStr(pname),
	                (char **)&symbol, &class) != noErr ) {
		loaderror = "Symbol not found";
	}

	if ( symbol == NULL ) {
		SDL_SetError("Failed loading %s: %s", name, loaderror);
	}
	return(symbol);
}

void SDL_UnloadObject(void *handle)
{
	CFragConnectionID library_id;
	if ( handle != NULL ) {
		library_id = (CFragConnectionID)handle;
		CloseConnection(&library_id);
	}
}

#endif /* SDL_LOADSO_MACOS */
