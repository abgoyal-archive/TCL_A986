

/* AGL implementation of SDL OpenGL support */

#include "SDL_config.h"

#if SDL_VIDEO_OPENGL
#include "SDL_opengl.h"
#if __MACOSX__
#include <AGL/agl.h>   /* AGL.framework */
#else
#include <agl.h>
#endif
#endif /* SDL_VIDEO_OPENGL */

/* OpenGL functions */
extern int Mac_GL_Init(_THIS);
extern void Mac_GL_Quit(_THIS);
#if SDL_VIDEO_OPENGL
extern int Mac_GL_MakeCurrent(_THIS);
extern int Mac_GL_GetAttribute(_THIS, SDL_GLattr attrib, int* value);
extern void Mac_GL_SwapBuffers(_THIS);
extern int Mac_GL_LoadLibrary(_THIS, const char *location);
extern void Mac_GL_UnloadLibrary(_THIS);
extern void* Mac_GL_GetProcAddress(_THIS, const char *proc);
#endif

