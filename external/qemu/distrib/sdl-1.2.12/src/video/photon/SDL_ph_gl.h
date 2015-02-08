
#include "SDL_config.h"

#ifndef __SDL_PH_GL_H__
#define __SDL_PH_GL_H__

#include "SDL_ph_video.h"

#define DEFAULT_OPENGL "/usr/lib/libGL.so"

#if SDL_VIDEO_OPENGL
    void  ph_GL_SwapBuffers(_THIS);
    int   ph_GL_GetAttribute(_THIS, SDL_GLattr attrib, int* value);
    int   ph_GL_LoadLibrary(_THIS, const char* path);
    void* ph_GL_GetProcAddress(_THIS, const char* proc);
    int   ph_GL_MakeCurrent(_THIS);

    int   ph_SetupOpenGLContext(_THIS, int width, int height, int bpp, Uint32 flags);
#endif /* SDL_VIDEO_OPENGL */

#endif /* __SDL_PH_GL_H__ */
