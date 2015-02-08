
#include "SDL_config.h"

/* StormMesa implementation of SDL OpenGL support */

#include "../SDL_sysvideo.h"

#define _THIS   SDL_VideoDevice *_this

#if SDL_VIDEO_OPENGL
#include <GL/Amigamesa.h>
extern void *AmiGetGLProc(const char *proc);
#endif /* SDL_VIDEO_OPENGL */

struct SDL_PrivateGLData {
	int gl_active;
};

/* OpenGL functions */
extern int CGX_GL_Init(_THIS);
extern void CGX_GL_Quit(_THIS);
extern int CGX_GL_Update(_THIS);
#if SDL_VIDEO_OPENGL
extern int CGX_GL_MakeCurrent(_THIS);
extern int CGX_GL_GetAttribute(_THIS, SDL_GLattr attrib, int* value);
extern void CGX_GL_SwapBuffers(_THIS);
extern void *CGX_GL_GetProcAddress(_THIS, const char *proc);
extern int CGX_GL_LoadLibrary(_THIS, const char *path);
#endif

#undef _THIS
