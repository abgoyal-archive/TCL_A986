
#include "SDL_config.h"

#if SDL_VIDEO_OPENGL_GLX
#include <GL/glx.h>
#include "SDL_loadso.h"
#endif

#include "../SDL_sysvideo.h"

struct SDL_PrivateGLData {
    int gl_active; /* to stop switching drivers while we have a valid context */

#if SDL_VIDEO_OPENGL_GLX
    GLXContext glx_context;	/* Current GL context */
    XVisualInfo* glx_visualinfo; /* XVisualInfo* returned by glXChooseVisual */

    void * (*glXGetProcAddress)(const GLubyte *procName);

    XVisualInfo* (*glXChooseVisual)
		( Display*		dpy,
		  int			screen,
		  int*			attribList );

    GLXContext (*glXCreateContext)
		( Display*		dpy,
		  XVisualInfo*		vis,
		  GLXContext		shareList,
		  Bool			direct );

    void (*glXDestroyContext)
		( Display* 		dpy,
		  GLXContext		ctx );

    Bool (*glXMakeCurrent)
		( Display*		dpy,
		  GLXDrawable		drawable,
		  GLXContext		ctx );

    void (*glXSwapBuffers)
		( Display*		dpy,
		  GLXDrawable		drawable );

    int (*glXGetConfig)
	 ( Display* dpy,
	   XVisualInfo* visual_info,
	   int attrib,
	   int* value );

    const char *(*glXQueryExtensionsString)
	    ( Display* dpy,
	      int screen );

    int (*glXSwapIntervalSGI) ( int interval );
    GLint (*glXSwapIntervalMESA) ( unsigned interval );
    GLint (*glXGetSwapIntervalMESA) ( void );

#endif /* SDL_VIDEO_OPENGL_GLX */
};

/* Old variable names */
#define gl_active		(this->gl_data->gl_active)
#define glx_context		(this->gl_data->glx_context)
#define glx_visualinfo		(this->gl_data->glx_visualinfo)

/* OpenGL functions */
extern XVisualInfo *X11_GL_GetVisual(_THIS);
extern int X11_GL_CreateWindow(_THIS, int w, int h);
extern int X11_GL_CreateContext(_THIS);
extern void X11_GL_Shutdown(_THIS);
#if SDL_VIDEO_OPENGL_GLX
extern int X11_GL_MakeCurrent(_THIS);
extern int X11_GL_GetAttribute(_THIS, SDL_GLattr attrib, int* value);
extern void X11_GL_SwapBuffers(_THIS);
extern int X11_GL_LoadLibrary(_THIS, const char* path);
extern void *X11_GL_GetProcAddress(_THIS, const char* proc);
#endif
extern void X11_GL_UnloadLibrary(_THIS);

