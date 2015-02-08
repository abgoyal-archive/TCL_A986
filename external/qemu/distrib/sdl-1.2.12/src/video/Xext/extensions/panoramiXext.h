
/* $Xorg: panoramiXext.h,v 1.4 2000/08/18 04:05:45 coskrey Exp $ */
/* $XFree86: xc/include/extensions/panoramiXext.h,v 3.6 2001/01/17 17:53:22 dawes Exp $ */

#include "SDL_name.h"

/* THIS IS NOT AN X PROJECT TEAM SPECIFICATION */

#define PANORAMIX_MAJOR_VERSION         1       /* current version number */
#define PANORAMIX_MINOR_VERSION         1

typedef struct {
    Window  window;         /* PanoramiX window - may not exist */
    int	    screen;
    int     State;          /* PanroamiXOff, PanoramiXOn */
    int	    width;	    /* width of this screen */
    int     height;	    /* height of this screen */
    int     ScreenCount;    /* real physical number of screens */
    XID     eventMask;      /* selected events for this client */
} SDL_NAME(XPanoramiXInfo);    

extern SDL_NAME(XPanoramiXInfo) *SDL_NAME(XPanoramiXAllocInfo) (
#if NeedFunctionPrototypes
    void
#endif
);        
