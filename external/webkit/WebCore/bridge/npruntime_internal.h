

#include "npapi.h"
#include "npfunctions.h"
#include "npruntime.h"

#ifdef XP_UNIX
    #include <X11/Xresource.h>

    #undef None
    #undef Above
    #undef Below
    #undef Auto
    #undef Complex
    #undef Status
    #undef CursorShape
    #undef FocusIn
    #undef FocusOut
    #undef KeyPress
    #undef KeyRelease
    #undef Unsorted
    #undef Bool
    #undef FontChange
    #undef GrayScale
    #undef NormalState
#endif
