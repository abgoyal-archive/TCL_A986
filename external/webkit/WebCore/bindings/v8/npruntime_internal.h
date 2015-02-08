

#include "npapi.h"
#include "npruntime.h"
#include "npfunctions.h"

#ifdef XP_UNIX
    #include <X11/Xresource.h>

    #undef None
    #undef Above
    #undef Below
    #undef Auto
    #undef Complex
    #undef Status
#endif
