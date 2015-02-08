

#ifndef PlatformWidget_h
#define PlatformWidget_h

#include <wtf/StdLibExtras.h>  // for intptr_t

// PlatformWidget is an opaque identifier corresponding to whatever native
// view type the embedder may use.  PlatformWidget CANNOT be assumed to be
// a valid pointer.  Some embedders may not use this identifier at all.

typedef intptr_t PlatformWidget;

#endif
