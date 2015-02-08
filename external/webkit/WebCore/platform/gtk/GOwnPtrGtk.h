

#ifndef GOwnPtrGtk_h
#define GOwnPtrGtk_h

#include "GOwnPtr.h"

typedef struct _SoupURI SoupURI;
typedef struct _GstElement GstElement;

namespace WTF {

template<> void freeOwnedGPtr<SoupURI>(SoupURI* ptr);
template<> void freeOwnedGPtr<GstElement>(GstElement* ptr);

}

#endif
