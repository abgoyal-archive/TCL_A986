

#ifndef GRefPtrGtk_h
#define GRefPtrGtk_h

#include "GRefPtr.h"

typedef struct _GtkTargetList GtkTargetList;
typedef struct _GdkCursor GdkCursor;

namespace WTF {

template <> GtkTargetList* refGPtr(GtkTargetList* ptr);
template <> void derefGPtr(GtkTargetList* ptr);

template <> GdkCursor* refGPtr(GdkCursor* ptr);
template <> void derefGPtr(GdkCursor* ptr);

}

#endif
