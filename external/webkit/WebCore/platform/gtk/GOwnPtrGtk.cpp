


#include "config.h"
#include "GOwnPtrGtk.h"

#if ENABLE(VIDEO)
#include <gst/gstelement.h>
#endif
#include <libsoup/soup-uri.h>

namespace WTF {

template <> void freeOwnedGPtr<SoupURI>(SoupURI* ptr)
{
    if (ptr)
        soup_uri_free(ptr);
}

#if ENABLE(VIDEO)
template <> void freeOwnedGPtr<GstElement>(GstElement* ptr)
{
    if (ptr)
        gst_object_unref(ptr);
}
#endif

}
