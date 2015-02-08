

#include "config.h"
#include "DNS.h"

#include "CString.h"
#include "GOwnPtrGtk.h"
#include "ResourceHandle.h"

namespace WebCore {

void prefetchDNS(const String& hostname)
{
#ifdef HAVE_LIBSOUP_2_29_90
    String uri = "http://"+hostname;
    GOwnPtr<SoupURI> soupURI(soup_uri_new(uri.utf8().data()));
    // We may get invalid hostnames, so NULL-check here.
    if (!soupURI)
        return;
    soup_session_prepare_for_uri(ResourceHandle::defaultSession(), soupURI.get());
#endif
}

}
