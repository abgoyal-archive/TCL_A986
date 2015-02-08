

#ifndef XSLTExtensions_h
#define XSLTExtensions_h

#if ENABLE(XSLT)

#include <libxslt/xsltInternals.h>

namespace WebCore {

void registerXSLTExtensions(xsltTransformContextPtr ctxt);

}

#endif
#endif
