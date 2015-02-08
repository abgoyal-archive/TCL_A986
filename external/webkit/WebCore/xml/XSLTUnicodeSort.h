
#ifndef XSLTUnicodeSort_h
#define XSLTUnicodeSort_h

#if ENABLE(XSLT)

#include <libxslt/xsltInternals.h>

namespace WebCore {

    void xsltUnicodeSortFunction(xsltTransformContextPtr ctxt, xmlNodePtr* sorts, int nbsorts);

}

#endif
#endif
