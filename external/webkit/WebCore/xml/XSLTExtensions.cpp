

#include "config.h"

#if ENABLE(XSLT)
#include "XSLTExtensions.h"

#include <libxml/xpathInternals.h>

#include <libxslt/xsltutils.h>
#include <libxslt/extensions.h>
#include <libxslt/extra.h>

#if PLATFORM(MAC)
#include "SoftLinking.h"
#endif

#if PLATFORM(MAC)
SOFT_LINK_LIBRARY(libxslt)
SOFT_LINK(libxslt, xsltRegisterExtFunction, int, (xsltTransformContextPtr ctxt, const xmlChar *name, const xmlChar *URI, xmlXPathFunction function), (ctxt, name, URI, function))
SOFT_LINK(libxslt, xsltFunctionNodeSet, void, (xmlXPathParserContextPtr ctxt, int nargs), (ctxt, nargs))
#endif

namespace WebCore {

// FIXME: This code is taken from libexslt 1.1.11; should sync with newer versions.
static void exsltNodeSetFunction(xmlXPathParserContextPtr ctxt, int nargs)
{
    xmlChar *strval;
    xmlNodePtr retNode;
    xmlXPathObjectPtr ret;

    if (nargs != 1) {
        xmlXPathSetArityError(ctxt);
        return;
    }

    if (xmlXPathStackIsNodeSet(ctxt)) {
        xsltFunctionNodeSet(ctxt, nargs);
        return;
    }

    strval = xmlXPathPopString(ctxt);
    retNode = xmlNewDocText(NULL, strval);
    ret = xmlXPathNewValueTree(retNode);
    
    // FIXME: It might be helpful to push any errors from xmlXPathNewValueTree
    // up to the Javascript Console.
    if (ret != NULL) 
        ret->type = XPATH_NODESET;

    if (strval != NULL)
        xmlFree(strval);

    valuePush(ctxt, ret);
}

void registerXSLTExtensions(xsltTransformContextPtr ctxt)
{
    xsltRegisterExtFunction(ctxt, (const xmlChar*)"node-set", (const xmlChar*)"http://exslt.org/common", exsltNodeSetFunction);
}

}

#endif
