

#include "config.h"
#include "XMLTokenizerScope.h"

namespace WebCore {

DocLoader* XMLTokenizerScope::currentDocLoader = 0;

XMLTokenizerScope::XMLTokenizerScope(DocLoader* docLoader)
    : m_oldDocLoader(currentDocLoader)
#if ENABLE(XSLT)
    , m_oldGenericErrorFunc(xmlGenericError)
    , m_oldStructuredErrorFunc(xmlStructuredError)
    , m_oldErrorContext(xmlGenericErrorContext)
#endif
{
    currentDocLoader = docLoader;
}

#if ENABLE(XSLT)
XMLTokenizerScope::XMLTokenizerScope(DocLoader* docLoader, xmlGenericErrorFunc genericErrorFunc, xmlStructuredErrorFunc structuredErrorFunc, void* errorContext)
    : m_oldDocLoader(currentDocLoader)
    , m_oldGenericErrorFunc(xmlGenericError)
    , m_oldStructuredErrorFunc(xmlStructuredError)
    , m_oldErrorContext(xmlGenericErrorContext)
{
    currentDocLoader = docLoader;
    if (genericErrorFunc)
        xmlSetGenericErrorFunc(errorContext, genericErrorFunc);
    if (structuredErrorFunc)
        xmlSetStructuredErrorFunc(errorContext, structuredErrorFunc);
}
#endif

XMLTokenizerScope::~XMLTokenizerScope()
{
    currentDocLoader = m_oldDocLoader;
#if ENABLE(XSLT)
    xmlSetGenericErrorFunc(m_oldErrorContext, m_oldGenericErrorFunc);
    xmlSetStructuredErrorFunc(m_oldErrorContext, m_oldStructuredErrorFunc);
#endif
}

}
