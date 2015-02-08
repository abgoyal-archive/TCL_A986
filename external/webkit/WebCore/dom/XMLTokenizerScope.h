

#ifndef XMLTokenizerScope_h
#define XMLTokenizerScope_h

#include <wtf/Noncopyable.h>

#if ENABLE(XSLT)
#include <libxml/tree.h>
#endif

namespace WebCore {

    class DocLoader;

    class XMLTokenizerScope : public Noncopyable {
    public:
        XMLTokenizerScope(DocLoader* docLoader);
        ~XMLTokenizerScope();

        static DocLoader* currentDocLoader;

#if ENABLE(XSLT)
        XMLTokenizerScope(DocLoader* docLoader, xmlGenericErrorFunc genericErrorFunc, xmlStructuredErrorFunc structuredErrorFunc = 0, void* errorContext = 0);
#endif

    private:
        DocLoader* m_oldDocLoader;

#if ENABLE(XSLT)
        xmlGenericErrorFunc m_oldGenericErrorFunc;
        xmlStructuredErrorFunc m_oldStructuredErrorFunc;
        void* m_oldErrorContext;
#endif
    };

} // namespace WebCore

#endif // XMLTokenizerScope_h
