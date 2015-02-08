

#ifndef XPathNSResolver_h
#define XPathNSResolver_h

#if ENABLE(XPATH)

#include <wtf/RefCounted.h>

namespace WebCore {

    class String;

    class XPathNSResolver : public RefCounted<XPathNSResolver> {
    public:
        virtual ~XPathNSResolver();
        virtual String lookupNamespaceURI(const String& prefix) = 0;
        
    protected:
        XPathNSResolver() { }
    };

}

#endif // ENABLE(XPATH)

#endif // XPathNSResolver_h
