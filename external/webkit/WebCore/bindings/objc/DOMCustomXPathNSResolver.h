

#ifndef DOMCustomXPathNSResolver_h
#define DOMCustomXPathNSResolver_h

#if ENABLE(XPATH)

#include "XPathNSResolver.h"

#include "DOMXPathNSResolver.h"
#include <wtf/PassRefPtr.h>

namespace WebCore {

    class Frame;

    class DOMCustomXPathNSResolver : public XPathNSResolver {
    public:
        static PassRefPtr<DOMCustomXPathNSResolver> create(id <DOMXPathNSResolver> customResolver) { return adoptRef(new DOMCustomXPathNSResolver(customResolver)); }
        virtual ~DOMCustomXPathNSResolver();

        virtual String lookupNamespaceURI(const String& prefix);

    private:
        DOMCustomXPathNSResolver(id <DOMXPathNSResolver>);
        id <DOMXPathNSResolver> m_customResolver; // DOMCustomXPathNSResolvers are always temporary, thus no need to GC protect the object.
    };

} // namespace WebCore

#endif // ENABLE(XPATH)

#endif // DOMCustomXPathNSResolver_h
