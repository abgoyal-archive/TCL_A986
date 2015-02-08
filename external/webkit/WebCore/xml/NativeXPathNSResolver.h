

#ifndef NativeXPathNSResolver_h
#define NativeXPathNSResolver_h

#if ENABLE(XPATH)

#include "XPathNSResolver.h"
#include "Node.h"
#include <wtf/RefPtr.h>

namespace WebCore {

    class NativeXPathNSResolver : public XPathNSResolver {
    public:
        static PassRefPtr<NativeXPathNSResolver> create(PassRefPtr<Node> node) { return adoptRef(new NativeXPathNSResolver(node)); }
        virtual ~NativeXPathNSResolver();

        virtual String lookupNamespaceURI(const String& prefix);

    private:
        NativeXPathNSResolver(PassRefPtr<Node>);
        RefPtr<Node> m_node;
    };

} // namespace WebCore

#endif // ENABLE(XPATH)

#endif // NativeXPathNSResolver_h
