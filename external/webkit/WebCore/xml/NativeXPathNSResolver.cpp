

#include "config.h"
#include "NativeXPathNSResolver.h"

#if ENABLE(XPATH)

#include "Node.h"
#include "PlatformString.h"
#include "XMLNames.h"

namespace WebCore {

NativeXPathNSResolver::NativeXPathNSResolver(PassRefPtr<Node> node)
    : m_node(node)
{
}

NativeXPathNSResolver::~NativeXPathNSResolver()
{
}

String NativeXPathNSResolver::lookupNamespaceURI(const String& prefix)
{
    // This is not done by Node::lookupNamespaceURI as per the DOM3 Core spec,
    // but the XPath spec says that we should do it for XPathNSResolver.
    if (prefix == "xml")
        return XMLNames::xmlNamespaceURI;
    
    return m_node ? m_node->lookupNamespaceURI(prefix) : String();
}

} // namespace WebCore

#endif // ENABLE(XPATH)
