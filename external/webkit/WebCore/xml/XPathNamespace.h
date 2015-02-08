

#ifndef XPathNamespace_h
#define XPathNamespace_h

#if ENABLE(XPATH)

#include "AtomicString.h"
#include "Node.h"

namespace WebCore {

    // FIXME: This class is never instantiated. Maybe it should be removed.

    class XPathNamespace : public Node {
    private:
        XPathNamespace(PassRefPtr<Element> ownerElement, const AtomicString& prefix, const AtomicString& uri);

        virtual Document* ownerDocument() const;
        virtual Element* ownerElement() const;

        virtual const AtomicString& prefix() const;
        virtual String nodeName() const;
        virtual String nodeValue() const;
        virtual const AtomicString& namespaceURI() const;

        virtual NodeType nodeType() const;

        RefPtr<Element> m_ownerElement;
        AtomicString m_prefix;
        AtomicString m_uri;
    };

}

#endif // ENABLE(XPATH)

#endif // XPathNamespace_h

