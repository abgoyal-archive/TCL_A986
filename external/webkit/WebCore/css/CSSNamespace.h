

#ifndef CSSNamespace_h
#define CSSNamespace_h

#include "AtomicString.h"

namespace WebCore {

    struct CSSNamespace : Noncopyable {
        AtomicString m_prefix;
        AtomicString m_uri;
        CSSNamespace* m_parent;

        CSSNamespace(const AtomicString& prefix, const AtomicString& uri, CSSNamespace* parent)
            : m_prefix(prefix)
            , m_uri(uri)
            , m_parent(parent)
        {
        }
        ~CSSNamespace() { delete m_parent; }

        const AtomicString& uri() { return m_uri; }
        const AtomicString& prefix() { return m_prefix; }

        CSSNamespace* namespaceForPrefix(const AtomicString& prefix)
        {
            if (prefix == m_prefix)
                return this;
            if (m_parent)
                return m_parent->namespaceForPrefix(prefix);
            return 0;
        }
    };

} // namespace WebCore

#endif // CSSNamespace_h
