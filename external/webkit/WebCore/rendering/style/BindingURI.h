

#ifndef BindingURI_h
#define BindingURI_h
#if ENABLE(XBL)

#include "StringImpl.h"

namespace WebCore {

// This struct holds information about shadows for the text-shadow and box-shadow properties.

struct BindingURI {
    BindingURI(StringImpl*);
    ~BindingURI();

    BindingURI* copy();

    bool operator==(const BindingURI& o) const;
    bool operator!=(const BindingURI& o) const
    {
        return !(*this == o);
    }

    BindingURI* next() { return m_next; }
    void setNext(BindingURI* n) { m_next = n; }

    StringImpl* uri() { return m_uri; }

    BindingURI* m_next;
    StringImpl* m_uri;
};

} // namespace WebCore

#endif // ENABLE(XBL)
#endif // BindingURI_h
