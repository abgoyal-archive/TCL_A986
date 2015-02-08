

#ifndef WebNodeCollection_h
#define WebNodeCollection_h

#include "WebCommon.h"

namespace WebCore { class HTMLCollection; }
#if WEBKIT_IMPLEMENTATION
namespace WTF { template <typename T> class PassRefPtr; }
#endif

namespace WebKit {
class WebNode;

// Provides readonly access to some properties of a DOM node.
class WebNodeCollection {
public:
    ~WebNodeCollection() { reset(); }

    WebNodeCollection() : m_private(0) { }
    WebNodeCollection(const WebNodeCollection& n) : m_private(0) { assign(n); }
    WebNodeCollection& operator=(const WebNodeCollection& n)
    {
        assign(n);
        return *this;
    }

    WEBKIT_API void reset();
    WEBKIT_API void assign(const WebNodeCollection&);

    WEBKIT_API unsigned length() const;
    WEBKIT_API WebNode nextItem() const;
    WEBKIT_API WebNode firstItem() const;

#if WEBKIT_IMPLEMENTATION
    WebNodeCollection(const WTF::PassRefPtr<WebCore::HTMLCollection>&);
#endif

private:
    void assign(WebCore::HTMLCollection*);
    WebCore::HTMLCollection* m_private;
};

} // namespace WebKit

#endif
