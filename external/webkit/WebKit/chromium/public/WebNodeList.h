

#ifndef WebNodeList_h
#define WebNodeList_h

#include "WebCommon.h"

namespace WebCore { class NodeList; }
#if WEBKIT_IMPLEMENTATION
namespace WTF { template <typename T> class PassRefPtr; }
#endif

namespace WebKit {
class WebNode;

// Provides readonly access to some properties of a DOM node.
class WebNodeList {
public:
    ~WebNodeList() { reset(); }

    WebNodeList() : m_private(0) { }
    WebNodeList(const WebNodeList& n) : m_private(0) { assign(n); }
    WebNodeList& operator=(const WebNodeList& n)
    {
        assign(n);
        return *this;
    }

    WEBKIT_API void reset();
    WEBKIT_API void assign(const WebNodeList&);

    WEBKIT_API unsigned length() const;
    WEBKIT_API WebNode item(size_t) const;

#if WEBKIT_IMPLEMENTATION
    WebNodeList(const WTF::PassRefPtr<WebCore::NodeList>&);
#endif

private:
    void assign(WebCore::NodeList*);
    WebCore::NodeList* m_private;
};

} // namespace WebKit

#endif
