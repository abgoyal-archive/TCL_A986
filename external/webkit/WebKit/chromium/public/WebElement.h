

#ifndef WebElement_h
#define WebElement_h

#include "WebNode.h"

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class Element; }
namespace WTF { template <typename T> class PassRefPtr; }
#endif

namespace WebKit {
    // Provides access to some properties of a DOM element node.
    class WebElement : public WebNode {
    public:
        WebElement() : WebNode() { }
        WebElement(const WebElement& e) : WebNode(e) { }

        WebElement& operator=(const WebElement& e) { WebNode::assign(e); return *this; }
        void assign(const WebElement& e) { WebNode::assign(e); }

        WEBKIT_API WebString tagName() const;
        WEBKIT_API bool hasTagName(const WebString&) const;
        WEBKIT_API bool hasAttribute(const WebString&) const;
        WEBKIT_API WebString getAttribute(const WebString&) const;
        WEBKIT_API bool setAttribute(const WebString& name, const WebString& value);
        WEBKIT_API WebString innerText() const;

#if WEBKIT_IMPLEMENTATION
        WebElement(const WTF::PassRefPtr<WebCore::Element>&);
        WebElement& operator=(const WTF::PassRefPtr<WebCore::Element>&);
        operator WTF::PassRefPtr<WebCore::Element>() const;
#endif
    };

} // namespace WebKit

#endif
