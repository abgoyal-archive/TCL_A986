

#ifndef WebDocument_h
#define WebDocument_h

#include "WebNode.h"

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class Document; }
namespace WTF { template <typename T> class PassRefPtr; }
#endif

namespace WebKit {
class WebElement;
class WebFrame;
class WebNodeCollection;
class WebString;
class WebURL;

// Provides readonly access to some properties of a DOM document.
class WebDocument : public WebNode {
public:
    WebDocument() { }
    WebDocument(const WebDocument& e) : WebNode(e) { }

    WebDocument& operator=(const WebDocument& e)
    {
        WebNode::assign(e);
        return *this;
    }
    void assign(const WebDocument& e) { WebNode::assign(e); }

    // Returns the frame the document belongs to or 0 if the document is frameless.
    WEBKIT_API WebFrame* frame() const;
    WEBKIT_API bool isHTMLDocument() const;
    WEBKIT_API WebURL baseURL() const;
    WEBKIT_API WebElement documentElement() const;
    WEBKIT_API WebElement body() const;
    WEBKIT_API WebElement head();
    WEBKIT_API WebNodeCollection all();
    WEBKIT_API WebURL completeURL(const WebString&) const;
    WEBKIT_API WebElement getElementById(const WebString& id) const;
    WEBKIT_API WebString applicationID() const;

#if WEBKIT_IMPLEMENTATION
    WebDocument(const WTF::PassRefPtr<WebCore::Document>&);
    WebDocument& operator=(const WTF::PassRefPtr<WebCore::Document>&);
    operator WTF::PassRefPtr<WebCore::Document>() const;
#endif
};

} // namespace WebKit

#endif
