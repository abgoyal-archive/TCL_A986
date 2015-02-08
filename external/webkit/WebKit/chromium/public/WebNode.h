

#ifndef WebNode_h
#define WebNode_h

#include "WebCommon.h"
#include "WebString.h"

namespace WebCore { class Node; }
#if WEBKIT_IMPLEMENTATION
namespace WTF { template <typename T> class PassRefPtr; }
#endif

namespace WebKit {
class WebDocument;
class WebEventListener;
class WebEventListenerPrivate;
class WebFrame;
class WebNodeList;

// Provides access to some properties of a DOM node.
class WebNode {
public:
    virtual ~WebNode() { reset(); }

    WebNode() : m_private(0) { }
    WebNode(const WebNode& n) : m_private(0) { assign(n); }
    WebNode& operator=(const WebNode& n)
    {
        assign(n);
        return *this;
    }

    WEBKIT_API void reset();
    WEBKIT_API void assign(const WebNode&);

    bool isNull() const { return !m_private; }

#if WEBKIT_IMPLEMENTATION
    WebNode(const WTF::PassRefPtr<WebCore::Node>&);
    WebNode& operator=(const WTF::PassRefPtr<WebCore::Node>&);
    operator WTF::PassRefPtr<WebCore::Node>() const;
#endif

    enum NodeType {
        ElementNode = 1,
        AttributeNode,
        TextNode,
        CDataSectionNode,
        EntityReferenceNode,
        EntityNode,
        ProcessingInstructionsNode,
        CommentNode,
        DocumentNode,
        DocumentTypeNode,
        DocumentFragmentNode,
        NotationNode,
        XPathNamespaceNode
    };
    WEBKIT_API NodeType nodeType() const;
    WEBKIT_API WebNode parentNode() const;
    WEBKIT_API WebString nodeName() const;
    WEBKIT_API WebString nodeValue() const;
    WEBKIT_API bool setNodeValue(const WebString&);
    // Deprecated. Use document().frame() instead.
    WEBKIT_API WebFrame* frame() const;
    WEBKIT_API WebDocument document() const;
    WEBKIT_API WebNode firstChild() const;
    WEBKIT_API WebNode lastChild() const;
    WEBKIT_API WebNode previousSibling() const;
    WEBKIT_API WebNode nextSibling() const;
    WEBKIT_API bool hasChildNodes() const;
    WEBKIT_API WebNodeList childNodes();
    WEBKIT_API WebString createMarkup() const;
    WEBKIT_API bool isTextNode() const;
    WEBKIT_API bool isElementNode() const;
    WEBKIT_API void addEventListener(const WebString& eventType, WebEventListener* listener, bool useCapture);
    WEBKIT_API void removeEventListener(const WebString& eventType, WebEventListener* listener, bool useCapture);

    template<typename T> T toElement()
    {
        T res;
        res.WebNode::assign(*this);
        return res;
    }

    template<typename T> const T toConstElement() const
    {
        T res;
        res.WebNode::assign(*this);
        return res;
    }

protected:
    typedef WebCore::Node WebNodePrivate;
    void assign(WebNodePrivate*);
    WebNodePrivate* m_private;
    
    template<typename T> T* unwrap()
    {
        return static_cast<T*>(m_private);
    }

    template<typename T> const T* constUnwrap() const
    {
        return static_cast<const T*>(m_private);
    }
};

} // namespace WebKit

#endif
