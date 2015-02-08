

#include "config.h"
#include "WebDocument.h"

#include "Document.h"
#include "DocumentLoader.h"
#include "Element.h"
#include "HTMLAllCollection.h"
#include "HTMLBodyElement.h"
#include "HTMLCollection.h"
#include "HTMLElement.h"
#include "HTMLHeadElement.h"
#include "NodeList.h"

#include "WebElement.h"
#include "WebFrameImpl.h"
#include "WebNodeCollection.h"
#include "WebURL.h"

#include <wtf/PassRefPtr.h>

using namespace WebCore;

namespace WebKit {

WebDocument::WebDocument(const PassRefPtr<Document>& elem)
    : WebNode(elem.releaseRef())
{
}

WebDocument& WebDocument::operator=(const PassRefPtr<Document>& elem)
{
    WebNode::assign(elem.releaseRef());
    return *this;
}

WebDocument::operator PassRefPtr<Document>() const
{
    return PassRefPtr<Document>(static_cast<Document*>(m_private));
}

WebFrame* WebDocument::frame() const
{
    return WebFrameImpl::fromFrame(constUnwrap<Document>()->frame());
}

bool WebDocument::isHTMLDocument() const
{  
    return constUnwrap<Document>()->isHTMLDocument();
}

WebURL WebDocument::baseURL() const
{
    return constUnwrap<Document>()->baseURL();
}

WebElement WebDocument::documentElement() const
{
    return WebElement(constUnwrap<Document>()->documentElement());
}

WebElement WebDocument::body() const
{
    return WebElement(constUnwrap<Document>()->body());
}

WebElement WebDocument::head()
{
    return WebElement(unwrap<Document>()->head());
}

WebNodeCollection WebDocument::all()
{
    return WebNodeCollection(unwrap<Document>()->all());
}

WebURL WebDocument::completeURL(const WebString& partialURL) const
{
    return constUnwrap<Document>()->completeURL(partialURL);
}

WebElement WebDocument::getElementById(const WebString& id) const
{
    return WebElement(constUnwrap<Document>()->getElementById(id));
}

WebString WebDocument::applicationID() const
{
    const char* kChromeApplicationHeader = "x-chrome-application";

    // First check if the document's response included a header indicating the
    // application it should go with.
    const Document* document = constUnwrap<Document>();
    Frame* frame = document->frame();
    if (!frame)
        return WebString();

    DocumentLoader* loader = frame->loader()->documentLoader();
    if (!loader)
        return WebString();

    WebString headerValue =
        loader->response().httpHeaderField(kChromeApplicationHeader);
    if (!headerValue.isEmpty())
        return headerValue;

    // Otherwise, fall back to looking for the meta tag.
    RefPtr<NodeList> metaTags =
        const_cast<Document*>(document)->getElementsByTagName("meta");
    for (unsigned i = 0; i < metaTags->length(); ++i) {
        Element* element = static_cast<Element*>(metaTags->item(i));
        if (element->getAttribute("http-equiv").lower() ==
                kChromeApplicationHeader) {
            return element->getAttribute("value");
        }
    }

    return WebString();
}

} // namespace WebKit
