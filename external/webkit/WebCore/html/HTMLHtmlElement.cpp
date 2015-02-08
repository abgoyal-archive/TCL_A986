
#include "config.h"
#include "HTMLHtmlElement.h"

#include "ApplicationCacheHost.h"
#include "Document.h"
#include "DocumentLoader.h"
#include "Frame.h"
#include "HTMLNames.h"

namespace WebCore {

using namespace HTMLNames;

HTMLHtmlElement::HTMLHtmlElement(const QualifiedName& tagName, Document* doc)
    : HTMLElement(tagName, doc)
{
    ASSERT(hasTagName(htmlTag));
}

HTMLHtmlElement::~HTMLHtmlElement()
{
}

String HTMLHtmlElement::version() const
{
    return getAttribute(versionAttr);
}

void HTMLHtmlElement::setVersion(const String &value)
{
    setAttribute(versionAttr, value);
}

bool HTMLHtmlElement::checkDTD(const Node* newChild)
{
    return newChild->hasTagName(headTag) || newChild->hasTagName(bodyTag) ||
           newChild->hasTagName(framesetTag) || newChild->hasTagName(noframesTag);
}

#if ENABLE(OFFLINE_WEB_APPLICATIONS)
void HTMLHtmlElement::insertedIntoDocument()
{
    HTMLElement::insertedIntoDocument();
    
    if (!document()->parsing())
        return;

    if (!document()->frame())
        return;

    DocumentLoader* documentLoader = document()->frame()->loader()->documentLoader();
    if (!documentLoader)
        return;

    // Check the manifest attribute
    // FIXME: Revisit this when we get a clarification from whatwg on how to handle empty
    // manifest attributes. As spec'd, and coded here, the system will initiate an update
    // passing in the document url as the manifest url. That's not a good thing.
    AtomicString manifest = getAttribute(manifestAttr);
    if (manifest.isNull())
        documentLoader->applicationCacheHost()->selectCacheWithoutManifest();
    else
        documentLoader->applicationCacheHost()->selectCacheWithManifest(document()->completeURL(manifest));
}
#endif

}
