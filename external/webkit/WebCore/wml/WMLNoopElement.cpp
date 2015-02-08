

#include "config.h"

#if ENABLE(WML)
#include "WMLNoopElement.h"

#include "WMLDoElement.h"
#include "WMLErrorHandling.h"
#include "WMLNames.h"

namespace WebCore {

using namespace WMLNames;

WMLNoopElement::WMLNoopElement(const QualifiedName& tagName, Document* doc)
    : WMLElement(tagName, doc)
{
}

WMLNoopElement::~WMLNoopElement()
{
}

void WMLNoopElement::insertedIntoDocument()
{
    WMLElement::insertedIntoDocument();

    Node* parent = parentNode();
    if (!parent || !parent->isWMLElement())
        return;

    if (parent->hasTagName(doTag)) {
        WMLDoElement* doElement = static_cast<WMLDoElement*>(parent);
        doElement->setNoop(true);

        if (doElement->attached())
            doElement->detach();

        ASSERT(!doElement->attached());
        doElement->attach();
    } else if (parent->hasTagName(anchorTag))
        reportWMLError(document(), WMLErrorForbiddenTaskInAnchorElement);
}

}

#endif
