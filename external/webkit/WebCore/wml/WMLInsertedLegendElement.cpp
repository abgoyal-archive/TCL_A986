

#include "config.h"

#if ENABLE(WML)
#include "WMLInsertedLegendElement.h"

namespace WebCore {

WMLInsertedLegendElement::WMLInsertedLegendElement(const QualifiedName& tagName, Document* doc)
    : WMLElement(tagName, doc)
{
}

WMLInsertedLegendElement::~WMLInsertedLegendElement()
{
}

}

#endif
