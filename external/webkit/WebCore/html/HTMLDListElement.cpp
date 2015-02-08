
#include "config.h"
#include "HTMLDListElement.h"

#include "HTMLNames.h"

namespace WebCore {

using namespace HTMLNames;

HTMLDListElement::HTMLDListElement(const QualifiedName& tagName, Document* doc)
    : HTMLElement(tagName, doc)
{
    ASSERT(hasTagName(dlTag));
}

bool HTMLDListElement::compact() const
{
    return !getAttribute(compactAttr).isNull();
}

void HTMLDListElement::setCompact(bool b)
{
    setAttribute(compactAttr, b ? "" : 0);
}

}
