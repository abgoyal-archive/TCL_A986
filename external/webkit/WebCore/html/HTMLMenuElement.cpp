
#include "config.h"
#include "HTMLMenuElement.h"

#include "HTMLNames.h"

namespace WebCore {

using namespace HTMLNames;

HTMLMenuElement::HTMLMenuElement(const QualifiedName& tagName, Document* doc)
    : HTMLElement(tagName, doc)
{
    ASSERT(hasTagName(menuTag));
}

bool HTMLMenuElement::compact() const
{
    return !getAttribute(compactAttr).isNull();
}

void HTMLMenuElement::setCompact(bool b)
{
    setAttribute(compactAttr, b ? "" : 0);
}

}
