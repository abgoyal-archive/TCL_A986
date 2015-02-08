

#include "config.h"
#include "HTMLBaseFontElement.h"

#include "HTMLNames.h"

namespace WebCore {

using namespace HTMLNames;

HTMLBaseFontElement::HTMLBaseFontElement(const QualifiedName& tagName, Document* document)
    : HTMLElement(tagName, document)
{
    ASSERT(hasTagName(basefontTag));
}

int HTMLBaseFontElement::size() const
{
    return getAttribute(sizeAttr).toInt();
}

void HTMLBaseFontElement::setSize(int value)
{
    setAttribute(sizeAttr, String::number(value));
}

}
