
#include "config.h"
#include "HTMLDirectoryElement.h"

#include "HTMLNames.h"

namespace WebCore {

using namespace HTMLNames;

HTMLDirectoryElement::HTMLDirectoryElement(const QualifiedName& tagName, Document* doc)
    : HTMLElement(tagName, doc)
{
    ASSERT(hasTagName(dirTag));
}

bool HTMLDirectoryElement::compact() const
{
    return !getAttribute(compactAttr).isNull();
}

void HTMLDirectoryElement::setCompact(bool b)
{
    setAttribute(compactAttr, b ? "" : 0);
}

}
