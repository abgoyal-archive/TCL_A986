
#include "config.h"
#include "HTMLHeadingElement.h"

#include "HTMLNames.h"

namespace WebCore {

using namespace HTMLNames;

HTMLHeadingElement::HTMLHeadingElement(const QualifiedName& tagName, Document *doc)
    : HTMLElement(tagName, doc)
{
}

bool HTMLHeadingElement::checkDTD(const Node* newChild)
{
    if (newChild->hasTagName(h1Tag) || newChild->hasTagName(h2Tag) ||
        newChild->hasTagName(h3Tag) || newChild->hasTagName(h4Tag) ||
        newChild->hasTagName(h5Tag) || newChild->hasTagName(h6Tag))
        return false;

    return inEitherTagList(newChild);
}

String HTMLHeadingElement::align() const
{
    return getAttribute(alignAttr);
}

void HTMLHeadingElement::setAlign(const String &value)
{
    setAttribute(alignAttr, value);
}

}
