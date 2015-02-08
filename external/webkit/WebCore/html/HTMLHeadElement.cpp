
#include "config.h"
#include "HTMLHeadElement.h"

#include "HTMLNames.h"
#include "Text.h"

namespace WebCore {

using namespace HTMLNames;

HTMLHeadElement::HTMLHeadElement(const QualifiedName& qName, Document* doc)
    : HTMLElement(qName, doc)
{
    ASSERT(hasTagName(headTag));
}

HTMLHeadElement::~HTMLHeadElement()
{
}

String HTMLHeadElement::profile() const
{
    return getAttribute(profileAttr);
}

void HTMLHeadElement::setProfile(const String &value)
{
    setAttribute(profileAttr, value);
}

bool HTMLHeadElement::childAllowed(Node* newChild)
{
    // Do not allow non-whitespace text nodes in the head
    if (newChild->isTextNode())
        return static_cast<Text*>(newChild)->containsOnlyWhitespace();
    
    return HTMLElement::childAllowed(newChild);
}

bool HTMLHeadElement::checkDTD(const Node* newChild)
{
    return newChild->hasTagName(noscriptTag) || newChild->hasTagName(titleTag) || newChild->hasTagName(isindexTag) ||
           newChild->hasTagName(baseTag) || newChild->hasTagName(scriptTag) ||
           newChild->hasTagName(styleTag) || newChild->hasTagName(metaTag) ||
           newChild->hasTagName(linkTag) || newChild->isTextNode();
}

}
