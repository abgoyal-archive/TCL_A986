

#include "config.h"
#include "HTMLBlockquoteElement.h"

#include "HTMLNames.h"

namespace WebCore {

using namespace HTMLNames;

HTMLBlockquoteElement::HTMLBlockquoteElement(const QualifiedName& tagName, Document* document)
    : HTMLElement(tagName, document)
{
    ASSERT(hasTagName(blockquoteTag));
}

}
