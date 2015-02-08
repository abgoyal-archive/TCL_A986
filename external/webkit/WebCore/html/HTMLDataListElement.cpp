

#include "config.h"
#if ENABLE(DATALIST)
#include "HTMLDataListElement.h"

#include "HTMLNames.h"

namespace WebCore {

HTMLDataListElement::HTMLDataListElement(const QualifiedName& tagName, Document *doc)
    : HTMLElement(tagName, doc)
{
}

HTMLDataListElement::~HTMLDataListElement()
{
}

bool HTMLDataListElement::checkDTD(const Node* newChild)
{
    return newChild->hasTagName(HTMLNames::optionTag) || HTMLElement::inInlineTagList(newChild);
}

PassRefPtr<HTMLCollection> HTMLDataListElement::options()
{
    return HTMLCollection::create(this, DataListOptions);
}

}  // namespace WebCore

#endif  // ENABLE(DATALIST)
