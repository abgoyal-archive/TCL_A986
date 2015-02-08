

#include "config.h"
#include "HTMLIsIndexElement.h"

#include "HTMLNames.h"
#include "MappedAttribute.h"

namespace WebCore {

using namespace HTMLNames;

HTMLIsIndexElement::HTMLIsIndexElement(const QualifiedName& tagName, Document *doc, HTMLFormElement *f)
    : HTMLInputElement(tagName, doc, f)
{
    ASSERT(hasTagName(isindexTag));
    setDefaultName(isindexTag.localName());
}

void HTMLIsIndexElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == promptAttr)
        setValue(attr->value());
    else if (attr->name() == placeholderAttr)
        updatePlaceholderVisibility(true);
    else
        // don't call HTMLInputElement::parseMappedAttribute here, as it would
        // accept attributes this element does not support
        HTMLFormControlElement::parseMappedAttribute(attr);
}

String HTMLIsIndexElement::prompt() const
{
    return getAttribute(promptAttr);
}

void HTMLIsIndexElement::setPrompt(const String &value)
{
    setAttribute(promptAttr, value);
}

} // namespace
