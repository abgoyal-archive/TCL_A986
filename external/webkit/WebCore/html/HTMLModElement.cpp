
#include "config.h"
#include "HTMLModElement.h"

#include "HTMLNames.h"

namespace WebCore {

using namespace HTMLNames;

HTMLModElement::HTMLModElement(const QualifiedName& tagName, Document *doc)
    : HTMLElement(tagName, doc)
{
}

String HTMLModElement::cite() const
{
    return getAttribute(citeAttr);
}

void HTMLModElement::setCite(const String& value)
{
    setAttribute(citeAttr, value);
}

String HTMLModElement::dateTime() const
{
    return getAttribute(datetimeAttr);
}

void HTMLModElement::setDateTime(const String& value)
{
    setAttribute(datetimeAttr, value);
}

}
