
#include "config.h"
#include "HTMLQuoteElement.h"

#include "Document.h"
#include "HTMLNames.h"

namespace WebCore {

using namespace HTMLNames;

HTMLQuoteElement::HTMLQuoteElement(const QualifiedName& tagName, Document* doc)
    : HTMLElement(tagName, doc)
{
    ASSERT(hasTagName(qTag));
}

void HTMLQuoteElement::insertedIntoDocument()
{
    document()->setUsesBeforeAfterRules(true);

    HTMLElement::insertedIntoDocument();
}

String HTMLQuoteElement::cite() const
{
    return getAttribute(citeAttr);
}

void HTMLQuoteElement::setCite(const String &value)
{
    setAttribute(citeAttr, value);
}

}
