

#include "config.h"

#if ENABLE(WML)
#include "WMLFieldSetElement.h"

#include "HTMLNames.h"
#include "MappedAttribute.h"
#include "RenderFieldset.h"
#include "Text.h"
#include "WMLElementFactory.h"
#include "WMLNames.h"

namespace WebCore {

using namespace WMLNames;

WMLFieldSetElement::WMLFieldSetElement(const QualifiedName& tagName, Document* doc)
    : WMLElement(tagName, doc)
{
}

WMLFieldSetElement::~WMLFieldSetElement()
{
}

void WMLFieldSetElement::insertedIntoDocument()
{
    WMLElement::insertedIntoDocument();

    String title = parseValueSubstitutingVariableReferences(getAttribute(HTMLNames::titleAttr));
    if (title.isEmpty())
        return;

    m_insertedLegendElement = WMLElementFactory::createWMLElement(insertedLegendTag, document());

    // Insert <dummyLegend> element, as RenderFieldset expect it to be present
    // to layout it's child text content, when rendering <fieldset> elements
    ExceptionCode ec = 0;
    appendChild(m_insertedLegendElement, ec);
    ASSERT(ec == 0);

    // Create text node holding the 'title' attribute value
    m_insertedLegendElement->appendChild(document()->createTextNode(title), ec);
    ASSERT(ec == 0);
}

void WMLFieldSetElement::removedFromDocument()
{
    m_insertedLegendElement.clear();
    WMLElement::removedFromDocument();
}

RenderObject* WMLFieldSetElement::createRenderer(RenderArena* arena, RenderStyle*)
{
    return new (arena) RenderFieldset(this);
}

}

#endif
