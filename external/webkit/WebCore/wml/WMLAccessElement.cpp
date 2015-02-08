

#include "config.h"

#if ENABLE(WML)
#include "WMLAccessElement.h"

#include "MappedAttribute.h"
#include "WMLDocument.h"
#include "WMLNames.h"
#include "WMLVariables.h"

namespace WebCore {

using namespace WMLNames;

WMLAccessElement::WMLAccessElement(const QualifiedName& tagName, Document* doc)
    : WMLElement(tagName, doc)
{
}

void WMLAccessElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == domainAttr) {
        String value = parseValueForbiddingVariableReferences(attr->value());
        if (value.isEmpty())
            return;

        m_domain = value;
    } else if (attr->name() == pathAttr) {
        String value = parseValueForbiddingVariableReferences(attr->value());
        if (value.isEmpty())
            return;

        m_path = value;
    } else
        WMLElement::parseMappedAttribute(attr);
}

void WMLAccessElement::insertedIntoDocument()
{
    WMLElement::insertedIntoDocument();

    WMLPageState* pageState = wmlPageStateForDocument(document());
    if (!pageState || pageState->processAccessControlData(m_domain, m_path))
        return;

    pageState->resetAccessControlData();
    reportWMLError(document(), WMLErrorMultipleAccessElements);
}

}

#endif
