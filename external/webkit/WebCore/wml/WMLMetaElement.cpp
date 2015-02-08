

#include "config.h"

#if ENABLE(WML)
#include "WMLMetaElement.h"

#include "Document.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"

namespace WebCore {

WMLMetaElement::WMLMetaElement(const QualifiedName& tagName, Document* doc)
    : WMLElement(tagName, doc)
{
}

WMLMetaElement::~WMLMetaElement()
{
}

void WMLMetaElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == HTMLNames::http_equivAttr)
        m_equiv = parseValueForbiddingVariableReferences(attr->value());
    else if (attr->name() == HTMLNames::contentAttr)
        m_content = parseValueForbiddingVariableReferences(attr->value());
    else if (attr->name() == HTMLNames::nameAttr) {
        // FIXME: The user agent must ignore any meta-data named with this attribute.
    } else
        WMLElement::parseMappedAttribute(attr);
}

void WMLMetaElement::insertedIntoDocument()
{
    WMLElement::insertedIntoDocument();

    if (m_equiv.isNull() || m_content.isNull())
        return;

    document()->processHttpEquiv(m_equiv, m_content);
}

}

#endif
