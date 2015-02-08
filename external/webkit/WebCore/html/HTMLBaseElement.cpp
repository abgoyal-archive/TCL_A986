

#include "config.h"
#include "HTMLBaseElement.h"

#include "CSSHelper.h"
#include "Document.h"
#include "Frame.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"
#include "XSSAuditor.h"

namespace WebCore {

using namespace HTMLNames;

HTMLBaseElement::HTMLBaseElement(const QualifiedName& qName, Document* document)
    : HTMLElement(qName, document)
{
    ASSERT(hasTagName(baseTag));
}

void HTMLBaseElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == hrefAttr) {
        m_hrefAttrValue = attr->value();
        m_href = deprecatedParseURL(attr->value());
        process();
    } else if (attr->name() == targetAttr) {
        m_target = attr->value();
        process();
    } else
        HTMLElement::parseMappedAttribute(attr);
}

void HTMLBaseElement::insertedIntoDocument()
{
    HTMLElement::insertedIntoDocument();
    process();
}

void HTMLBaseElement::removedFromDocument()
{
    HTMLElement::removedFromDocument();

    // Since the document doesn't have a base element, clear the base URL and target.
    // FIXME: This does not handle the case of multiple base elements correctly.
    document()->setBaseElementURL(KURL());
    document()->setBaseElementTarget(String());
}

void HTMLBaseElement::process()
{
    if (!inDocument())
        return;

    if (!m_href.isEmpty() && (!document()->frame() || document()->frame()->script()->xssAuditor()->canSetBaseElementURL(m_hrefAttrValue)))
        document()->setBaseElementURL(KURL(document()->url(), m_href));

    if (!m_target.isEmpty())
        document()->setBaseElementTarget(m_target);

    // FIXME: Changing a document's base URL should probably automatically update the resolved relative URLs of all images, stylesheets, etc.
}

}
