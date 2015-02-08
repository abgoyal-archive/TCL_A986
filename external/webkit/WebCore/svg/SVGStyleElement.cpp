

#include "config.h"

#if ENABLE(SVG)
#include "SVGStyleElement.h"

#include "CSSStyleSheet.h"
#include "Document.h"
#include "ExceptionCode.h"
#include "MappedAttribute.h"
#include "SVGNames.h"
#include <wtf/StdLibExtras.h>

namespace WebCore {

using namespace SVGNames;

SVGStyleElement::SVGStyleElement(const QualifiedName& tagName, Document* doc, bool createdByParser)
     : SVGElement(tagName, doc)
     , SVGLangSpace()
     , m_createdByParser(createdByParser)
{
}

const AtomicString& SVGStyleElement::type() const
{
    DEFINE_STATIC_LOCAL(const AtomicString, defaultValue, ("text/css"));
    const AtomicString& n = getAttribute(SVGNames::typeAttr);
    return n.isNull() ? defaultValue : n;
}

void SVGStyleElement::setType(const AtomicString& type, ExceptionCode& ec)
{
    setAttribute(SVGNames::typeAttr, type, ec);
}

const AtomicString& SVGStyleElement::media() const
{
    DEFINE_STATIC_LOCAL(const AtomicString, defaultValue, ("all"));
    const AtomicString& n = getAttribute(SVGNames::mediaAttr);
    return n.isNull() ? defaultValue : n;
}

void SVGStyleElement::setMedia(const AtomicString& media, ExceptionCode& ec)
{
    setAttribute(SVGNames::mediaAttr, media, ec);
}

String SVGStyleElement::title() const
{
    return getAttribute(SVGNames::titleAttr);
}

void SVGStyleElement::setTitle(const AtomicString& title, ExceptionCode& ec)
{
    setAttribute(SVGNames::titleAttr, title, ec);
}

void SVGStyleElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == SVGNames::titleAttr && m_sheet)
        m_sheet->setTitle(attr->value());
    else {
        if (SVGLangSpace::parseMappedAttribute(attr))
            return;
        SVGElement::parseMappedAttribute(attr);
    }
}

void SVGStyleElement::finishParsingChildren()
{
    StyleElement::sheet(this);
    m_createdByParser = false;
    SVGElement::finishParsingChildren();
}

void SVGStyleElement::insertedIntoDocument()
{
    SVGElement::insertedIntoDocument();
    document()->addStyleSheetCandidateNode(this, m_createdByParser);
    if (!m_createdByParser)
        StyleElement::insertedIntoDocument(document(), this);
}

void SVGStyleElement::removedFromDocument()
{
    SVGElement::removedFromDocument();
    if (document()->renderer())
        document()->removeStyleSheetCandidateNode(this);
    StyleElement::removedFromDocument(document());
}

void SVGStyleElement::childrenChanged(bool changedByParser, Node* beforeChange, Node* afterChange, int childCountDelta)
{
    SVGElement::childrenChanged(changedByParser, beforeChange, afterChange, childCountDelta);
    StyleElement::process(this);
}

StyleSheet* SVGStyleElement::sheet()
{
    return StyleElement::sheet(this);
}

bool SVGStyleElement::sheetLoaded()
{
    document()->removePendingSheet();
    return true;
}

}

// vim:ts=4:noet
#endif // ENABLE(SVG)
