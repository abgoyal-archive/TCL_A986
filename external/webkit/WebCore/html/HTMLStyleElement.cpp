

#include "config.h"
#include "HTMLStyleElement.h"

#include "Document.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"

namespace WebCore {

using namespace HTMLNames;

HTMLStyleElement::HTMLStyleElement(const QualifiedName& tagName, Document* doc, bool createdByParser)
    : HTMLElement(tagName, doc)
    , m_loading(false)
    , m_createdByParser(createdByParser)
{
    ASSERT(hasTagName(styleTag));
}

// other stuff...
void HTMLStyleElement::parseMappedAttribute(MappedAttribute *attr)
{
    if (attr->name() == mediaAttr)
        m_media = attr->value().string().lower();
    else if (attr->name() == titleAttr && m_sheet)
        m_sheet->setTitle(attr->value());
     else
        HTMLElement::parseMappedAttribute(attr);
}

void HTMLStyleElement::finishParsingChildren()
{
    StyleElement::process(this);
    StyleElement::sheet(this);
    m_createdByParser = false;
    HTMLElement::finishParsingChildren();
}

void HTMLStyleElement::insertedIntoDocument()
{
    HTMLElement::insertedIntoDocument();

    document()->addStyleSheetCandidateNode(this, m_createdByParser);
    if (!m_createdByParser)
        StyleElement::insertedIntoDocument(document(), this);
}

void HTMLStyleElement::removedFromDocument()
{
    HTMLElement::removedFromDocument();
    document()->removeStyleSheetCandidateNode(this);
    StyleElement::removedFromDocument(document());
}

void HTMLStyleElement::childrenChanged(bool changedByParser, Node* beforeChange, Node* afterChange, int childCountDelta)
{
    if (!changedByParser)
        StyleElement::process(this);
    HTMLElement::childrenChanged(changedByParser, beforeChange, afterChange, childCountDelta);
}

StyleSheet* HTMLStyleElement::sheet()
{
    return StyleElement::sheet(this);
}

bool HTMLStyleElement::isLoading() const
{
    if (m_loading)
        return true;
    if (!m_sheet)
        return false;
    return static_cast<CSSStyleSheet *>(m_sheet.get())->isLoading();
}

bool HTMLStyleElement::sheetLoaded()
{
    if (!isLoading()) {
        document()->removePendingSheet();
        return true;
    }
    return false;
}

bool HTMLStyleElement::disabled() const
{
    return !getAttribute(disabledAttr).isNull();
}

void HTMLStyleElement::setDisabled(bool disabled)
{
    setAttribute(disabledAttr, disabled ? "" : 0);
}

const AtomicString& HTMLStyleElement::media() const
{
    return getAttribute(mediaAttr);
}

void HTMLStyleElement::setMedia(const AtomicString &value)
{
    setAttribute(mediaAttr, value);
}

const AtomicString& HTMLStyleElement::type() const
{
    return getAttribute(typeAttr);
}

void HTMLStyleElement::setType(const AtomicString &value)
{
    setAttribute(typeAttr, value);
}

void HTMLStyleElement::addSubresourceAttributeURLs(ListHashSet<KURL>& urls) const
{    
    HTMLElement::addSubresourceAttributeURLs(urls);

    if (StyleSheet* styleSheet = const_cast<HTMLStyleElement*>(this)->sheet())
        styleSheet->addSubresourceStyleURLs(urls);
}

}
