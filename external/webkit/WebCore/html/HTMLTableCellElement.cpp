

#include "config.h"
#include "HTMLTableCellElement.h"

#include "CSSPropertyNames.h"
#include "CSSValueKeywords.h"
#include "HTMLNames.h"
#include "HTMLTableElement.h"
#include "MappedAttribute.h"
#include "RenderTableCell.h"
#ifdef ANDROID_LAYOUT
#include "Document.h"
#include "Frame.h"
#include "Settings.h"
#endif

using std::max;
using std::min;

namespace WebCore {

// Clamp rowspan at 8k to match Firefox.
static const int maxRowspan = 8190;

using namespace HTMLNames;

HTMLTableCellElement::HTMLTableCellElement(const QualifiedName& tagName, Document *doc)
    : HTMLTablePartElement(tagName, doc)
    , _row(-1)
    , _col(-1)
    , rSpan(1)
    , cSpan(1)
    , rowHeight(0)
    , m_solid(false)
{
}

HTMLTableCellElement::~HTMLTableCellElement()
{
}

int HTMLTableCellElement::cellIndex() const
{
    int index = 0;
    for (const Node * node = previousSibling(); node; node = node->previousSibling()) {
        if (node->hasTagName(tdTag) || node->hasTagName(thTag))
            index++;
    }
    
    return index;
}

bool HTMLTableCellElement::mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const
{
    if (attrName == nowrapAttr) {
        result = eUniversal;
        return false;
    }
    
    if (attrName == widthAttr ||
        attrName == heightAttr) {
        result = eCell; // Because of the quirky behavior of ignoring 0 values, cells are special.
        return false;
    }

    return HTMLTablePartElement::mapToEntry(attrName, result);
}

void HTMLTableCellElement::parseMappedAttribute(MappedAttribute *attr)
{
    if (attr->name() == rowspanAttr) {
        rSpan = !attr->isNull() ? attr->value().toInt() : 1;
        rSpan = max(1, min(rSpan, maxRowspan));
        if (renderer() && renderer()->isTableCell())
            toRenderTableCell(renderer())->updateFromElement();
    } else if (attr->name() == colspanAttr) {
        cSpan = !attr->isNull() ? attr->value().toInt() : 1;
        cSpan = max(1, cSpan);
        if (renderer() && renderer()->isTableCell())
            toRenderTableCell(renderer())->updateFromElement();
    } else if (attr->name() == nowrapAttr) {
#ifdef ANDROID_LAYOUT
        if (!(document()->frame()) || document()->frame()->settings()->layoutAlgorithm() != Settings::kLayoutSSR)
#endif
        if (!attr->isNull())
            addCSSProperty(attr, CSSPropertyWhiteSpace, CSSValueWebkitNowrap);
    } else if (attr->name() == widthAttr) {
        if (!attr->value().isEmpty()) {
            int widthInt = attr->value().toInt();
            if (widthInt > 0) // width="0" is ignored for compatibility with WinIE.
                addCSSLength(attr, CSSPropertyWidth, attr->value());
        }
    } else if (attr->name() == heightAttr) {
        if (!attr->value().isEmpty()) {
            int heightInt = attr->value().toInt();
            if (heightInt > 0) // height="0" is ignored for compatibility with WinIE.
                addCSSLength(attr, CSSPropertyHeight, attr->value());
        }
    } else
        HTMLTablePartElement::parseMappedAttribute(attr);
}

// used by table cells to share style decls created by the enclosing table.
void HTMLTableCellElement::additionalAttributeStyleDecls(Vector<CSSMutableStyleDeclaration*>& results)
{
    Node* p = parentNode();
    while (p && !p->hasTagName(tableTag))
        p = p->parentNode();
    if (!p)
        return;
    static_cast<HTMLTableElement*>(p)->addSharedCellDecls(results);
}

bool HTMLTableCellElement::isURLAttribute(Attribute *attr) const
{
    return attr->name() == backgroundAttr;
}

String HTMLTableCellElement::abbr() const
{
    return getAttribute(abbrAttr);
}

void HTMLTableCellElement::setAbbr(const String &value)
{
    setAttribute(abbrAttr, value);
}

String HTMLTableCellElement::align() const
{
    return getAttribute(alignAttr);
}

void HTMLTableCellElement::setAlign(const String &value)
{
    setAttribute(alignAttr, value);
}

String HTMLTableCellElement::axis() const
{
    return getAttribute(axisAttr);
}

void HTMLTableCellElement::setAxis(const String &value)
{
    setAttribute(axisAttr, value);
}

String HTMLTableCellElement::bgColor() const
{
    return getAttribute(bgcolorAttr);
}

void HTMLTableCellElement::setBgColor(const String &value)
{
    setAttribute(bgcolorAttr, value);
}

String HTMLTableCellElement::ch() const
{
    return getAttribute(charAttr);
}

void HTMLTableCellElement::setCh(const String &value)
{
    setAttribute(charAttr, value);
}

String HTMLTableCellElement::chOff() const
{
    return getAttribute(charoffAttr);
}

void HTMLTableCellElement::setChOff(const String &value)
{
    setAttribute(charoffAttr, value);
}

void HTMLTableCellElement::setColSpan(int n)
{
    setAttribute(colspanAttr, String::number(n));
}

String HTMLTableCellElement::headers() const
{
    return getAttribute(headersAttr);
}

void HTMLTableCellElement::setHeaders(const String &value)
{
    setAttribute(headersAttr, value);
}

String HTMLTableCellElement::height() const
{
    return getAttribute(heightAttr);
}

void HTMLTableCellElement::setHeight(const String &value)
{
    setAttribute(heightAttr, value);
}

bool HTMLTableCellElement::noWrap() const
{
    return !getAttribute(nowrapAttr).isNull();
}

void HTMLTableCellElement::setNoWrap(bool b)
{
    setAttribute(nowrapAttr, b ? "" : 0);
}

void HTMLTableCellElement::setRowSpan(int n)
{
    setAttribute(rowspanAttr, String::number(n));
}

String HTMLTableCellElement::scope() const
{
    return getAttribute(scopeAttr);
}

void HTMLTableCellElement::setScope(const String &value)
{
    setAttribute(scopeAttr, value);
}

String HTMLTableCellElement::vAlign() const
{
    return getAttribute(valignAttr);
}

void HTMLTableCellElement::setVAlign(const String &value)
{
    setAttribute(valignAttr, value);
}

String HTMLTableCellElement::width() const
{
    return getAttribute(widthAttr);
}

void HTMLTableCellElement::setWidth(const String &value)
{
    setAttribute(widthAttr, value);
}

void HTMLTableCellElement::addSubresourceAttributeURLs(ListHashSet<KURL>& urls) const
{
    HTMLTablePartElement::addSubresourceAttributeURLs(urls);

    addSubresourceURL(urls, document()->completeURL(getAttribute(backgroundAttr)));
}

}
