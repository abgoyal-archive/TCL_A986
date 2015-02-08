

#include "config.h"
#include "HTMLParamElement.h"

#include "Document.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"

namespace WebCore {

using namespace HTMLNames;

HTMLParamElement::HTMLParamElement(const QualifiedName& tagName, Document* doc)
    : HTMLElement(tagName, doc)
{
    ASSERT(hasTagName(paramTag));
}

HTMLParamElement::~HTMLParamElement()
{
}

void HTMLParamElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == idAttributeName()) {
        // Must call base class so that hasID bit gets set.
        HTMLElement::parseMappedAttribute(attr);
        if (document()->isHTMLDocument())
            return;
        m_name = attr->value();
    } else if (attr->name() == nameAttr) {
        m_name = attr->value();
    } else if (attr->name() == valueAttr) {
        m_value = attr->value();
    } else
        HTMLElement::parseMappedAttribute(attr);
}

bool HTMLParamElement::isURLAttribute(Attribute* attr) const
{
    if (attr->name() == valueAttr) {
        Attribute* attr = attributes()->getAttributeItem(nameAttr);
        if (attr) {
            const AtomicString& value = attr->value();
            if (equalIgnoringCase(value, "data") || equalIgnoringCase(value, "movie") || equalIgnoringCase(value, "src"))
                return true;
        }
    }
    return false;
}

void HTMLParamElement::setName(const String& value)
{
    setAttribute(nameAttr, value);
}

String HTMLParamElement::type() const
{
    return getAttribute(typeAttr);
}

void HTMLParamElement::setType(const String& value)
{
    setAttribute(typeAttr, value);
}

void HTMLParamElement::setValue(const String& value)
{
    setAttribute(valueAttr, value);
}

String HTMLParamElement::valueType() const
{
    return getAttribute(valuetypeAttr);
}

void HTMLParamElement::setValueType(const String& value)
{
    setAttribute(valuetypeAttr, value);
}

void HTMLParamElement::addSubresourceAttributeURLs(ListHashSet<KURL>& urls) const
{
    HTMLElement::addSubresourceAttributeURLs(urls);

    if (!equalIgnoringCase(name(), "data") &&
        !equalIgnoringCase(name(), "movie") &&
        !equalIgnoringCase(name(), "src"))
        return;
    
    addSubresourceURL(urls, document()->completeURL(value()));
}

}
