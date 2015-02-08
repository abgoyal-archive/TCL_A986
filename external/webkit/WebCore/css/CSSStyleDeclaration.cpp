

#include "config.h"
#include "CSSStyleDeclaration.h"

#include "CSSMutableStyleDeclaration.h"
#include "CSSParser.h"
#include "CSSProperty.h"
#include "CSSPropertyNames.h"
#include "CSSRule.h"
#include <wtf/ASCIICType.h>

using namespace WTF;

namespace WebCore {

CSSStyleDeclaration::CSSStyleDeclaration(CSSRule* parent)
    : StyleBase(parent)
{
}

PassRefPtr<CSSValue> CSSStyleDeclaration::getPropertyCSSValue(const String& propertyName)
{
    int propID = cssPropertyID(propertyName);
    if (!propID)
        return 0;
    return getPropertyCSSValue(propID);
}

String CSSStyleDeclaration::getPropertyValue(const String &propertyName)
{
    int propID = cssPropertyID(propertyName);
    if (!propID)
        return String();
    return getPropertyValue(propID);
}

String CSSStyleDeclaration::getPropertyPriority(const String& propertyName)
{
    int propID = cssPropertyID(propertyName);
    if (!propID)
        return String();
    return getPropertyPriority(propID) ? "important" : "";
}

String CSSStyleDeclaration::getPropertyShorthand(const String& propertyName)
{
    int propID = cssPropertyID(propertyName);
    if (!propID)
        return String();
    int shorthandID = getPropertyShorthand(propID);
    if (!shorthandID)
        return String();
    return getPropertyName(static_cast<CSSPropertyID>(shorthandID));
}

bool CSSStyleDeclaration::isPropertyImplicit(const String& propertyName)
{
    int propID = cssPropertyID(propertyName);
    if (!propID)
        return false;
    return isPropertyImplicit(propID);
}

void CSSStyleDeclaration::setProperty(const String& propertyName, const String& value, ExceptionCode& ec)
{
    int important = value.find("!important", 0, false);
    if (important == -1)
        setProperty(propertyName, value, "", ec);
    else
        setProperty(propertyName, value.left(important - 1), "important", ec);
}

void CSSStyleDeclaration::setProperty(const String& propertyName, const String& value, const String& priority, ExceptionCode& ec)
{
    int propID = cssPropertyID(propertyName);
    if (!propID) {
        // FIXME: Should we raise an exception here?
        return;
    }
    bool important = priority.find("important", 0, false) != -1;
    setProperty(propID, value, important, ec);
}

String CSSStyleDeclaration::removeProperty(const String& propertyName, ExceptionCode& ec)
{
    int propID = cssPropertyID(propertyName);
    if (!propID)
        return String();
    return removeProperty(propID, ec);
}

bool CSSStyleDeclaration::isPropertyName(const String& propertyName)
{
    return cssPropertyID(propertyName);
}

CSSRule* CSSStyleDeclaration::parentRule() const
{
    return (parent() && parent()->isRule()) ? static_cast<CSSRule*>(parent()) : 0;
}

bool CSSStyleDeclaration::cssPropertyMatches(const CSSProperty* property) const
{
    RefPtr<CSSValue> value = getPropertyCSSValue(property->id());
    return value && value->cssText() == property->value()->cssText();
}

void CSSStyleDeclaration::diff(CSSMutableStyleDeclaration* style) const
{
    if (!style)
        return;

    Vector<int> propertiesToRemove;
    {
        CSSMutableStyleDeclaration::const_iterator end = style->end();
        for (CSSMutableStyleDeclaration::const_iterator it = style->begin(); it != end; ++it) {
            const CSSProperty& property = *it;
            if (cssPropertyMatches(&property))
                propertiesToRemove.append(property.id());
        }
    }

    // FIXME: This should use mass removal.
    for (unsigned i = 0; i < propertiesToRemove.size(); i++)
        style->removeProperty(propertiesToRemove[i]);
}

PassRefPtr<CSSMutableStyleDeclaration> CSSStyleDeclaration::copyPropertiesInSet(const int* set, unsigned length) const
{
    Vector<CSSProperty> list;
    list.reserveInitialCapacity(length);
    unsigned variableDependentValueCount = 0;
    for (unsigned i = 0; i < length; i++) {
        RefPtr<CSSValue> value = getPropertyCSSValue(set[i]);
        if (value) {
            if (value->isVariableDependentValue())
                variableDependentValueCount++;
            list.append(CSSProperty(set[i], value.release(), false));
        }
    }
    return CSSMutableStyleDeclaration::create(list, variableDependentValueCount);
}

} // namespace WebCore
