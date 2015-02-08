

#include "config.h"
#include "CSSProperty.h"

#include "CSSPropertyNames.h"
#include "PlatformString.h"

namespace WebCore {

String CSSProperty::cssText() const
{
    if (id() == CSSPropertyWebkitVariableDeclarationBlock)
        return m_value->cssText() + ";";
    return String(getPropertyName(static_cast<CSSPropertyID>(id()))) + ": " + m_value->cssText() + (isImportant() ? " !important" : "") + "; ";
}

bool operator==(const CSSProperty& a, const CSSProperty& b)
{
    return a.m_id == b.m_id && a.m_important == b.m_important && a.m_value == b.m_value;
}

} // namespace WebCore
