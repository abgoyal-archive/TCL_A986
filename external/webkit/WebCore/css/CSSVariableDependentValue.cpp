

#include "config.h"
#include "CSSVariableDependentValue.h"

#include "CSSValueList.h"

namespace WebCore {

CSSVariableDependentValue::CSSVariableDependentValue(PassRefPtr<CSSValueList> list)
: m_list(list)
{
}

CSSVariableDependentValue::~CSSVariableDependentValue()
{
}

String CSSVariableDependentValue::cssText() const
{
    if (m_list)
        return m_list->cssText();
    return "";
}

}
