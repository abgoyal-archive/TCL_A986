

#include "config.h"
#include "CSSFunctionValue.h"
#include "CSSValueList.h"

namespace WebCore {

CSSFunctionValue::CSSFunctionValue(CSSParserFunction* function)
{
    m_name = function->name;
    if (function->args)
        m_args = CSSValueList::createFromParserValueList(function->args);
}

CSSFunctionValue::~CSSFunctionValue()
{
}

String CSSFunctionValue::cssText() const
{
    String result = m_name; // Includes the '('
    if (m_args)
        result += m_args->cssText();
    result += ")";
    return result;
}

CSSParserValue CSSFunctionValue::parserValue() const
{
    CSSParserValue val;
    val.id = 0;
    val.isInt = false;
    val.unit = CSSParserValue::Function;
    val.function = new CSSParserFunction;
    val.function->name.characters = const_cast<UChar*>(m_name.characters());
    val.function->name.length = m_name.length();
    val.function->args = m_args ? m_args->createParserValueList() : 0;
    return val;
}

}
