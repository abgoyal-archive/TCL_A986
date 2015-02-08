

#include "config.h"
#include "XPathValue.h"

#if ENABLE(XPATH)

#include "Node.h"
#include "XPathExpressionNode.h"
#include "XPathUtil.h"
#include <limits>
#include <wtf/MathExtras.h>
#include <wtf/StdLibExtras.h>

using std::numeric_limits;

namespace WebCore {
namespace XPath {

const Value::AdoptTag Value::adopt = {};

const NodeSet& Value::toNodeSet() const
{
    if (!isNodeSet())
        Expression::evaluationContext().hadTypeConversionError = true;

    if (!m_data) {
        DEFINE_STATIC_LOCAL(NodeSet, emptyNodeSet, ());
        return emptyNodeSet;
    }

    return m_data->m_nodeSet;
}    

NodeSet& Value::modifiableNodeSet()
{
    if (!isNodeSet())
        Expression::evaluationContext().hadTypeConversionError = true;

    if (!m_data)
        m_data = ValueData::create();
    
    m_type = NodeSetValue;
    return m_data->m_nodeSet;
}

bool Value::toBoolean() const
{
    switch (m_type) {
        case NodeSetValue:
            return !m_data->m_nodeSet.isEmpty();
        case BooleanValue:
            return m_bool;
        case NumberValue:
            return m_number != 0 && !isnan(m_number);
        case StringValue:
            return !m_data->m_string.isEmpty();
    }
    ASSERT_NOT_REACHED();
    return false;
}

double Value::toNumber() const
{
    switch (m_type) {
        case NodeSetValue:
            return Value(toString()).toNumber();
        case NumberValue:
            return m_number;
        case StringValue: {
            const String& str = m_data->m_string.simplifyWhiteSpace();

            // String::toDouble() supports exponential notation, which is not allowed in XPath.
            unsigned len = str.length();
            for (unsigned i = 0; i < len; ++i) {
                UChar c = str[i];
                if (!isASCIIDigit(c) && c != '.'  && c != '-')
                    return numeric_limits<double>::quiet_NaN();
            }

            bool canConvert;
            double value = str.toDouble(&canConvert);
            if (canConvert)
                return value;
            return numeric_limits<double>::quiet_NaN();
        }
        case BooleanValue:
            return m_bool;
    }
    ASSERT_NOT_REACHED();
    return 0.0;
}

String Value::toString() const
{
    switch (m_type) {
        case NodeSetValue:
            if (m_data->m_nodeSet.isEmpty()) 
                return "";
            return stringValue(m_data->m_nodeSet.firstNode());
        case StringValue:
            return m_data->m_string;
        case NumberValue:
            if (isnan(m_number))
                return "NaN";
            if (m_number == 0)
                return "0";
            if (isinf(m_number))
                return signbit(m_number) ? "-Infinity" : "Infinity";
            return String::number(m_number);
        case BooleanValue:
            return m_bool ? "true" : "false";
    }
    ASSERT_NOT_REACHED();
    return String();
}

}
}

#endif // ENABLE(XPATH)
