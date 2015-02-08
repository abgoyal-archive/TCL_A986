

#include "config.h"
#include "SQLValue.h"

namespace WebCore {

SQLValue::SQLValue(const SQLValue& val)
    : m_type(val.m_type)
    , m_number(val.m_number)
    , m_string(val.m_string.threadsafeCopy())
{
}

String SQLValue::string() const
{
    ASSERT(m_type == StringValue);

    // Must return a copy since ref-shared Strings are not thread safe
    return m_string.threadsafeCopy();
}

double SQLValue::number() const
{
    ASSERT(m_type == NumberValue);

    return m_number;
}

}
