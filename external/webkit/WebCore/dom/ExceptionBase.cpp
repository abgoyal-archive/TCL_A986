

#include "config.h"
#include "ExceptionBase.h"

namespace WebCore {

ExceptionBase::ExceptionBase(const ExceptionCodeDescription& description)
    : m_code(description.code)
    , m_name(description.name)
    , m_description(description.description)
{
    if (description.name)
        m_message = String::format("%s: %s Exception %d", description.name, description.typeName, description.code);
    else
        m_message = String::format("%s Exception %d", description.typeName, description.code);
}

String ExceptionBase::toString() const
{
    return "Error: " + m_message;
}

} // namespace WebCore
