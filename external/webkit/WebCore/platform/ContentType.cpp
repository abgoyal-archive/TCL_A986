

#include "config.h"
#include "ContentType.h"

namespace WebCore {

ContentType::ContentType(const String& contentType)
    : m_type(contentType)
{
}

String ContentType::parameter(const String& parameterName) const
{
    String parameterValue;
    String strippedType = m_type.stripWhiteSpace();

    // a MIME type can have one or more "param=value" after a semi-colon, and separated from each other by semi-colons
    int semi = strippedType.find(';');
    if (semi != -1) {
        int start = strippedType.find(parameterName, semi + 1, false);
        if (start != -1) {
            start = strippedType.find('=', start + 6);
            if (start != -1) {
                int end = strippedType.find(';', start + 6);
                if (end == -1)
                    end = strippedType.length();
                parameterValue = strippedType.substring(start + 1, end - (start + 1)).stripWhiteSpace();
            }
        }
    }

    return parameterValue;
}

String ContentType::type() const
{
    String strippedType = m_type.stripWhiteSpace();

    // "type" can have parameters after a semi-colon, strip them
    int semi = strippedType.find(';');
    if (semi != -1)
        strippedType = strippedType.left(semi).stripWhiteSpace();

    return strippedType;
}

} // namespace WebCore
