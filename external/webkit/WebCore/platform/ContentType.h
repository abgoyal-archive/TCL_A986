

#ifndef ContentType_h
#define ContentType_h

#include "PlatformString.h"

namespace WebCore {

    class ContentType {
    public:
        ContentType(const String& type);

        String parameter (const String& parameterName) const;
        String type() const;
        const String& raw() const { return m_type; }
    private:
        String m_type;
    };

} // namespace WebCore

#endif // ContentType_h
