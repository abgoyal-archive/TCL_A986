

#ifndef DatabaseDetails_h
#define DatabaseDetails_h

#if ENABLE(DATABASE)

#include "PlatformString.h"

namespace WebCore {

class DatabaseDetails {
public:
    DatabaseDetails()
        : m_expectedUsage(0)
        , m_currentUsage(0)
    {
    }

    DatabaseDetails(const String& databaseName, const String& displayName, unsigned long long expectedUsage, unsigned long long currentUsage)
        : m_name(databaseName)
        , m_displayName(displayName)
        , m_expectedUsage(expectedUsage)
        , m_currentUsage(currentUsage)
    {
    }

    const String& name() const { return m_name; }
    const String& displayName() const { return m_displayName; }
    unsigned long long expectedUsage() const { return m_expectedUsage; }
    unsigned long long currentUsage() const { return m_currentUsage; }

private:
    String m_name;
    String m_displayName;
    unsigned long long m_expectedUsage;
    unsigned long long m_currentUsage;

};

} // namespace WebCore

#endif

#endif // DatabaseDetails_h
