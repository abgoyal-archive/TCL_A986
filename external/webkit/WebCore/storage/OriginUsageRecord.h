
#ifndef OriginUsageRecord_h
#define OriginUsageRecord_h

#if ENABLE(DATABASE)

#include "PlatformString.h"
#include "StringHash.h"

#include <wtf/HashMap.h>
#include <wtf/HashSet.h>

namespace WebCore {

// Objects of this class can be used from multiple threads with external synchronization.
// String arguments are also supposed to be deeply copied by the caller when necessary.
class OriginUsageRecord : public Noncopyable {
public:
    OriginUsageRecord();

    void addDatabase(const String& identifier, const String& fullPath);
    void removeDatabase(const String& identifier);
    void markDatabase(const String& identifier); // Size may have changed, and will need to be recalculated.
    unsigned long long diskUsage();

private:
    struct DatabaseEntry {
        DatabaseEntry() { }
        DatabaseEntry(const String& filename) : filename(filename) { }
        DatabaseEntry(const String& filename, unsigned long long size) : filename(filename), size(size) { }
        String filename;
        unsigned long long size; // This member remains uninitialized until disk usage is calculated.
    };
    HashMap<String, DatabaseEntry> m_databaseMap;
    HashSet<String> m_unknownSet;

    unsigned long long m_cachedDiskUsage;
    bool m_cachedDiskUsageIsValid;
};

} // namespace WebCore

#endif

#endif
