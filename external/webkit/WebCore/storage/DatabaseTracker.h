

#ifndef DatabaseTracker_h
#define DatabaseTracker_h

#if ENABLE(DATABASE)

#include "PlatformString.h"
#include "StringHash.h"
#include <wtf/HashMap.h>
#include <wtf/HashSet.h>

#if !PLATFORM(CHROMIUM)
#include "DatabaseDetails.h"
#include "SQLiteDatabase.h"
#include <wtf/OwnPtr.h>
#endif // !PLATFORM(CHROMIUM)

namespace WebCore {

class Database;
class ScriptExecutionContext;
class SecurityOrigin;

struct SecurityOriginHash;

#if !PLATFORM(CHROMIUM)
class DatabaseTrackerClient;
class OriginQuotaManager;

struct SecurityOriginTraits;
#endif // !PLATFORM(CHROMIUM)

class DatabaseTracker : public Noncopyable {
public:
    static DatabaseTracker& tracker();
    // FIXME: Due to workers having multiple threads in a single process sharing
    // a DatabaseTracker, this singleton will have to be synchronized or moved
    // to TLS.

    bool canEstablishDatabase(ScriptExecutionContext*, const String& name, const String& displayName, unsigned long estimatedSize);
    void setDatabaseDetails(SecurityOrigin*, const String& name, const String& displayName, unsigned long estimatedSize);
    String fullPathForDatabase(SecurityOrigin*, const String& name, bool createIfDoesNotExist = true);

    void addOpenDatabase(Database*);
    void removeOpenDatabase(Database*);
    void getOpenDatabases(SecurityOrigin* origin, const String& name, HashSet<RefPtr<Database> >* databases);

    unsigned long long getMaxSizeForDatabase(const Database*);

private:
    DatabaseTracker();

    typedef HashSet<Database*> DatabaseSet;
    typedef HashMap<String, DatabaseSet*> DatabaseNameMap;
    typedef HashMap<RefPtr<SecurityOrigin>, DatabaseNameMap*, SecurityOriginHash> DatabaseOriginMap;

    Mutex m_openDatabaseMapGuard;
    mutable OwnPtr<DatabaseOriginMap> m_openDatabaseMap;

#if !PLATFORM(CHROMIUM)
public:
    void setDatabaseDirectoryPath(const String&);
    const String& databaseDirectoryPath() const;

    void origins(Vector<RefPtr<SecurityOrigin> >& result);
    bool databaseNamesForOrigin(SecurityOrigin*, Vector<String>& result);

    DatabaseDetails detailsForNameAndOrigin(const String&, SecurityOrigin*);

    unsigned long long usageForDatabase(const String&, SecurityOrigin*);
    unsigned long long usageForOrigin(SecurityOrigin*);
    unsigned long long quotaForOrigin(SecurityOrigin*);
    void setQuota(SecurityOrigin*, unsigned long long);

    void deleteAllDatabases();
    void deleteOrigin(SecurityOrigin*);
    void deleteDatabase(SecurityOrigin*, const String& name);

    void setClient(DatabaseTrackerClient*);

    // From a secondary thread, must be thread safe with its data
    void scheduleNotifyDatabaseChanged(SecurityOrigin*, const String& name);

    OriginQuotaManager& originQuotaManager();


    bool hasEntryForOrigin(SecurityOrigin*);

private:
    String trackerDatabasePath() const;
    void openTrackerDatabase(bool createIfDoesNotExist);

    String originPath(SecurityOrigin*) const;

    bool hasEntryForDatabase(SecurityOrigin*, const String& databaseIdentifier);

    bool addDatabase(SecurityOrigin*, const String& name, const String& path);
    void populateOrigins();

    bool deleteDatabaseFile(SecurityOrigin*, const String& name);

    SQLiteDatabase m_database;

    typedef HashMap<RefPtr<SecurityOrigin>, unsigned long long, SecurityOriginHash> QuotaMap;
    Mutex m_quotaMapGuard;
    mutable OwnPtr<QuotaMap> m_quotaMap;

    OwnPtr<OriginQuotaManager> m_quotaManager;

    String m_databaseDirectoryPath;

    DatabaseTrackerClient* m_client;

    std::pair<SecurityOrigin*, DatabaseDetails>* m_proposedDatabase;

#ifndef NDEBUG
    ThreadIdentifier m_thread;
#endif

    static void scheduleForNotification();
    static void notifyDatabasesChanged(void*);
#endif // !PLATFORM(CHROMIUM)
};

} // namespace WebCore

#endif // ENABLE(DATABASE)
#endif // DatabaseTracker_h
