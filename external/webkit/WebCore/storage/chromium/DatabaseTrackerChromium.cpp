

#include "config.h"
#include "DatabaseTracker.h"

#include "CString.h"
#include "Database.h"
#include "DatabaseObserver.h"
#include "DatabaseThread.h"
#include "QuotaTracker.h"
#include "ScriptExecutionContext.h"
#include "SecurityOrigin.h"
#include "SecurityOriginHash.h"
#include "SQLiteFileSystem.h"
#include <wtf/HashSet.h>
#include <wtf/MainThread.h>
#include <wtf/StdLibExtras.h>

namespace WebCore {

DatabaseTracker& DatabaseTracker::tracker()
{
    DEFINE_STATIC_LOCAL(DatabaseTracker, tracker, ());
    return tracker;
}

DatabaseTracker::DatabaseTracker()
{
    SQLiteFileSystem::registerSQLiteVFS();
}

bool DatabaseTracker::canEstablishDatabase(ScriptExecutionContext*, const String&, const String&, unsigned long)
{
    // In Chromium, a database can always be established (even though we might not
    // be able to write anything to it if the quota for this origin was exceeded)
    return true;
}

void DatabaseTracker::setDatabaseDetails(SecurityOrigin*, const String&, const String&, unsigned long)
{
    // Chromium sets the database details when the database is opened
}

String DatabaseTracker::fullPathForDatabase(SecurityOrigin* origin, const String& name, bool)
{
    return origin->databaseIdentifier() + "/" + name + "#";
}

void DatabaseTracker::addOpenDatabase(Database* database)
{
    ASSERT(database->scriptExecutionContext()->isContextThread());
    MutexLocker openDatabaseMapLock(m_openDatabaseMapGuard);
    if (!m_openDatabaseMap)
        m_openDatabaseMap.set(new DatabaseOriginMap());

    DatabaseNameMap* nameMap = m_openDatabaseMap->get(database->securityOrigin());
    if (!nameMap) {
        nameMap = new DatabaseNameMap();
        m_openDatabaseMap->set(database->securityOrigin(), nameMap);
    }

    String name(database->stringIdentifier());
    DatabaseSet* databaseSet = nameMap->get(name);
    if (!databaseSet) {
        databaseSet = new DatabaseSet();
        nameMap->set(name, databaseSet);
    }

    databaseSet->add(database);

    DatabaseObserver::databaseOpened(database);
}

class TrackerRemoveOpenDatabaseTask : public ScriptExecutionContext::Task {
public:
    static PassOwnPtr<TrackerRemoveOpenDatabaseTask> create(PassRefPtr<Database> database)
    {
        return new TrackerRemoveOpenDatabaseTask(database);
    }

    virtual void performTask(ScriptExecutionContext* context)
    {
        DatabaseTracker::tracker().removeOpenDatabase(m_database.get());
    }

private:
    TrackerRemoveOpenDatabaseTask(PassRefPtr<Database> database)
        : m_database(database)
    {
    }

    RefPtr<Database> m_database;
};

void DatabaseTracker::removeOpenDatabase(Database* database)
{
    if (!database->scriptExecutionContext()->isContextThread()) {
        database->scriptExecutionContext()->postTask(TrackerRemoveOpenDatabaseTask::create(database));
        return;
    }

    MutexLocker openDatabaseMapLock(m_openDatabaseMapGuard);
    ASSERT(m_openDatabaseMap);
    DatabaseNameMap* nameMap = m_openDatabaseMap->get(database->securityOrigin());
    ASSERT(nameMap);
    String name(database->stringIdentifier());
    DatabaseSet* databaseSet = nameMap->get(name);
    ASSERT(databaseSet);
    databaseSet->remove(database);

    if (databaseSet->isEmpty()) {
        nameMap->remove(name);
        delete databaseSet;
        if (nameMap->isEmpty()) {
            m_openDatabaseMap->remove(database->securityOrigin());
            delete nameMap;
        }
    }

    DatabaseObserver::databaseClosed(database);
}


void DatabaseTracker::getOpenDatabases(SecurityOrigin* origin, const String& name, HashSet<RefPtr<Database> >* databases)
{
    MutexLocker openDatabaseMapLock(m_openDatabaseMapGuard);
    if (!m_openDatabaseMap)
        return;

    DatabaseNameMap* nameMap = m_openDatabaseMap->get(origin);
    if (!nameMap)
        return;

    DatabaseSet* databaseSet = nameMap->get(name);
    if (!databaseSet)
        return;

    for (DatabaseSet::iterator it = databaseSet->begin(); it != databaseSet->end(); ++it)
        databases->add(*it);
}

unsigned long long DatabaseTracker::getMaxSizeForDatabase(const Database* database)
{
    ASSERT(currentThread() == database->scriptExecutionContext()->databaseThread()->getThreadID());
    unsigned long long spaceAvailable = 0;
    unsigned long long databaseSize = 0;
    QuotaTracker::instance().getDatabaseSizeAndSpaceAvailableToOrigin(
        database->securityOrigin()->databaseIdentifier(),
        database->stringIdentifier(), &databaseSize, &spaceAvailable);
    return databaseSize + spaceAvailable;
}

}
