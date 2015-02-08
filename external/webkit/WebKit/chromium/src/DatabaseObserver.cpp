

#include "config.h"
#include "DatabaseObserver.h"

#include "Database.h"
#include "WebDatabase.h"
#include "WebDatabaseObserver.h"

using namespace WebKit;

namespace WebCore {

void DatabaseObserver::databaseOpened(Database* database)
{
    ASSERT(isMainThread());
    WebDatabase::observer()->databaseOpened(WebDatabase(database));
}

void DatabaseObserver::databaseModified(Database* database)
{
    ASSERT(isMainThread());
    WebDatabase::observer()->databaseModified(WebDatabase(database));
}

void DatabaseObserver::databaseClosed(Database* database)
{
    ASSERT(isMainThread());
    WebDatabase::observer()->databaseClosed(WebDatabase(database));
}

} // namespace WebCore
