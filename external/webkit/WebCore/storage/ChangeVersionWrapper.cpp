
#include "config.h"
#include "ChangeVersionWrapper.h"

#if ENABLE(DATABASE)
#include "Database.h"

namespace WebCore {

ChangeVersionWrapper::ChangeVersionWrapper(const String& oldVersion, const String& newVersion)
    : m_oldVersion(oldVersion.crossThreadString())
    , m_newVersion(newVersion.crossThreadString())
{
}

bool ChangeVersionWrapper::performPreflight(SQLTransaction* transaction)
{
    ASSERT(transaction && transaction->database());

    String actualVersion;

    if (!transaction->database()->getVersionFromDatabase(actualVersion)) {
        LOG_ERROR("Unable to retrieve actual current version from database");
        m_sqlError = SQLError::create(0, "unable to verify current version of database");
        return false;
    }

    if (actualVersion != m_oldVersion) {
        LOG_ERROR("Old version doesn't match actual version");
        m_sqlError = SQLError::create(2, "current version of the database and `oldVersion` argument do not match");
        return false;
    }

    return true;
}

bool ChangeVersionWrapper::performPostflight(SQLTransaction* transaction)
{
    ASSERT(transaction && transaction->database());

    if (!transaction->database()->setVersionInDatabase(m_newVersion)) {
        LOG_ERROR("Unable to set new version in database");
        m_sqlError = SQLError::create(0, "unable to set new version in database");
        return false;
    }

    transaction->database()->setExpectedVersion(m_newVersion);

    return true;
}

} // namespace WebCore

#endif // ENABLE(DATABASE)
