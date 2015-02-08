
#include "config.h"
#include "DatabaseTask.h"

#if ENABLE(DATABASE)

#include "Database.h"
#include "Logging.h"

namespace WebCore {

DatabaseTaskSynchronizer::DatabaseTaskSynchronizer()
    : m_taskCompleted(false)
{
}

void DatabaseTaskSynchronizer::waitForTaskCompletion()
{
    m_synchronousMutex.lock();
    if (!m_taskCompleted)
        m_synchronousCondition.wait(m_synchronousMutex);
    m_synchronousMutex.unlock();
}

void DatabaseTaskSynchronizer::taskCompleted()
{
    m_synchronousMutex.lock();
    m_taskCompleted = true;
    m_synchronousCondition.signal();
    m_synchronousMutex.unlock();
}

DatabaseTask::DatabaseTask(Database* database, DatabaseTaskSynchronizer* synchronizer)
    : m_database(database)
    , m_synchronizer(synchronizer)
#ifndef NDEBUG
    , m_complete(false)
#endif
{
}

DatabaseTask::~DatabaseTask()
{
}

void DatabaseTask::performTask()
{
    // Database tasks are meant to be used only once, so make sure this one hasn't been performed before.
    ASSERT(!m_complete);

    LOG(StorageAPI, "Performing %s %p\n", debugTaskName(), this);

    m_database->resetAuthorizer();
    doPerformTask();
    m_database->performPolicyChecks();

    if (m_synchronizer)
        m_synchronizer->taskCompleted();
}

// *** DatabaseOpenTask ***
// Opens the database file and verifies the version matches the expected version.

DatabaseOpenTask::DatabaseOpenTask(Database* database, DatabaseTaskSynchronizer* synchronizer, ExceptionCode& code, bool& success)
    : DatabaseTask(database, synchronizer)
    , m_code(code)
    , m_success(success)
{
    ASSERT(synchronizer); // A task with output parameters is supposed to be synchronous.
}

void DatabaseOpenTask::doPerformTask()
{
    m_success = database()->performOpenAndVerify(m_code);
}

#ifndef NDEBUG
const char* DatabaseOpenTask::debugTaskName() const
{
    return "DatabaseOpenTask";
}
#endif

// *** DatabaseCloseTask ***
// Closes the database.

DatabaseCloseTask::DatabaseCloseTask(Database* database, DatabaseTaskSynchronizer* synchronizer)
    : DatabaseTask(database, synchronizer)
{
}

void DatabaseCloseTask::doPerformTask()
{
    database()->close();
}

#ifndef NDEBUG
const char* DatabaseCloseTask::debugTaskName() const
{
    return "DatabaseCloseTask";
}
#endif

// *** DatabaseTransactionTask ***
// Starts a transaction that will report its results via a callback.

DatabaseTransactionTask::DatabaseTransactionTask(PassRefPtr<SQLTransaction> transaction)
    : DatabaseTask(transaction->database(), 0)
    , m_transaction(transaction)
{
}

DatabaseTransactionTask::~DatabaseTransactionTask()
{
}

void DatabaseTransactionTask::doPerformTask()
{
    if (m_transaction->performNextStep()) {
        // The transaction is complete, we can move on to the next one.
        MutexLocker locker(m_transaction->database()->m_transactionInProgressMutex);
        m_transaction->database()->scheduleTransaction();
    }
}

#ifndef NDEBUG
const char* DatabaseTransactionTask::debugTaskName() const
{
    return "DatabaseTransactionTask";
}
#endif

// *** DatabaseTableNamesTask ***
// Retrieves a list of all tables in the database - for WebInspector support.

DatabaseTableNamesTask::DatabaseTableNamesTask(Database* database, DatabaseTaskSynchronizer* synchronizer, Vector<String>& names)
    : DatabaseTask(database, synchronizer)
    , m_tableNames(names)
{
    ASSERT(synchronizer); // A task with output parameters is supposed to be synchronous.
}

void DatabaseTableNamesTask::doPerformTask()
{
    m_tableNames = database()->performGetTableNames();
}

#ifndef NDEBUG
const char* DatabaseTableNamesTask::debugTaskName() const
{
    return "DatabaseTableNamesTask";
}
#endif

} // namespace WebCore

#endif
