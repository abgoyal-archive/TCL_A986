
#ifndef DatabaseTask_h
#define DatabaseTask_h

#if ENABLE(DATABASE)
#include "ExceptionCode.h"
#include "PlatformString.h"
#include <wtf/OwnPtr.h>
#include <wtf/PassOwnPtr.h>
#include <wtf/PassRefPtr.h>
#include <wtf/Threading.h>
#include <wtf/Vector.h>

namespace WebCore {

class Database;
class DatabaseTask;
class DatabaseThread;
class SQLValue;
class SQLCallback;
class SQLTransaction;
class VersionChangeCallback;

// Can be used to wait until DatabaseTask is completed.
// Has to be passed into DatabaseTask::create to be associated with the task.
class DatabaseTaskSynchronizer : public Noncopyable {
public:
    DatabaseTaskSynchronizer();

    // Called from main thread to wait until task is completed.
    void waitForTaskCompletion();

    // Called by the task.
    void taskCompleted();
private:

    bool m_taskCompleted;
    Mutex m_synchronousMutex;
    ThreadCondition m_synchronousCondition;
};

class DatabaseTask : public Noncopyable {
    friend class Database;
public:
    virtual ~DatabaseTask();

    void performTask();

    Database* database() const { return m_database; }

protected:
    DatabaseTask(Database*, DatabaseTaskSynchronizer*);

private:
    virtual void doPerformTask() = 0;

    Database* m_database;
    DatabaseTaskSynchronizer* m_synchronizer;

#ifndef NDEBUG
     virtual const char* debugTaskName() const = 0;
     bool m_complete;
#endif
};

class DatabaseOpenTask : public DatabaseTask {
public:
    static PassOwnPtr<DatabaseOpenTask> create(Database* db, DatabaseTaskSynchronizer* synchronizer, ExceptionCode& code, bool& success)
    {
        return new DatabaseOpenTask(db, synchronizer, code, success);
    }

private:
    DatabaseOpenTask(Database*, DatabaseTaskSynchronizer*, ExceptionCode&, bool& success);

    virtual void doPerformTask();
#ifndef NDEBUG
    virtual const char* debugTaskName() const;
#endif

    ExceptionCode& m_code;
    bool& m_success;
};

class DatabaseCloseTask : public DatabaseTask {
public:
    static PassOwnPtr<DatabaseCloseTask> create(Database* db, DatabaseTaskSynchronizer* synchronizer)
    { 
        return new DatabaseCloseTask(db, synchronizer);
    }

private:
    DatabaseCloseTask(Database*, DatabaseTaskSynchronizer*);

    virtual void doPerformTask();
#ifndef NDEBUG
    virtual const char* debugTaskName() const;
#endif
};

class DatabaseTransactionTask : public DatabaseTask {
public:
    // Transaction task is never synchronous, so no 'synchronizer' parameter.
    static PassOwnPtr<DatabaseTransactionTask> create(PassRefPtr<SQLTransaction> transaction)
    {
        return new DatabaseTransactionTask(transaction);
    }

    SQLTransaction* transaction() const { return m_transaction.get(); }

    virtual ~DatabaseTransactionTask();
private:
    DatabaseTransactionTask(PassRefPtr<SQLTransaction>);

    virtual void doPerformTask();
#ifndef NDEBUG
    virtual const char* debugTaskName() const;
#endif

    RefPtr<SQLTransaction> m_transaction;
};

class DatabaseTableNamesTask : public DatabaseTask {
public:
    static PassOwnPtr<DatabaseTableNamesTask> create(Database* db, DatabaseTaskSynchronizer* synchronizer, Vector<String>& names)
    {
        return new DatabaseTableNamesTask(db, synchronizer, names);
    }

private:
    DatabaseTableNamesTask(Database*, DatabaseTaskSynchronizer*, Vector<String>& names);

    virtual void doPerformTask();
#ifndef NDEBUG
    virtual const char* debugTaskName() const;
#endif

    Vector<String>& m_tableNames;
};

} // namespace WebCore

#endif // ENABLE(DATABASE)
#endif // DatabaseTask_h
