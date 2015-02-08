
#ifndef DatabaseThread_h
#define DatabaseThread_h

#if ENABLE(DATABASE)
#include <wtf/Deque.h>
#include <wtf/HashMap.h>
#include <wtf/HashSet.h>
#include <wtf/MessageQueue.h>
#include <wtf/OwnPtr.h>
#include <wtf/PassOwnPtr.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>
#include <wtf/Threading.h>

namespace WebCore {

class Database;
class DatabaseTask;
class DatabaseTaskSynchronizer;
class Document;
class SQLTransactionClient;
class SQLTransactionCoordinator;

class DatabaseThread : public ThreadSafeShared<DatabaseThread> {
public:
    static PassRefPtr<DatabaseThread> create() { return adoptRef(new DatabaseThread); }
    ~DatabaseThread();

    bool start();
    void requestTermination(DatabaseTaskSynchronizer* cleanupSync);
    bool terminationRequested() const;

    void scheduleTask(PassOwnPtr<DatabaseTask>);
    void scheduleImmediateTask(PassOwnPtr<DatabaseTask>); // This just adds the task to the front of the queue - the caller needs to be extremely careful not to create deadlocks when waiting for completion.
    void unscheduleDatabaseTasks(Database*);

    void recordDatabaseOpen(Database*);
    void recordDatabaseClosed(Database*);
    ThreadIdentifier getThreadID() { return m_threadID; }

    SQLTransactionClient* transactionClient() { return m_transactionClient.get(); }
    SQLTransactionCoordinator* transactionCoordinator() { return m_transactionCoordinator.get(); }

private:
    DatabaseThread();

    static void* databaseThreadStart(void*);
    void* databaseThread();

    Mutex m_threadCreationMutex;
    ThreadIdentifier m_threadID;
    RefPtr<DatabaseThread> m_selfRef;

    MessageQueue<DatabaseTask> m_queue;

    // This set keeps track of the open databases that have been used on this thread.
    typedef HashSet<RefPtr<Database> > DatabaseSet;
    DatabaseSet m_openDatabaseSet;

    OwnPtr<SQLTransactionClient> m_transactionClient;
    OwnPtr<SQLTransactionCoordinator> m_transactionCoordinator;
    DatabaseTaskSynchronizer* m_cleanupSync;
};

} // namespace WebCore

#endif // ENABLE(DATABASE)
#endif // DatabaseThread_h
