

#ifndef DatabaseThreadWince_h
#define DatabaseThreadWince_h

#include <wtf/Deque.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    class Database;
    class DatabaseTask;

    class DatabaseThread: public WTF::RefCounted<DatabaseThread> {

    public:
        static PassRefPtr<DatabaseThread> create() { return adoptRef(new DatabaseThread); }
        ~DatabaseThread();

        bool start() { return true; }
        void requestTermination();
        bool terminationRequested() const;

        void scheduleTask(PassRefPtr<DatabaseTask>);
        void scheduleImmediateTask(PassRefPtr<DatabaseTask>);
        void unscheduleDatabaseTasks(Database*);
        void recordDatabaseOpen(Database*);
        void recordDatabaseClosed(Database*);
#ifndef NDEBUG
        ThreadIdentifier getThreadID() const { return currentThread(); }
#endif

    private:
        DatabaseThread();

        void timerFired(Timer<DatabaseThread>*);

        Deque<RefPtr<DatabaseTask> > m_queue;
        Timer<DatabaseThread> m_timer;
    };

} // namespace WebCore

#endif // DatabaseThreadWince
