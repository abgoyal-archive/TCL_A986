

#ifndef WorkerRunLoop_h
#define WorkerRunLoop_h

#if ENABLE(WORKERS)

#include "ScriptExecutionContext.h"
#include <wtf/MessageQueue.h>
#include <wtf/OwnPtr.h>
#include <wtf/PassOwnPtr.h>

namespace WebCore {

    class ModePredicate;
    class WorkerContext;
    class WorkerSharedTimer;

    class WorkerRunLoop {
    public:
        WorkerRunLoop();
        ~WorkerRunLoop();
        
        // Blocking call. Waits for tasks and timers, invokes the callbacks.
        void run(WorkerContext*);

        // Waits for a single task and returns.
        MessageQueueWaitResult runInMode(WorkerContext*, const String& mode);

        void terminate();
        bool terminated() { return m_messageQueue.killed(); }

        void postTask(PassOwnPtr<ScriptExecutionContext::Task>);
        void postTaskForMode(PassOwnPtr<ScriptExecutionContext::Task>, const String& mode);

        unsigned long createUniqueId() { return ++m_uniqueId; }

        static String defaultMode();

        class Task : public Noncopyable {
        public:
            static PassOwnPtr<Task> create(PassOwnPtr<ScriptExecutionContext::Task> task, const String& mode);
            ~Task() { }
            const String& mode() const { return m_mode; }
            void performTask(ScriptExecutionContext* context);

        private:
            Task(PassOwnPtr<ScriptExecutionContext::Task> task, const String& mode);
        
            OwnPtr<ScriptExecutionContext::Task> m_task;
            String m_mode;
        };

    private:
        friend class RunLoopSetup;
        MessageQueueWaitResult runInMode(WorkerContext*, const ModePredicate&);

        MessageQueue<Task> m_messageQueue;
        OwnPtr<WorkerSharedTimer> m_sharedTimer;
        int m_nestedCount;
        unsigned long m_uniqueId;
    };

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // WorkerRunLoop_h
