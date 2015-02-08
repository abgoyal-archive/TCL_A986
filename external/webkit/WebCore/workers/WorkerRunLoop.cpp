
 
#include "config.h"

#if ENABLE(WORKERS)

#include "ScriptExecutionContext.h"
#include "SharedTimer.h"
#include "ThreadGlobalData.h"
#include "ThreadTimers.h"
#include "WorkerRunLoop.h"
#include "WorkerContext.h"
#include "WorkerThread.h"

namespace WebCore {

class WorkerSharedTimer : public SharedTimer {
public:
    WorkerSharedTimer()
        : m_sharedTimerFunction(0)
        , m_nextFireTime(0)
    {
    }

    // SharedTimer interface.
    virtual void setFiredFunction(void (*function)()) { m_sharedTimerFunction = function; }
    virtual void setFireTime(double fireTime) { m_nextFireTime = fireTime; }
    virtual void stop() { m_nextFireTime = 0; }

    bool isActive() { return m_sharedTimerFunction && m_nextFireTime; }
    double fireTime() { return m_nextFireTime; }
    void fire() { m_sharedTimerFunction(); }

private:
    void (*m_sharedTimerFunction)();
    double m_nextFireTime;
};

class ModePredicate {
public:
    ModePredicate(const String& mode)
        : m_mode(mode)
        , m_defaultMode(mode == WorkerRunLoop::defaultMode())
    {
    }

    bool isDefaultMode() const
    {
        return m_defaultMode;
    }

    bool operator()(WorkerRunLoop::Task* task) const
    {
        return m_defaultMode || m_mode == task->mode();
    }

private:
    String m_mode;
    bool m_defaultMode;
};

WorkerRunLoop::WorkerRunLoop()
    : m_sharedTimer(new WorkerSharedTimer)
    , m_nestedCount(0)
    , m_uniqueId(0)
{
}

WorkerRunLoop::~WorkerRunLoop()
{
    ASSERT(!m_nestedCount);
}

String WorkerRunLoop::defaultMode()
{
    return String();
}

class RunLoopSetup : public Noncopyable {
public:
    RunLoopSetup(WorkerRunLoop& runLoop)
        : m_runLoop(runLoop)
    {
        if (!m_runLoop.m_nestedCount)
            threadGlobalData().threadTimers().setSharedTimer(m_runLoop.m_sharedTimer.get());
        m_runLoop.m_nestedCount++;
    }

    ~RunLoopSetup()
    {
        m_runLoop.m_nestedCount--;
        if (!m_runLoop.m_nestedCount)
            threadGlobalData().threadTimers().setSharedTimer(0);
    }
private:
    WorkerRunLoop& m_runLoop;
};

void WorkerRunLoop::run(WorkerContext* context)
{
    RunLoopSetup setup(*this);
    ModePredicate modePredicate(defaultMode());
    MessageQueueWaitResult result;
    do {
        result = runInMode(context, modePredicate);
    } while (result != MessageQueueTerminated);
}

MessageQueueWaitResult WorkerRunLoop::runInMode(WorkerContext* context, const String& mode)
{
    RunLoopSetup setup(*this);
    ModePredicate modePredicate(mode);
    MessageQueueWaitResult result = runInMode(context, modePredicate);
    return result;
}

MessageQueueWaitResult WorkerRunLoop::runInMode(WorkerContext* context, const ModePredicate& predicate)
{
    ASSERT(context);
    ASSERT(context->thread());
    ASSERT(context->thread()->threadID() == currentThread());

    double absoluteTime = (predicate.isDefaultMode() && m_sharedTimer->isActive()) ? m_sharedTimer->fireTime() : MessageQueue<Task>::infiniteTime();
    MessageQueueWaitResult result;
    OwnPtr<WorkerRunLoop::Task> task = m_messageQueue.waitForMessageFilteredWithTimeout(result, predicate, absoluteTime);

    // If the context is closing, don't execute any further JavaScript tasks (per section 4.1.1 of the Web Workers spec).  However, there may be implementation cleanup tasks in the queue, so keep running through it.

    switch (result) {
    case MessageQueueTerminated:
        break;

    case MessageQueueMessageReceived:
        task->performTask(context);
        break;

    case MessageQueueTimeout:
        if (!context->isClosing())
            m_sharedTimer->fire();
        break;
    }

    return result;
}

void WorkerRunLoop::terminate()
{
    m_messageQueue.kill();
}

void WorkerRunLoop::postTask(PassOwnPtr<ScriptExecutionContext::Task> task)
{
    postTaskForMode(task, defaultMode());
}

void WorkerRunLoop::postTaskForMode(PassOwnPtr<ScriptExecutionContext::Task> task, const String& mode)
{
    m_messageQueue.append(Task::create(task, mode.crossThreadString()));
}

PassOwnPtr<WorkerRunLoop::Task> WorkerRunLoop::Task::create(PassOwnPtr<ScriptExecutionContext::Task> task, const String& mode)
{
    return new Task(task, mode);
}

void WorkerRunLoop::Task::performTask(ScriptExecutionContext* context)
{
    WorkerContext* workerContext = static_cast<WorkerContext *>(context);
    if (!workerContext->isClosing() || m_task->isCleanupTask())
        m_task->performTask(context);
}

WorkerRunLoop::Task::Task(PassOwnPtr<ScriptExecutionContext::Task> task, const String& mode)
    : m_task(task)
    , m_mode(mode.crossThreadString())
{
}


} // namespace WebCore

#endif // ENABLE(WORKERS)
