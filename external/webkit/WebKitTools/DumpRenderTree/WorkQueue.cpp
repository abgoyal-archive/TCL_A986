

#include "config.h"
#include "WorkQueue.h"

#include "WorkQueueItem.h"
#include <wtf/Assertions.h>

static const unsigned queueLength = 1024;

static WorkQueueItem* theQueue[queueLength];
static unsigned startOfQueue;
static unsigned endOfQueue;

WorkQueue* WorkQueue::shared()
{
    static WorkQueue* sharedInstance = new WorkQueue;
    return sharedInstance;
}

WorkQueue::WorkQueue()
    : m_frozen(false)
{
}

void WorkQueue::queue(WorkQueueItem* item)
{
    ASSERT(endOfQueue < queueLength);
    ASSERT(endOfQueue >= startOfQueue);

    if (m_frozen) {
        delete item;
        return;
    }

    theQueue[endOfQueue++] = item;
}

WorkQueueItem* WorkQueue::dequeue()
{
    ASSERT(endOfQueue >= startOfQueue);

    if (startOfQueue == endOfQueue)
        return 0;

    return theQueue[startOfQueue++];
}

unsigned WorkQueue::count()
{
    return endOfQueue - startOfQueue;
}

void WorkQueue::clear()
{
    for (unsigned i = startOfQueue; i < endOfQueue; ++i) {
        delete theQueue[i];
        theQueue[i] = 0;
    }

    startOfQueue = 0;
    endOfQueue = 0;
}

bool WorkQueue::processWork()
{
    bool startedLoad = false;

    while (!startedLoad && count()) {
        WorkQueueItem* item = dequeue();
        ASSERT(item);
        startedLoad = item->invoke();
        delete item;
    }

    // If we're done and we didn't start a load, then we're really done, so return true.
    return !startedLoad;
}
