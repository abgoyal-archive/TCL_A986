

#ifndef WorkQueue_h
#define WorkQueue_h

class WorkQueueItem;

class WorkQueue {
public:
    static WorkQueue* shared();

    void queue(WorkQueueItem*);
    WorkQueueItem* dequeue();
    void clear();
    unsigned count();

    void setFrozen(bool b) { m_frozen = b; }

    bool processWork(); // Returns true if all work is done, false if we started a load.

private:
    WorkQueue();

    bool m_frozen;
};

#endif // !defined(WorkQueue_h)
