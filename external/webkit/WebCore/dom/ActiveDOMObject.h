

#ifndef ActiveDOMObject_h
#define ActiveDOMObject_h

#include <wtf/Assertions.h>

namespace WebCore {

    class ScriptExecutionContext;

    class ActiveDOMObject {
    public:
        ActiveDOMObject(ScriptExecutionContext*, void* upcastPointer);

        ScriptExecutionContext* scriptExecutionContext() const { return m_scriptExecutionContext; }
        virtual bool hasPendingActivity() const;

        virtual void contextDestroyed();

        // canSuspend() is used by the caller if there is a choice between suspending and stopping.
        // For example, a page won't be suspended and placed in the back/forward cache if it has
        // the objects that can not be suspended.
        // However, 'suspend' can be called even if canSuspend() would return 'false'. That
        // happens in step-by-step JS debugging for example - in this case it would be incorrect
        // to stop the object. Exact semantics of suspend is up to the object then.
        virtual bool canSuspend() const;
        virtual void suspend();
        virtual void resume();
        virtual void stop();

        template<class T> void setPendingActivity(T* thisObject)
        {
            ASSERT(thisObject == this);
            thisObject->ref();
            m_pendingActivityCount++;
        }

        template<class T> void unsetPendingActivity(T* thisObject)
        {
            ASSERT(m_pendingActivityCount > 0);
            --m_pendingActivityCount;
            thisObject->deref();
        }

    protected:
        virtual ~ActiveDOMObject();

    private:
        ScriptExecutionContext* m_scriptExecutionContext;
        unsigned m_pendingActivityCount;
    };

} // namespace WebCore

#endif // ActiveDOMObject_h
