

#ifndef PluginMainThreadScheduler_h
#define PluginMainThreadScheduler_h

#include <wtf/Deque.h>
#include <wtf/HashMap.h>
#include <wtf/MainThread.h>
#include <wtf/Threading.h>

typedef struct _NPP NPP_t;
typedef NPP_t* NPP;

namespace WebCore {

class PluginMainThreadScheduler : public Noncopyable {
public:
    typedef void MainThreadFunction(void*);

    static PluginMainThreadScheduler& scheduler();

    void scheduleCall(NPP, MainThreadFunction*, void* userData);

    void registerPlugin(NPP);
    void unregisterPlugin(NPP);

private:
    PluginMainThreadScheduler();
    void dispatchCalls();

    class Call;

    void dispatchCallsForPlugin(NPP, const Deque<Call>& calls);
    typedef HashMap<NPP, Deque<Call> > CallQueueMap;

    static void mainThreadCallback(void* context);

    class Call {
    public:
        Call(MainThreadFunction* function, void* userData)
            : m_function(function)
            , m_userData(userData)
        {
        }

        void performCall() const
        {
            m_function(m_userData);
        }

    private:
        MainThreadFunction* m_function;
        void* m_userData;
    };

    bool m_callPending;
    CallQueueMap m_callQueueMap;
    Mutex m_queueMutex;
};

} // namespace WebCore

#endif // PluginMainThreadScheduler_h
