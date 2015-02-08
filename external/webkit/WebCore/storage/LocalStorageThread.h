

#ifndef LocalStorageThread_h
#define LocalStorageThread_h

#if ENABLE(DOM_STORAGE)

#include <wtf/HashSet.h>
#include <wtf/MessageQueue.h>
#include <wtf/PassOwnPtr.h>
#include <wtf/PassRefPtr.h>
#include <wtf/Threading.h>

namespace WebCore {

    class StorageAreaSync;
    class LocalStorageTask;

    // FIXME: Rename this class to StorageThread
    class LocalStorageThread : public Noncopyable {
    public:
        static PassOwnPtr<LocalStorageThread> create();
        ~LocalStorageThread();

        bool start();
        void terminate();
        void scheduleTask(PassOwnPtr<LocalStorageTask>);

        // Background thread part of the terminate procedure.
        void performTerminate();

    private:
        LocalStorageThread();

        // Called on background thread.
        static void* threadEntryPointCallback(void*);
        void* threadEntryPoint();

        ThreadIdentifier m_threadID;
        MessageQueue<LocalStorageTask> m_queue;
    };

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)

#endif // LocalStorageThread_h
