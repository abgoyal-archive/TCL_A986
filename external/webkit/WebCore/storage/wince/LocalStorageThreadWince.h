


#ifndef LocalStorageThreadWince_h
#define LocalStorageThreadWince_h

#include <wtf/Deque.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

    class StorageAreaSync;
    class LocalStorageTask;

    class LocalStorageThread : public RefCounted<LocalStorageThread> {
    public:
        static PassRefPtr<LocalStorageThread> create() { return adoptRef(new LocalStorageThread); }

        ~LocalStorageThread();
        bool start();
        void scheduleImport(PassRefPtr<StorageAreaSync>);
        void scheduleSync(PassRefPtr<StorageAreaSync>);
        void terminate();
        void performTerminate();

    private:
        LocalStorageThread();

        void timerFired(Timer<LocalStorageThread>*);

        Deque<RefPtr<LocalStorageTask> > m_queue;
        Timer<LocalStorageThread> m_timer;
    };

} // namespace WebCore

#endif // LocalStorageThreadWince_h
