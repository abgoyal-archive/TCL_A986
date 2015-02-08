

#ifndef LocalStorageTask_h
#define LocalStorageTask_h

#if ENABLE(DOM_STORAGE)

#include <wtf/PassOwnPtr.h>
#include <wtf/Threading.h>

namespace WebCore {

    class StorageAreaSync;
    class LocalStorageThread;

    // FIXME: Rename this class to StorageTask
    class LocalStorageTask : public Noncopyable {
    public:
        enum Type { AreaImport, AreaSync, TerminateThread };

        ~LocalStorageTask();

        static PassOwnPtr<LocalStorageTask> createImport(StorageAreaSync* area) { return new LocalStorageTask(AreaImport, area); }
        static PassOwnPtr<LocalStorageTask> createSync(StorageAreaSync* area) { return new LocalStorageTask(AreaSync, area); }
        static PassOwnPtr<LocalStorageTask> createTerminate(LocalStorageThread* thread) { return new LocalStorageTask(TerminateThread, thread); }

        void performTask();

    private:
        LocalStorageTask(Type, StorageAreaSync*);
        LocalStorageTask(Type, LocalStorageThread*);

        Type m_type;
        StorageAreaSync* m_area;
        LocalStorageThread* m_thread;
    };

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)

#endif // LocalStorageTask_h
