

#ifndef StorageSyncManager_h
#define StorageSyncManager_h

#if ENABLE(DOM_STORAGE)

#include "PlatformString.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/OwnPtr.h>

namespace WebCore {

    class LocalStorageThread;
    class SecurityOrigin;
    class StorageAreaSync;

    class StorageSyncManager : public RefCounted<StorageSyncManager> {
    public:
        static PassRefPtr<StorageSyncManager> create(const String& path);
        ~StorageSyncManager();

        bool scheduleImport(PassRefPtr<StorageAreaSync>);
        void scheduleSync(PassRefPtr<StorageAreaSync>);

        void close();

    private:
        StorageSyncManager(const String& path);

        OwnPtr<LocalStorageThread> m_thread;

    // The following members are subject to thread synchronization issues
    public:
        // To be called from the background thread:
        String fullDatabaseFilename(const String& databaseIdentifier);

    private:
        String m_path;
    };

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)

#endif // StorageSyncManager_h
