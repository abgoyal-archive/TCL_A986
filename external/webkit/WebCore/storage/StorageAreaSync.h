

#ifndef StorageAreaSync_h
#define StorageAreaSync_h

#if ENABLE(DOM_STORAGE)

#include "PlatformString.h"
#include "SQLiteDatabase.h"
#include "StringHash.h"
#include "Timer.h"
#include <wtf/HashMap.h>

namespace WebCore {

    class Frame;
    class StorageAreaImpl;
    class StorageSyncManager;

    class StorageAreaSync : public RefCounted<StorageAreaSync> {
    public:
        static PassRefPtr<StorageAreaSync> create(PassRefPtr<StorageSyncManager> storageSyncManager, PassRefPtr<StorageAreaImpl> storageArea, String databaseIdentifier);
        ~StorageAreaSync();

        void scheduleFinalSync();
        void blockUntilImportComplete();

        void scheduleItemForSync(const String& key, const String& value);
        void scheduleClear();

    private:
        StorageAreaSync(PassRefPtr<StorageSyncManager> storageSyncManager, PassRefPtr<StorageAreaImpl> storageArea, String databaseIdentifier);

        void dispatchStorageEvent(const String& key, const String& oldValue, const String& newValue, Frame* sourceFrame);

        Timer<StorageAreaSync> m_syncTimer;
        HashMap<String, String> m_changedItems;
        bool m_itemsCleared;

        bool m_finalSyncScheduled;

        RefPtr<StorageAreaImpl> m_storageArea;
        RefPtr<StorageSyncManager> m_syncManager;

        // The database handle will only ever be opened and used on the background thread.
        SQLiteDatabase m_database;

    // The following members are subject to thread synchronization issues.
    public:
        // Called from the background thread
        void performImport();
        void performSync();

    private:
        void syncTimerFired(Timer<StorageAreaSync>*);
        void sync(bool clearItems, const HashMap<String, String>& items);

        const String m_databaseIdentifier;

        Mutex m_syncLock;
        HashMap<String, String> m_itemsPendingSync;
        bool m_clearItemsWhileSyncing;
        bool m_syncScheduled;

        mutable Mutex m_importLock;
        mutable ThreadCondition m_importCondition;
        mutable bool m_importComplete;
        void markImported();
    };

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)

#endif // StorageAreaSync_h
