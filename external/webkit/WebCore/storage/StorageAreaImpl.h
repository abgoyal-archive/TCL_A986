

#ifndef StorageAreaImpl_h
#define StorageAreaImpl_h

#if ENABLE(DOM_STORAGE)

#include "StorageArea.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

    class SecurityOrigin;
    class StorageMap;
    class StorageAreaSync;

    class StorageAreaImpl : public StorageArea {
    public:
        static PassRefPtr<StorageAreaImpl> create(StorageType, PassRefPtr<SecurityOrigin>, PassRefPtr<StorageSyncManager>, unsigned quota);
        virtual ~StorageAreaImpl();

        // The HTML5 DOM Storage API (and contains)
        virtual unsigned length() const;
        virtual String key(unsigned index) const;
        virtual String getItem(const String& key) const;
        virtual String setItem(const String& key, const String& value, ExceptionCode& ec, Frame* sourceFrame);
        virtual String removeItem(const String& key, Frame* sourceFrame);
        virtual bool clear(Frame* sourceFrame);
        virtual bool contains(const String& key) const;

        PassRefPtr<StorageAreaImpl> copy();
        void close();

        // Only called from a background thread.
        void importItem(const String& key, const String& value);

    private:
        StorageAreaImpl(StorageType, PassRefPtr<SecurityOrigin>, PassRefPtr<StorageSyncManager>, unsigned quota);
        StorageAreaImpl(StorageAreaImpl*);

        void blockUntilImportComplete() const;

        StorageType m_storageType;
        RefPtr<SecurityOrigin> m_securityOrigin;
        RefPtr<StorageMap> m_storageMap;

        RefPtr<StorageAreaSync> m_storageAreaSync;
        RefPtr<StorageSyncManager> m_storageSyncManager;

#ifndef NDEBUG
        bool m_isShutdown;
#endif
    };

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)

#endif // StorageAreaImpl_h
