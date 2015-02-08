

#ifndef StorageMap_h
#define StorageMap_h

#if ENABLE(DOM_STORAGE)

#include "PlatformString.h"
#include "StringHash.h"

#include <wtf/HashMap.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    class StorageMap : public RefCounted<StorageMap> {
    public:
        // Quota size mesured in bytes.
        static PassRefPtr<StorageMap> create(unsigned quotaSize);

        unsigned length() const;
        String key(unsigned index);
        String getItem(const String&) const;
        PassRefPtr<StorageMap> setItem(const String& key, const String& value, String& oldValue, bool& quota_exception);
        PassRefPtr<StorageMap> removeItem(const String&, String& oldValue);

        bool contains(const String& key) const;

        void importItem(const String& key, const String& value);

        unsigned quota() const { return m_quotaSize; }

        static const unsigned noQuota = UINT_MAX;

    private:
        StorageMap(unsigned quota);
        PassRefPtr<StorageMap> copy();
        void invalidateIterator();
        void setIteratorToIndex(unsigned);

        HashMap<String, String> m_map;
        HashMap<String, String>::iterator m_iterator;
        unsigned m_iteratorIndex;

        unsigned m_quotaSize;  // Measured in bytes.
        unsigned m_currentLength;  // Measured in UChars.
    };

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)

#endif // StorageMap_h
