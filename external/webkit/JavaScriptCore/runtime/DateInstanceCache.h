

#ifndef DateInstanceCache_h
#define DateInstanceCache_h

#include <wtf/DateMath.h>
#include <wtf/HashFunctions.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace JSC {

    extern const double NaN;

    class DateInstanceData : public RefCounted<DateInstanceData> {
    public:
        static PassRefPtr<DateInstanceData> create() { return adoptRef(new DateInstanceData); }

        double m_gregorianDateTimeCachedForMS;
        GregorianDateTime m_cachedGregorianDateTime;
        double m_gregorianDateTimeUTCCachedForMS;
        GregorianDateTime m_cachedGregorianDateTimeUTC;

    private:
        DateInstanceData()
            : m_gregorianDateTimeCachedForMS(NaN)
            , m_gregorianDateTimeUTCCachedForMS(NaN)
        {
        }
    };

    class DateInstanceCache {
    public:
        DateInstanceCache()
        {
            reset();
        }
        
        void reset()
        {
            for (size_t i = 0; i < cacheSize; ++i)
                m_cache[i].key = NaN;
        }
        
        DateInstanceData* add(double d)
        {
            CacheEntry& entry = lookup(d);
            if (d == entry.key)
                return entry.value.get();

            entry.key = d;
            entry.value = DateInstanceData::create();
            return entry.value.get();
        }

    private:
        static const size_t cacheSize = 16;

        struct CacheEntry {
            double key;
            RefPtr<DateInstanceData> value;
        };

        CacheEntry& lookup(double d) { return m_cache[WTF::FloatHash<double>::hash(d) & (cacheSize - 1)]; }

        CacheEntry m_cache[cacheSize];
    };

} // namespace JSC

#endif // DateInstanceCache_h
