

#ifndef NumericStrings_h
#define NumericStrings_h

#include "UString.h"
#include <wtf/HashFunctions.h>

namespace JSC {

    class NumericStrings {
    public:
        UString add(double d)
        {
            CacheEntry<double>& entry = lookup(d);
            if (d == entry.key && !entry.value.isNull())
                return entry.value;
            entry.key = d;
            entry.value = UString::from(d);
            return entry.value;
        }

        UString add(int i)
        {
            CacheEntry<int>& entry = lookup(i);
            if (i == entry.key && !entry.value.isNull())
                return entry.value;
            entry.key = i;
            entry.value = UString::from(i);
            return entry.value;
        }

    private:
        static const size_t cacheSize = 64;

        template<typename T>
        struct CacheEntry {
            T key;
            UString value;
        };

        CacheEntry<double>& lookup(double d) { return doubleCache[WTF::FloatHash<double>::hash(d) & (cacheSize - 1)]; }
        CacheEntry<int>& lookup(int i) { return intCache[WTF::IntHash<int>::hash(i) & (cacheSize - 1)]; }

        CacheEntry<double> doubleCache[cacheSize];
        CacheEntry<int> intCache[cacheSize];
    };

} // namespace JSC

#endif // NumericStrings_h
