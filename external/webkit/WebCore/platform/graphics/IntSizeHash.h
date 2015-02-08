
#ifndef IntSizeHash_h
#define IntSizeHash_h

#include "IntSize.h"
#include <wtf/HashMap.h>
#include <wtf/HashSet.h>

using WebCore::IntSize;

namespace WTF {

    template<> struct IntHash<IntSize> {
        static unsigned hash(const IntSize& key) { return intHash((static_cast<uint64_t>(key.width()) << 32 | key.height())); }
        static bool equal(const IntSize& a, const IntSize& b) { return a == b; }
        static const bool safeToCompareToEmptyOrDeleted = true;
    };
    template<> struct DefaultHash<IntSize> { typedef IntHash<IntSize> Hash; };
    
    template<> struct HashTraits<IntSize> : GenericHashTraits<IntSize> {
        static const bool emptyValueIsZero = true;
        static const bool needsDestruction = false;
        static void constructDeletedValue(IntSize& slot) { new (&slot) IntSize(-1, -1); }
        static bool isDeletedValue(const IntSize& value) { return value.width() == -1 && value.height() == -1; }
    };
} // namespace WTF

#endif
