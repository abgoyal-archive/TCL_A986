

#ifndef AtomicStringHash_h
#define AtomicStringHash_h

#include "AtomicString.h"
#include <wtf/HashTraits.h>

namespace WebCore {

    struct AtomicStringHash {
        static unsigned hash(const AtomicString& key)
        {
            return key.impl()->existingHash();
        }

        static bool equal(const AtomicString& a, const AtomicString& b)
        {
            return a == b;
        }

        static const bool safeToCompareToEmptyOrDeleted = false;
    };

}

namespace WTF {

    // WebCore::AtomicStringHash is the default hash for AtomicString
    template<> struct HashTraits<WebCore::AtomicString> : GenericHashTraits<WebCore::AtomicString> {
        static const bool emptyValueIsZero = true;
        static void constructDeletedValue(WebCore::AtomicString& slot) { new (&slot) WebCore::AtomicString(HashTableDeletedValue); }
        static bool isDeletedValue(const WebCore::AtomicString& slot) { return slot.isHashTableDeletedValue(); }
    };

}

#endif
