

#ifndef KURLHash_h
#define KURLHash_h

#include "KURL.h"
#include "PlatformString.h"
#include "StringHash.h"

namespace WebCore {

    struct KURLHash {
        static unsigned hash(const KURL& key)
        {
            return key.string().impl()->hash();
        }

        static bool equal(const KURL& a, const KURL& b)
        {
            return StringHash::equal(a.string(), b.string());
        }

        static const bool safeToCompareToEmptyOrDeleted = false;
    };

} // namespace WebCore

namespace WTF {

    template<> struct HashTraits<WebCore::KURL> : GenericHashTraits<WebCore::KURL> {
        static const bool emptyValueIsZero = true;
        static void constructDeletedValue(WebCore::KURL& slot) { new (&slot) WebCore::KURL(WebCore::ParsedURLString, WebCore::String(HashTableDeletedValue)); }
        static bool isDeletedValue(const WebCore::KURL& slot) { return slot.string().isHashTableDeletedValue(); }
    };

} // namespace WTF

#endif // KURLHash_h
