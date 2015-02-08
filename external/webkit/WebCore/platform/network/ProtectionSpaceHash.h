

#ifndef ProtectionSpaceHash_h
#define ProtectionSpaceHash_h

#include "ProtectionSpace.h"
#include <wtf/HashTraits.h>

namespace WebCore {

struct ProtectionSpaceHash {
    static unsigned hash(const ProtectionSpace& protectionSpace)
    { 
        unsigned hashCodes[5] = {
            protectionSpace.host().impl() ? protectionSpace.host().impl()->hash() : 0, 
            protectionSpace.port(), 
            protectionSpace.serverType(),
            protectionSpace.authenticationScheme(),
            protectionSpace.realm().impl() ? protectionSpace.realm().impl()->hash() : 0
        };

        unsigned codeCount = sizeof(hashCodes) / sizeof(UChar);
        // Ignore realm for proxies.
        if (protectionSpace.isProxy())
            codeCount -= sizeof(hashCodes[0]) / sizeof(UChar);
        return StringImpl::computeHash(reinterpret_cast<UChar*>(hashCodes), codeCount);
    }
    
    static bool equal(const ProtectionSpace& a, const ProtectionSpace& b) { return a == b; }
    static const bool safeToCompareToEmptyOrDeleted = false;
};

} // namespace WebCore

namespace WTF {

    // WebCore::ProtectionSpaceHash is the default hash for ProtectionSpace
    template<> struct HashTraits<WebCore::ProtectionSpace> : GenericHashTraits<WebCore::ProtectionSpace> {
        static const bool emptyValueIsZero = true;
        static void constructDeletedValue(WebCore::ProtectionSpace& slot) { new (&slot) WebCore::ProtectionSpace(HashTableDeletedValue); }
        static bool isDeletedValue(const WebCore::ProtectionSpace& slot) { return slot.isHashTableDeletedValue(); }
    };

    template<typename T> struct DefaultHash;
    template<> struct DefaultHash<WebCore::ProtectionSpace> {
        typedef WebCore::ProtectionSpaceHash Hash;
    };

} // namespace WTF


#endif // ProtectionSpaceHash_h
