

#ifndef SecurityOriginHash_h
#define SecurityOriginHash_h

#include "KURL.h"
#include "SecurityOrigin.h"
#include <wtf/RefPtr.h>

namespace WebCore {

struct SecurityOriginHash {
    static unsigned hash(SecurityOrigin* origin)
    {
        unsigned hashCodes[3] = {
            origin->protocol().impl() ? origin->protocol().impl()->hash() : 0,
            origin->host().impl() ? origin->host().impl()->hash() : 0,
            origin->port()
        };
        return StringImpl::computeHash(reinterpret_cast<UChar*>(hashCodes), sizeof(hashCodes) / sizeof(UChar));
    }
    static unsigned hash(const RefPtr<SecurityOrigin>& origin)
    {
        return hash(origin.get());
    }

    static bool equal(SecurityOrigin* a, SecurityOrigin* b)
    {
        // FIXME: The hash function above compares three specific fields.
        // This code to compare those three specific fields should be moved here from
        // SecurityOrigin as mentioned in SecurityOrigin.h so we don't accidentally change
        // equal without changing hash to match it.
        if (!a || !b)
            return a == b;
        return a->equal(b);
    }
    static bool equal(SecurityOrigin* a, const RefPtr<SecurityOrigin>& b)
    {
        return equal(a, b.get());
    }
    static bool equal(const RefPtr<SecurityOrigin>& a, SecurityOrigin* b)
    {
        return equal(a.get(), b);
    }
    static bool equal(const RefPtr<SecurityOrigin>& a, const RefPtr<SecurityOrigin>& b)
    {
        return equal(a.get(), b.get());
    }

    static const bool safeToCompareToEmptyOrDeleted = false;
};

} // namespace WebCore

#endif
