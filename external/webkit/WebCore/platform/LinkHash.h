

#ifndef LinkHash_h
#define LinkHash_h

#include "StringHash.h"

namespace WebCore {

class AtomicString;
class KURL;

typedef uint64_t LinkHash;

// Use the low 32-bits of the 64-bit LinkHash as the key for HashSets.
struct LinkHashHash {
    static unsigned hash(LinkHash key) { return static_cast<unsigned>(key); }
    static bool equal(LinkHash a, LinkHash b) { return a == b; }
    static const bool safeToCompareToEmptyOrDeleted = true;

    // See AlreadyHashed::avoidDeletedValue.
    static unsigned avoidDeletedValue(LinkHash hash64)
    {
        ASSERT(hash64);
        unsigned hash = static_cast<unsigned>(hash64);
        unsigned newHash = hash | (!(hash + 1) << 31);
        ASSERT(newHash);
        ASSERT(newHash != 0xFFFFFFFF);
        return newHash;
    }
};

// Returns the has of the string that will be used for visited link coloring.
LinkHash visitedLinkHash(const UChar* url, unsigned length);

// Resolves the potentially relative URL "attributeURL" relative to the given
// base URL, and returns the hash of the string that will be used for visited
// link coloring. It will return the special value of 0 if attributeURL does not
// look like a relative URL.
LinkHash visitedLinkHash(const KURL& base, const AtomicString& attributeURL);

// Resolves the potentially relative URL "attributeURL" relative to the given
// base URL, and returns the hash of the string that will be used for visited.
// It will return an empty Vector in case of errors.
void visitedURL(const KURL& base, const AtomicString& attributeURL, Vector<UChar, 512>&);


}  // namespace WebCore

#endif  // LinkHash_h
