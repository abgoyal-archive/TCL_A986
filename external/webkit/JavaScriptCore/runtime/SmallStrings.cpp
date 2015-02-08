

#include "config.h"
#include "SmallStrings.h"

#include "JSGlobalObject.h"
#include "JSString.h"

#include <wtf/Noncopyable.h>

namespace JSC {
static const unsigned numCharactersToStore = 0x100;

static inline bool isMarked(JSString* string)
{
    return string && Heap::isCellMarked(string);
}

class SmallStringsStorage : public Noncopyable {
public:
    SmallStringsStorage();

    UString::Rep* rep(unsigned char character) { return &m_reps[character]; }

private:
    UString::Rep m_reps[numCharactersToStore];
};

SmallStringsStorage::SmallStringsStorage()
{
    UChar* characterBuffer = 0;
    RefPtr<UStringImpl> baseString = UStringImpl::createUninitialized(numCharactersToStore, characterBuffer);
    for (unsigned i = 0; i < numCharactersToStore; ++i) {
        characterBuffer[i] = i;
        new (&m_reps[i]) UString::Rep(&characterBuffer[i], 1, PassRefPtr<UStringImpl>(baseString));
    }
}

SmallStrings::SmallStrings()
{
    COMPILE_ASSERT(numCharactersToStore == sizeof(m_singleCharacterStrings) / sizeof(m_singleCharacterStrings[0]), IsNumCharactersConstInSyncWithClassUsage);
    clear();
}

SmallStrings::~SmallStrings()
{
}

void SmallStrings::markChildren(MarkStack& markStack)
{
    /*
       Our hypothesis is that small strings are very common. So, we cache them
       to avoid GC churn. However, in cases where this hypothesis turns out to
       be false -- including the degenerate case where all JavaScript execution
       has terminated -- we don't want to waste memory.

       To test our hypothesis, we check if any small string has been marked. If
       so, it's probably reasonable to mark the rest. If not, we clear the cache.
     */

    bool isAnyStringMarked = isMarked(m_emptyString);
    for (unsigned i = 0; i < numCharactersToStore && !isAnyStringMarked; ++i)
        isAnyStringMarked |= isMarked(m_singleCharacterStrings[i]);
    
    if (!isAnyStringMarked) {
        clear();
        return;
    }
    
    if (m_emptyString)
        markStack.append(m_emptyString);
    for (unsigned i = 0; i < numCharactersToStore; ++i) {
        if (m_singleCharacterStrings[i])
            markStack.append(m_singleCharacterStrings[i]);
    }
}

void SmallStrings::clear()
{
    m_emptyString = 0;
    for (unsigned i = 0; i < numCharactersToStore; ++i)
        m_singleCharacterStrings[i] = 0;
}

unsigned SmallStrings::count() const
{
    unsigned count = 0;
    if (m_emptyString)
        ++count;
    for (unsigned i = 0; i < numCharactersToStore; ++i) {
        if (m_singleCharacterStrings[i])
            ++count;
    }
    return count;
}

void SmallStrings::createEmptyString(JSGlobalData* globalData)
{
    ASSERT(!m_emptyString);
    m_emptyString = new (globalData) JSString(globalData, "", JSString::HasOtherOwner);
}

void SmallStrings::createSingleCharacterString(JSGlobalData* globalData, unsigned char character)
{
    if (!m_storage)
        m_storage.set(new SmallStringsStorage);
    ASSERT(!m_singleCharacterStrings[character]);
    m_singleCharacterStrings[character] = new (globalData) JSString(globalData, m_storage->rep(character), JSString::HasOtherOwner);
}

UString::Rep* SmallStrings::singleCharacterStringRep(unsigned char character)
{
    if (!m_storage)
        m_storage.set(new SmallStringsStorage);
    return m_storage->rep(character);
}

} // namespace JSC
