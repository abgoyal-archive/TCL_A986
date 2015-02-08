

#ifndef SmallStrings_h
#define SmallStrings_h

#include "UString.h"
#include <wtf/OwnPtr.h>

namespace JSC {

    class JSGlobalData;
    class JSString;
    class MarkStack;
    class SmallStringsStorage;

    class SmallStrings : public Noncopyable {
    public:
        SmallStrings();
        ~SmallStrings();

        JSString* emptyString(JSGlobalData* globalData)
        {
            if (!m_emptyString)
                createEmptyString(globalData);
            return m_emptyString;
        }
        JSString* singleCharacterString(JSGlobalData* globalData, unsigned char character)
        {
            if (!m_singleCharacterStrings[character])
                createSingleCharacterString(globalData, character);
            return m_singleCharacterStrings[character];
        }

        UString::Rep* singleCharacterStringRep(unsigned char character);

        void markChildren(MarkStack&);
        void clear();

        unsigned count() const;

    private:
        void createEmptyString(JSGlobalData*);
        void createSingleCharacterString(JSGlobalData*, unsigned char);

        JSString* m_emptyString;
        JSString* m_singleCharacterStrings[0x100];
        OwnPtr<SmallStringsStorage> m_storage;
    };

} // namespace JSC

#endif // SmallStrings_h
