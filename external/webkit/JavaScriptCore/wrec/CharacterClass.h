

#ifndef CharacterClass_h
#define CharacterClass_h

#include <wtf/Platform.h>

#if ENABLE(WREC)

#include <wtf/unicode/Unicode.h>

namespace JSC { namespace WREC {

    struct CharacterRange {
        UChar begin;
        UChar end;
    };

    struct CharacterClass {
        static const CharacterClass& newline();
        static const CharacterClass& digits();
        static const CharacterClass& spaces();
        static const CharacterClass& wordchar();
        static const CharacterClass& nondigits();
        static const CharacterClass& nonspaces();
        static const CharacterClass& nonwordchar();

        const UChar* matches;
        unsigned numMatches;

        const CharacterRange* ranges;
        unsigned numRanges;

        const UChar* matchesUnicode;
        unsigned numMatchesUnicode;

        const CharacterRange* rangesUnicode;
        unsigned numRangesUnicode;
    };

} } // namespace JSC::WREC

#endif // ENABLE(WREC)

#endif // CharacterClass_h
