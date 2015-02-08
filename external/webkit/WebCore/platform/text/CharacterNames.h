

#ifndef CharacterNames_h
#define CharacterNames_h

#include <wtf/unicode/Unicode.h>

namespace WebCore {

    // Names here are taken from the Unicode standard.

    // Note, these are UChar constants, not UChar32, which makes them
    // more convenient for WebCore code that mostly uses UTF-16.

    const UChar blackSquare = 0x25A0;
    const UChar bullet = 0x2022;
    const UChar ethiopicPrefaceColon = 0x1366;
    const UChar hebrewPunctuationGeresh = 0x05F3;
    const UChar hebrewPunctuationGershayim = 0x05F4;
    const UChar horizontalEllipsis = 0x2026;
    const UChar hyphenMinus = 0x002D;
    const UChar ideographicComma = 0x3001;
    const UChar ideographicFullStop = 0x3002;
    const UChar ideographicSpace = 0x3000;
    const UChar leftDoubleQuotationMark = 0x201C;
    const UChar leftSingleQuotationMark = 0x2018;
    const UChar leftToRightEmbed = 0x202A;
    const UChar leftToRightMark = 0x200E;
    const UChar leftToRightOverride = 0x202D;
    const UChar newlineCharacter = 0x000A;
    const UChar noBreakSpace = 0x00A0;
    const UChar objectReplacementCharacter = 0xFFFC;
    const UChar popDirectionalFormatting = 0x202C;
    const UChar replacementCharacter = 0xFFFD;
    const UChar rightDoubleQuotationMark = 0x201D;
    const UChar rightSingleQuotationMark = 0x2019;
    const UChar rightToLeftEmbed = 0x202B;
    const UChar rightToLeftMark = 0x200F;
    const UChar rightToLeftOverride = 0x202E;
    const UChar softHyphen = 0x00AD;
    const UChar space = 0x0020;
    const UChar whiteBullet = 0x25E6;
    const UChar zeroWidthSpace = 0x200B;

}

#endif // CharacterNames_h
