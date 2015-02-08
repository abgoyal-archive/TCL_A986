

#include "config.h"
#include "break_lines.h"

#include "CharacterNames.h"
#include "TextBreakIterator.h"

#if PLATFORM(MAC)
#include <CoreServices/CoreServices.h>
#endif

namespace WebCore {

static inline bool isBreakableSpace(UChar ch, bool treatNoBreakSpaceAsBreak)
{
    switch (ch) {
        case ' ':
        case '\n':
        case '\t':
            return true;
        case noBreakSpace:
            return treatNoBreakSpaceAsBreak;
        default:
            return false;
    }
}

// This differs from the Unicode algorithm only in that Unicode does not break
// between a question mark and a vertical line (U+007C).
static const unsigned char internetExplorerLineBreaksAfterQuestionMarkTable[0x80] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, // \t
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, // ! " ' ) , . /
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, // : ; ?
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, // ]
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1  // }
};

static const size_t internetExplorerLineBreaksAfterQuestionMarkTableSize = sizeof(internetExplorerLineBreaksAfterQuestionMarkTable) / sizeof(*internetExplorerLineBreaksAfterQuestionMarkTable);

static inline bool shouldBreakAfter(UChar ch, UChar nextCh)
{
    switch (ch) {
        // For a question mark preceding a non-ASCII characters, defer to the Unicode algorithm by returning false.
        // For ASCII characters, use a lookup table for enhanced speed and for compatibility with Internet Explorer. 
        case '?':
            return nextCh < internetExplorerLineBreaksAfterQuestionMarkTableSize && internetExplorerLineBreaksAfterQuestionMarkTable[nextCh];
        // Internet Explorer always allows breaking after a hyphen.
        case '-':
        case softHyphen:
        // FIXME: cases for ideographicComma and ideographicFullStop are a workaround for an issue in Unicode 5.0
        // which is likely to be resolved in Unicode 5.1 <http://bugs.webkit.org/show_bug.cgi?id=17411>.
        // We may want to remove or conditionalize this workaround at some point.
        case ideographicComma:
        case ideographicFullStop:
#ifdef ANDROID_LAYOUT
        // as '/' is used in uri which is always long, we would like to break it
        case '/':
#endif
            return true;
        default:
            return false;
    }
}

static inline bool needsLineBreakIterator(UChar ch)
{
    return ch > 0x7F && ch != noBreakSpace;
}

#if PLATFORM(MAC) && defined(BUILDING_ON_TIGER)
static inline TextBreakLocatorRef lineBreakLocator()
{
    TextBreakLocatorRef locator = 0;
    UCCreateTextBreakLocator(0, 0, kUCTextBreakLineMask, &locator);
    return locator;
}
#endif

int nextBreakablePosition(const UChar* str, int pos, int len, bool treatNoBreakSpaceAsBreak)
{
#if !PLATFORM(MAC) || !defined(BUILDING_ON_TIGER)
    TextBreakIterator* breakIterator = 0;
#endif
    int nextBreak = -1;

    UChar lastCh = pos > 0 ? str[pos - 1] : 0;
    for (int i = pos; i < len; i++) {
        UChar ch = str[i];

        if (isBreakableSpace(ch, treatNoBreakSpaceAsBreak) || shouldBreakAfter(lastCh, ch))
            return i;

        if (needsLineBreakIterator(ch) || needsLineBreakIterator(lastCh)) {
            if (nextBreak < i && i) {
#if !PLATFORM(MAC) || !defined(BUILDING_ON_TIGER)
                if (!breakIterator)
                    breakIterator = lineBreakIterator(str, len);
                if (breakIterator)
                    nextBreak = textBreakFollowing(breakIterator, i - 1);
#else
                static TextBreakLocatorRef breakLocator = lineBreakLocator();
                if (breakLocator) {
                    UniCharArrayOffset nextUCBreak;
                    if (UCFindTextBreak(breakLocator, kUCTextBreakLineMask, 0, str, len, i, &nextUCBreak) == 0)
                        nextBreak = nextUCBreak;
                }
#endif
            }
            if (i == nextBreak && !isBreakableSpace(lastCh, treatNoBreakSpaceAsBreak))
                return i;
        }

        lastCh = ch;
    }

    return len;
}

} // namespace WebCore
