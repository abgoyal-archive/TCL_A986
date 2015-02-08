

#ifndef TextBreakIterator_h
#define TextBreakIterator_h

#include <wtf/unicode/Unicode.h>

namespace WebCore {

    class TextBreakIterator;

    // Note: The returned iterator is good only until you get another iterator.

    // Iterates over "extended grapheme clusters", as defined in UAX #29.
    // Note that platform implementations may be less sophisticated - e.g. ICU prior to
    // version 4.0 only supports "legacy grapheme clusters".
    // Use this for general text processing, e.g. string truncation.
    TextBreakIterator* characterBreakIterator(const UChar*, int length);

    // This is similar to character break iterator in most cases, but is subject to
    // platform UI conventions. One notable example where this can be different
    // from character break iterator is Thai prepend characters, see bug 24342.
    // Use this for insertion point and selection manipulations.
    TextBreakIterator* cursorMovementIterator(const UChar*, int length);

    TextBreakIterator* wordBreakIterator(const UChar*, int length);
    TextBreakIterator* lineBreakIterator(const UChar*, int length);
    TextBreakIterator* sentenceBreakIterator(const UChar*, int length);

    int textBreakFirst(TextBreakIterator*);
    int textBreakLast(TextBreakIterator*);
    int textBreakNext(TextBreakIterator*);
    int textBreakPrevious(TextBreakIterator*);
    int textBreakCurrent(TextBreakIterator*);
    int textBreakPreceding(TextBreakIterator*, int);
    int textBreakFollowing(TextBreakIterator*, int);
    bool isTextBreak(TextBreakIterator*, int);

    const int TextBreakDone = -1;

}

#endif
