

#include "config.h"
#include "TextBoundaries.h"

#include "StringImpl.h"
#include "TextBreakIterator.h"
#include <wtf/unicode/Unicode.h>

using namespace WTF;
using namespace Unicode;

namespace WebCore {

int findNextWordFromIndex(const UChar* chars, int len, int position, bool forward)
{
    TextBreakIterator* it = wordBreakIterator(chars, len);

    if (forward) {
        position = textBreakFollowing(it, position);
        while (position != TextBreakDone) {
            // We stop searching when the character preceeding the break
            // is alphanumeric.
            if (position < len && isAlphanumeric(chars[position - 1]))
                return position;

            position = textBreakFollowing(it, position);
        }

        return len;
    } else {
        position = textBreakPreceding(it, position);
        while (position != TextBreakDone) {
            // We stop searching when the character following the break
            // is alphanumeric.
            if (position > 0 && isAlphanumeric(chars[position]))
                return position;

            position = textBreakPreceding(it, position);
        }

        return 0;
    }
}

void findWordBoundary(const UChar* chars, int len, int position, int* start, int* end)
{
    TextBreakIterator* it = wordBreakIterator(chars, len);
    *end = textBreakFollowing(it, position);
    if (*end < 0)
        *end = textBreakLast(it);
    *start = textBreakPrevious(it);
}

} // namespace WebCore
