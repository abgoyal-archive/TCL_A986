

#ifndef TextBoundaries_h
#define TextBoundaries_h

#include <wtf/unicode/Unicode.h>

namespace WebCore {

    inline bool requiresContextForWordBoundary(UChar32 ch)
    {
        return WTF::Unicode::hasLineBreakingPropertyComplexContext(ch);
    }

    void findWordBoundary(const UChar*, int len, int position, int* start, int* end);
    int findNextWordFromIndex(const UChar*, int len, int position, bool forward);

}

#endif
