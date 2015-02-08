

#ifndef break_lines_h
#define break_lines_h

#include <wtf/unicode/Unicode.h>

namespace WebCore {

    int nextBreakablePosition(const UChar*, int pos, int len, bool breakNBSP = false);

    inline bool isBreakable(const UChar* str, int pos, int len, int& nextBreakable, bool breakNBSP = false)
    {
        if (pos > nextBreakable)
            nextBreakable = nextBreakablePosition(str, pos, len, breakNBSP);
        return pos == nextBreakable;
    }

} // namespace WebCore

#endif // break_lines_h
