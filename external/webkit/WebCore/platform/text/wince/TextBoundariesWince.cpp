

#include "config.h"
#include "TextBoundaries.h"

#include "NotImplemented.h"
#include "PlatformString.h"

using namespace WTF::Unicode;

namespace WebCore {

int findNextWordFromIndex(const UChar * buffer, int len, int position, bool forward)
{
    notImplemented();
    return 0;
}

void findWordBoundary(const UChar * buffer, int len, int position, int* start, int* end)
{
    if (position > len) {
        *start = 0;
        *end = 0;
        return;
    }

    String str(buffer, len);

    int currentPosition = position - 1;
    String foundWord;
    while (currentPosition >= 0 && isLetter(str[currentPosition])) {
        UChar c = str[currentPosition];
        foundWord.insert(&c, 1, 0);
        --currentPosition;
    }

    // currentPosition == 0 means the first char is not letter
    // currentPosition == -1 means we reached the beginning
    int startPos = (currentPosition < 0) ? 0 : ++currentPosition;
    currentPosition = position;
    while (isLetter(str[currentPosition])) {
        foundWord.append(str[currentPosition]);
        ++currentPosition;
    }

    *start = startPos;
    *end = currentPosition;
}


}
