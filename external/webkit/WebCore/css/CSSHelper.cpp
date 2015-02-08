

#include "config.h"
#include "CSSHelper.h"

#include "PlatformString.h"
#include <wtf/Vector.h>

namespace WebCore {

String deprecatedParseURL(const String& url)
{
    StringImpl* i = url.impl();
    if (!i)
        return String();

    int o = 0;
    int l = i->length();

    while (o < l && (*i)[o] <= ' ') {
        ++o;
        --l;
    }
    while (l > 0 && (*i)[o + l - 1] <= ' ')
        --l;

    if (l >= 5
            && ((*i)[o] == 'u' || (*i)[o] == 'U')
            && ((*i)[o + 1] == 'r' || (*i)[o + 1] == 'R')
            && ((*i)[o + 2] == 'l' || (*i)[o + 2] == 'L')
            && (*i)[o + 3] == '('
            && (*i)[o + l - 1] == ')') {
        o += 4;
        l -= 5;
    }

    while (o < l && (*i)[o] <= ' ') {
        ++o;
        --l;
    }
    while (l > 0 && (*i)[o + l - 1] <= ' ')
        --l;

    if (l >= 2 && (*i)[o] == (*i)[o + l - 1] && ((*i)[o] == '\'' || (*i)[o] == '\"')) {
        o++;
        l -= 2;
    }

    while (o < l && (*i)[o] <= ' ') {
        ++o;
        --l;
    }
    while (l > 0 && (*i)[o + l - 1] <= ' ')
        --l;

    Vector<UChar, 2048> buffer(l);

    int nl = 0;
    for (int k = o; k < o + l; k++) {
        UChar c = (*i)[k];
        if (c > '\r')
            buffer[nl++] = c;
    }

    return String(buffer.data(), nl);
}

} // namespace WebCore
