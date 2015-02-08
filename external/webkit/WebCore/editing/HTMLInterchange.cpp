

#include "config.h"
#include "HTMLInterchange.h"

#include "CharacterNames.h"
#include "Text.h"
#include "TextIterator.h"
#include <wtf/StdLibExtras.h>

namespace WebCore {

namespace {

String convertedSpaceString()
{
    DEFINE_STATIC_LOCAL(String, convertedSpaceString, ());
    if (convertedSpaceString.isNull()) {
        convertedSpaceString = "<span class=\"";
        convertedSpaceString += AppleConvertedSpace;
        convertedSpaceString += "\">";
        convertedSpaceString.append(noBreakSpace);
        convertedSpaceString += "</span>";
    }
    return convertedSpaceString;
}

} // end anonymous namespace

String convertHTMLTextToInterchangeFormat(const String& in, const Text* node)
{
    // Assume all the text comes from node.
    if (node->renderer() && node->renderer()->style()->preserveNewline())
        return in;

    Vector<UChar> s;

    unsigned i = 0;
    unsigned consumed = 0;
    while (i < in.length()) {
        consumed = 1;
        if (isCollapsibleWhitespace(in[i])) {
            // count number of adjoining spaces
            unsigned j = i + 1;
            while (j < in.length() && isCollapsibleWhitespace(in[j]))
                j++;
            unsigned count = j - i;
            consumed = count;
            while (count) {
                unsigned add = count % 3;
                switch (add) {
                    case 0:
                        append(s, convertedSpaceString());
                        s.append(' ');
                        append(s, convertedSpaceString());
                        add = 3;
                        break;
                    case 1:
                        if (i == 0 || i + 1 == in.length()) // at start or end of string
                            append(s, convertedSpaceString());
                        else
                            s.append(' ');
                        break;
                    case 2:
                        if (i == 0) {
                             // at start of string
                            append(s, convertedSpaceString());
                            s.append(' ');
                        } else if (i + 2 == in.length()) {
                             // at end of string
                            append(s, convertedSpaceString());
                            append(s, convertedSpaceString());
                        } else {
                            append(s, convertedSpaceString());
                            s.append(' ');
                        }
                        break;
                }
                count -= add;
            }
        } else
            s.append(in[i]);
        i += consumed;
    }

    return String::adopt(s);
}

} // namespace WebCore
