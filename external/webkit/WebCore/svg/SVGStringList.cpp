

#include "config.h"

#if ENABLE(SVG)
#include "SVGStringList.h"

#include "SVGParserUtilities.h"

namespace WebCore {

SVGStringList::SVGStringList(const QualifiedName& attributeName)
    : SVGList<String>(attributeName)
{
}

SVGStringList::~SVGStringList()
{
}

void SVGStringList::reset(const String& str)
{
    ExceptionCode ec = 0;

    parse(str, ' ');
    if (numberOfItems() == 0)
        appendItem(String(""), ec); // Create empty string...
}

void SVGStringList::parse(const String& data, UChar delimiter)
{
    // TODO : more error checking/reporting
    ExceptionCode ec = 0;
    clear(ec);

    const UChar* ptr = data.characters();
    const UChar* end = ptr + data.length();
    while (ptr < end) {
        const UChar* start = ptr;
        while (ptr < end && *ptr != delimiter && !isWhitespace(*ptr))
            ptr++;
        if (ptr == start)
            break;
        appendItem(String(start, ptr - start), ec);
        skipOptionalSpacesOrDelimiter(ptr, end, delimiter);
    }
}

}

#endif // ENABLE(SVG)
