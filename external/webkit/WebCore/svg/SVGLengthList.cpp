

#include "config.h"

#if ENABLE(SVG)
#include "SVGLengthList.h"

#include "SVGParserUtilities.h"

namespace WebCore {

SVGLengthList::SVGLengthList(const QualifiedName& attributeName)
    : SVGPODList<SVGLength>(attributeName)
{
}

SVGLengthList::~SVGLengthList()
{
}

void SVGLengthList::parse(const String& value, SVGLengthMode mode)
{
    ExceptionCode ec = 0;
    clear(ec);

    const UChar* ptr = value.characters();
    const UChar* end = ptr + value.length();
    while (ptr < end) {
        const UChar* start = ptr;
        while (ptr < end && *ptr != ',' && !isWhitespace(*ptr))
            ptr++;
        if (ptr == start)
            break;
        SVGLength length(mode);
        if (!length.setValueAsString(String(start, ptr - start)))
            return;
        appendItem(length, ec);
        skipOptionalSpacesOrDelimiter(ptr, end);
    }
}

String SVGLengthList::valueAsString() const
{
    String result;

    ExceptionCode ec = 0;
    for (unsigned int i = 0; i < numberOfItems(); ++i) {
        if (i > 0)
            result += ", ";

        result += getItem(i, ec).valueAsString();
        ASSERT(ec == 0);
    }

    return result;
}

}

#endif // ENABLE(SVG)
