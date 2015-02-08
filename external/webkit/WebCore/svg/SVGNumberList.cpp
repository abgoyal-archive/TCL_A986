

#include "config.h"

#if ENABLE(SVG)
#include "SVGNumberList.h"

#include "SVGParserUtilities.h"

namespace WebCore {

SVGNumberList::SVGNumberList(const QualifiedName& attributeName)
    : SVGPODList<float>(attributeName)
{
}

SVGNumberList::~SVGNumberList()
{
}

void SVGNumberList::parse(const String& value)
{
    ExceptionCode ec = 0;
    clear(ec);

    float number = 0.0f;
   
    const UChar* ptr = value.characters();
    const UChar* end = ptr + value.length();
    // The spec strangely doesn't allow leading whitespace.  We might choose to violate that intentionally. (section 4.1)
    while (ptr < end) {
        if (!parseNumber(ptr, end, number))
            return;
        appendItem(number, ec);
    }
}

String SVGNumberList::valueAsString() const
{
    String result;

    ExceptionCode ec = 0;
    for (unsigned int i = 0; i < numberOfItems(); ++i) {
        if (i > 0)
            result += ", ";

        result += String::number(getItem(i, ec));
        ASSERT(ec == 0);
    }

    return result;
}

}

#endif // ENABLE(SVG)
