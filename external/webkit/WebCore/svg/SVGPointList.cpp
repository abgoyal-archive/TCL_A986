

#include "config.h"

#if ENABLE(SVG)
#include "SVGPointList.h"
#include "PlatformString.h"

namespace WebCore {

SVGPointList::SVGPointList(const QualifiedName& attributeName)
    : SVGPODList<FloatPoint>(attributeName)
{
}

SVGPointList::~SVGPointList()
{
}

String SVGPointList::valueAsString() const
{
    String result;

    ExceptionCode ec = 0;
    for (unsigned int i = 0; i < numberOfItems(); ++i) {
        if (i > 0)
            result += " ";

        FloatPoint point = getItem(i, ec);
        ASSERT(ec == 0);

        result += String::format("%.6lg %.6lg", point.x(), point.y());
    }

    return result;
}

}

#endif // ENABLE(SVG)
