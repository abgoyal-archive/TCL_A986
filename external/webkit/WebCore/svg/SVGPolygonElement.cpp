

#include "config.h"

#if ENABLE(SVG)
#include "SVGPolygonElement.h"

#include "SVGPointList.h"

namespace WebCore {

SVGPolygonElement::SVGPolygonElement(const QualifiedName& tagName, Document* doc)
    : SVGPolyElement(tagName, doc)
{
}

SVGPolygonElement::~SVGPolygonElement()
{
}

Path SVGPolygonElement::toPathData() const
{
    Path polyData;

    int len = points()->numberOfItems();
    if (len < 1)
        return polyData;
    
    ExceptionCode ec = 0;
    polyData.moveTo(points()->getItem(0, ec));

    for (int i = 1; i < len; ++i)
        polyData.addLineTo(points()->getItem(i, ec));

    polyData.closeSubpath();
    return polyData;
}

}

#endif // ENABLE(SVG)
