

#include "config.h"

#if ENABLE(SVG)
#include "SVGPolylineElement.h"

#include "SVGPointList.h"

namespace WebCore {

SVGPolylineElement::SVGPolylineElement(const QualifiedName& tagName, Document* doc)
    : SVGPolyElement(tagName, doc)
{
}

SVGPolylineElement::~SVGPolylineElement()
{
}

Path SVGPolylineElement::toPathData() const
{
    Path polyData;

    int len = points()->numberOfItems();
    if (len < 1)
        return polyData;

    ExceptionCode ec = 0;
    polyData.moveTo(points()->getItem(0, ec));

    for (int i = 1; i < len; ++i)
        polyData.addLineTo(points()->getItem(i, ec));
    
    return polyData;
}

}

#endif // ENABLE(SVG)
