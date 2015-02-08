

#include "config.h"

#if ENABLE(SVG)
#include "SVGTransformList.h"

#include "AffineTransform.h"
#include "SVGSVGElement.h"
#include "SVGTransform.h"

using namespace WebCore;

SVGTransformList::SVGTransformList(const QualifiedName& attributeName)
    : SVGPODList<SVGTransform>(attributeName)
{
}

SVGTransformList::~SVGTransformList()
{
}

SVGTransform SVGTransformList::createSVGTransformFromMatrix(const AffineTransform& matrix) const
{
    return SVGSVGElement::createSVGTransformFromMatrix(matrix);
}

SVGTransform SVGTransformList::consolidate()
{
    ExceptionCode ec = 0;
    return initialize(concatenate(), ec);
}

SVGTransform SVGTransformList::concatenate() const
{
    unsigned int length = numberOfItems();
    if (!length)
        return SVGTransform();
        
    AffineTransform matrix;
    ExceptionCode ec = 0;
    for (unsigned int i = 0; i < length; i++)
        matrix = getItem(i, ec).matrix() * matrix;

    return SVGTransform(matrix);
}

String SVGTransformList::valueAsString() const
{
    // TODO: We may want to build a real transform string, instead of concatting to a matrix(...).
    SVGTransform transform = concatenate();
    if (transform.type() == SVGTransform::SVG_TRANSFORM_MATRIX) {
        AffineTransform matrix = transform.matrix();
        return String::format("matrix(%f %f %f %f %f %f)", matrix.a(), matrix.b(), matrix.c(), matrix.d(), matrix.e(), matrix.f());
    }

    return String();
}

#endif // ENABLE(SVG)
