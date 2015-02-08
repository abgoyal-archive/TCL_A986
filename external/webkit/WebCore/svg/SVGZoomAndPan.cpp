

#include "config.h"

#if ENABLE(SVG)
#include "SVGZoomAndPan.h"

#include "MappedAttribute.h"
#include "SVGNames.h"
#include "SVGParserUtilities.h"

namespace WebCore {

SVGZoomAndPan::SVGZoomAndPan()
    : m_zoomAndPan(SVG_ZOOMANDPAN_MAGNIFY)
{
}

SVGZoomAndPan::~SVGZoomAndPan()
{
}

unsigned short SVGZoomAndPan::zoomAndPan() const
{
    return m_zoomAndPan;
}

void SVGZoomAndPan::setZoomAndPan(unsigned short zoomAndPan)
{
    m_zoomAndPan = zoomAndPan;
}

bool SVGZoomAndPan::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == SVGNames::zoomAndPanAttr) {
        const UChar* start = attr->value().characters();
        const UChar* end = start + attr->value().length();
        parseZoomAndPan(start, end);
        return true;
    }

    return false;
}

bool SVGZoomAndPan::isKnownAttribute(const QualifiedName& attrName)
{
    return attrName == SVGNames::zoomAndPanAttr;
}

static const UChar disable[] =  {'d', 'i', 's', 'a', 'b', 'l', 'e'};
static const UChar magnify[] =  {'m', 'a', 'g', 'n', 'i', 'f', 'y'};

bool SVGZoomAndPan::parseZoomAndPan(const UChar*& start, const UChar* end)
{
    if (skipString(start, end, disable, sizeof(disable) / sizeof(UChar)))
        setZoomAndPan(SVG_ZOOMANDPAN_DISABLE);
    else if (skipString(start, end, magnify, sizeof(magnify) / sizeof(UChar)))
        setZoomAndPan(SVG_ZOOMANDPAN_MAGNIFY);
    else
        return false;

    return true;
}

}

#endif // ENABLE(SVG)
