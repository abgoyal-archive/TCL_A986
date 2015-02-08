

#ifndef SVGUnitTypes_h
#define SVGUnitTypes_h

#if ENABLE(SVG)

#include <wtf/RefCounted.h>

namespace WebCore {

class SVGUnitTypes : public RefCounted<SVGUnitTypes> {
public:
    enum SVGUnitType {
        SVG_UNIT_TYPE_UNKNOWN               = 0,
        SVG_UNIT_TYPE_USERSPACEONUSE        = 1,
        SVG_UNIT_TYPE_OBJECTBOUNDINGBOX     = 2
    };

private:
    SVGUnitTypes() { }
};

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGUnitTypes_h

// vim:ts=4:noet
