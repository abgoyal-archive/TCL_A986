

#ifndef SVGTransformable_h
#define SVGTransformable_h

#if ENABLE(SVG)
#include "PlatformString.h"
#include "SVGLocatable.h"
#include "SVGTransformList.h"

namespace WebCore {
    
class AffineTransform;
class AtomicString;
class SVGTransform;
class QualifiedName;

class SVGTransformable : virtual public SVGLocatable {
public:
    SVGTransformable();
    virtual ~SVGTransformable();

    enum TransformParsingMode {
        ClearList,
        DoNotClearList
    };

    static bool parseTransformAttribute(SVGTransformList*, const AtomicString& transform);
    static bool parseTransformAttribute(SVGTransformList*, const UChar*& ptr, const UChar* end, TransformParsingMode mode = ClearList);
    static bool parseTransformValue(unsigned type, const UChar*& ptr, const UChar* end, SVGTransform&);
    
    AffineTransform getCTM(const SVGElement*) const;
    AffineTransform getScreenCTM(const SVGElement*) const;
    
    virtual AffineTransform animatedLocalTransform() const = 0;

    bool isKnownAttribute(const QualifiedName&);
};

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGTransformable_h
