

#ifndef SVGLocatable_h
#define SVGLocatable_h

#if ENABLE(SVG)

#include "ExceptionCode.h"

namespace WebCore {

class AffineTransform;
class FloatRect;
class SVGElement;

class SVGLocatable {
public:
    SVGLocatable();
    virtual ~SVGLocatable();

    // 'SVGLocatable' functions
    virtual SVGElement* nearestViewportElement() const = 0;
    virtual SVGElement* farthestViewportElement() const = 0;

    virtual FloatRect getBBox() const = 0;
    virtual AffineTransform getCTM() const = 0;
    virtual AffineTransform getScreenCTM() const = 0;
    AffineTransform getTransformToElement(SVGElement*, ExceptionCode&) const;

    static SVGElement* nearestViewportElement(const SVGElement*);
    static SVGElement* farthestViewportElement(const SVGElement*);

protected:
    static FloatRect getBBox(const SVGElement*);
    static AffineTransform getCTM(const SVGElement*);
    static AffineTransform getScreenCTM(const SVGElement*);
};

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGLocatable_h

// vim:ts=4:noet
