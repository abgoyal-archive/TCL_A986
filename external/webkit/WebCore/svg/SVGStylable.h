

#ifndef SVGStylable_h
#define SVGStylable_h

#if ENABLE(SVG)
#include <wtf/PassRefPtr.h>

namespace WebCore {

    class CSSValue;
    class CSSStyleDeclaration;
    class String;
    class QualifiedName;

    class SVGStylable {
    public:
        SVGStylable();
        virtual ~SVGStylable();

        virtual CSSStyleDeclaration* style() = 0;
        virtual PassRefPtr<CSSValue> getPresentationAttribute(const String&) = 0;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGStylable_h
