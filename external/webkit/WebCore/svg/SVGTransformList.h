

#ifndef SVGTransformList_h
#define SVGTransformList_h

#if ENABLE(SVG)
#include "SVGList.h"
#include "SVGTransform.h"
#include <wtf/PassRefPtr.h>

namespace WebCore {

    class String;

    class SVGTransformList : public SVGPODList<SVGTransform> {
    public:
        static PassRefPtr<SVGTransformList> create(const QualifiedName& attributeName) { return adoptRef(new SVGTransformList(attributeName)); }
        virtual ~SVGTransformList();

        SVGTransform createSVGTransformFromMatrix(const AffineTransform&) const;
        SVGTransform consolidate();

        // Internal use only
        SVGTransform concatenate() const;
 
        String valueAsString() const;

    private:
        SVGTransformList(const QualifiedName&);
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGTransformList_h
