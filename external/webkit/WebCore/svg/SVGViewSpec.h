

#ifndef SVGViewSpec_h
#define SVGViewSpec_h

#if ENABLE(SVG)
#include "SVGFitToViewBox.h"
#include "SVGZoomAndPan.h"

#include <wtf/RefPtr.h>

namespace WebCore {

    class SVGElement;
    class SVGSVGElement;
    class SVGTransformList;

    class SVGViewSpec : public SVGFitToViewBox,
                        public SVGZoomAndPan,
                        public Noncopyable {
    public:
        SVGViewSpec(const SVGSVGElement*);
        virtual ~SVGViewSpec();

        bool parseViewSpec(const String&);

        void setTransform(const String&);
        SVGTransformList* transform() const { return m_transform.get(); }

        void setViewBoxString(const String&);

        void setPreserveAspectRatioString(const String&);

        void setViewTargetString(const String&);
        String viewTargetString() const { return m_viewTargetString; }
        SVGElement* viewTarget() const;

        SVGSVGElement* contextElement() const { return const_cast<SVGSVGElement*>(m_contextElement); }

    private:
        const SVGSVGElement* m_contextElement;

        // SVGFitToViewBox
        DECLARE_ANIMATED_PROPERTY(SVGViewSpec, SVGNames::viewBoxAttr, FloatRect, ViewBox, viewBox)
        DECLARE_ANIMATED_PROPERTY(SVGViewSpec, SVGNames::preserveAspectRatioAttr, SVGPreserveAspectRatio, PreserveAspectRatio, preserveAspectRatio)

        mutable RefPtr<SVGTransformList> m_transform;
        String m_viewTargetString;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
