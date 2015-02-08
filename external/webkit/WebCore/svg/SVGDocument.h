

#ifndef SVGDocument_h
#define SVGDocument_h
#if ENABLE(SVG)

#include "Document.h"
#include "FloatPoint.h"

namespace WebCore {

    class DOMImplementation;
    class SVGElement;
    class SVGSVGElement;

    class SVGDocument : public Document {
    public:
        static PassRefPtr<SVGDocument> create(Frame* frame)
        {
            return adoptRef(new SVGDocument(frame));
        }

        virtual ~SVGDocument();
        
        SVGSVGElement* rootElement() const;
        
        void dispatchZoomEvent(float prevScale, float newScale);
        void dispatchScrollEvent();

        bool zoomAndPanEnabled() const;

        void startPan(const FloatPoint& start);
        void updatePan(const FloatPoint& pos) const;

    private:
        SVGDocument(Frame*);

        virtual bool isSVGDocument() const { return true; }

        virtual bool childShouldCreateRenderer(Node*) const;

        FloatPoint m_translate;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGDocument_h

// vim:ts=4:noet
