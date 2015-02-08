

#ifndef SVGPathSegList_h
#define SVGPathSegList_h

#if ENABLE(SVG)
#include "SVGList.h"
#include "SVGPathSeg.h"

namespace WebCore {

    class Path;
    class SVGElement;
 
    class SVGPathSegList : public SVGList<RefPtr<SVGPathSeg> > {
    public:
        static PassRefPtr<SVGPathSegList> create(const QualifiedName& attributeName) { return adoptRef(new SVGPathSegList(attributeName)); }
        virtual ~SVGPathSegList();

        unsigned getPathSegAtLength(double, ExceptionCode&);
        Path toPathData();
        
        static PassRefPtr<SVGPathSegList> createAnimated(const SVGPathSegList* fromList, const SVGPathSegList* toList, float progress);
        
    private:
        SVGPathSegList(const QualifiedName&);
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
