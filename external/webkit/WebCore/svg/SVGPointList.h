

#ifndef SVGPointList_h
#define SVGPointList_h

#if ENABLE(SVG)
#include "SVGList.h"
#include "FloatPoint.h"
#include <wtf/PassRefPtr.h>

namespace WebCore {

    class String;

    class SVGPointList : public SVGPODList<FloatPoint> {
    public:
        static PassRefPtr<SVGPointList> create(const QualifiedName& attributeName) { return adoptRef(new SVGPointList(attributeName)); }
        virtual ~SVGPointList();

        String valueAsString() const;

    private:
        SVGPointList(const QualifiedName&);
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
