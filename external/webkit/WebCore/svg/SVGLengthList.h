

#ifndef SVGLengthList_h
#define SVGLengthList_h

#if ENABLE(SVG)
#include "SVGLength.h"
#include "SVGList.h"

namespace WebCore {

    class SVGLengthList : public SVGPODList<SVGLength> {
    public:
        static PassRefPtr<SVGLengthList> create(const QualifiedName& attributeName) { return adoptRef(new SVGLengthList(attributeName)); }
        virtual ~SVGLengthList();

        void parse(const String& value, SVGLengthMode mode);
 
        String valueAsString() const;

    private:
        SVGLengthList(const QualifiedName&);
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
