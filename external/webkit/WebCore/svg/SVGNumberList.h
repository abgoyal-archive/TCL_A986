

#ifndef SVGNumberList_h
#define SVGNumberList_h

#if ENABLE(SVG)
#include "SVGList.h"
#include <wtf/PassRefPtr.h>

namespace WebCore {

    class String;

    class SVGNumberList : public SVGPODList<float> {
    public:
        static PassRefPtr<SVGNumberList> create(const QualifiedName& attributeName) { return adoptRef(new SVGNumberList(attributeName)); }
        virtual ~SVGNumberList();

        void parse(const String& value);
 
        String valueAsString() const;

    private:
        SVGNumberList(const QualifiedName&);
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
