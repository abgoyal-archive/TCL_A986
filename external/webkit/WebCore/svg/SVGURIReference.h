

#ifndef SVGURIReference_h
#define SVGURIReference_h

#if ENABLE(SVG)
#include "SVGElement.h"
#include "XLinkNames.h"

namespace WebCore {

    class MappedAttribute;

    class SVGURIReference {
    public:
        SVGURIReference();
        virtual ~SVGURIReference();

        bool parseMappedAttribute(MappedAttribute*);
        bool isKnownAttribute(const QualifiedName&);

        static String getTarget(const String& url);

    protected:
        virtual void setHrefBaseValue(SVGAnimatedPropertyTraits<String>::PassType) = 0;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGURIReference_h
