

#ifndef SVGStringList_h
#define SVGStringList_h

#if ENABLE(SVG)
#include "PlatformString.h"
#include "SVGList.h"

namespace WebCore {

    class SVGStringList : public SVGList<String> {
    public:
        static PassRefPtr<SVGStringList> create(const QualifiedName& attributeName) { return adoptRef(new SVGStringList(attributeName)); }
        virtual ~SVGStringList();

        void reset(const String& str);
        void parse(const String& data, UChar delimiter = ',');
        
    private:
        SVGStringList(const QualifiedName&);
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGStringList_h
