

#ifndef CSSCursorImageValue_h
#define CSSCursorImageValue_h

#include "CSSImageValue.h"
#include "IntPoint.h"
#include <wtf/HashSet.h>

namespace WebCore {

class Element;
class SVGElement;

class CSSCursorImageValue : public CSSImageValue {
public:
    static PassRefPtr<CSSCursorImageValue> create(const String& url, const IntPoint& hotspot)
    {
        return adoptRef(new CSSCursorImageValue(url, hotspot));
    }

    virtual ~CSSCursorImageValue();

    IntPoint hotspot() const { return m_hotspot; }

    bool updateIfSVGCursorIsUsed(Element*);
    virtual StyleCachedImage* cachedImage(DocLoader*);

#if ENABLE(SVG)
    void removeReferencedElement(SVGElement*);
#endif

private:
    CSSCursorImageValue(const String& url, const IntPoint& hotspot);

    IntPoint m_hotspot;

#if ENABLE(SVG)
    HashSet<SVGElement*> m_referencedElements;
#endif
};

} // namespace WebCore

#endif // CSSCursorImageValue_h
