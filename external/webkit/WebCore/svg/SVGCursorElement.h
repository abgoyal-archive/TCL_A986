

#ifndef SVGCursorElement_h
#define SVGCursorElement_h

#if ENABLE(SVG)
#include "SVGLength.h"
#include "SVGElement.h"
#include "SVGTests.h"
#include "SVGURIReference.h"
#include "SVGExternalResourcesRequired.h"

namespace WebCore {

    class SVGCursorElement : public SVGElement,
                             public SVGTests,
                             public SVGExternalResourcesRequired,
                             public SVGURIReference {
    public:
        SVGCursorElement(const QualifiedName&, Document*);
        virtual ~SVGCursorElement();

        void addClient(SVGElement*);
        void removeClient(SVGElement*);

        virtual bool isValid() const { return SVGTests::isValid(); }

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);

        virtual void addSubresourceAttributeURLs(ListHashSet<KURL>&) const;

    private:
        DECLARE_ANIMATED_PROPERTY(SVGCursorElement, SVGNames::xAttr, SVGLength, X, x)
        DECLARE_ANIMATED_PROPERTY(SVGCursorElement, SVGNames::yAttr, SVGLength, Y, y)

        // SVGURIReference
        DECLARE_ANIMATED_PROPERTY(SVGCursorElement, XLinkNames::hrefAttr, String, Href, href)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGCursorElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)

        HashSet<SVGElement*> m_clients;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
