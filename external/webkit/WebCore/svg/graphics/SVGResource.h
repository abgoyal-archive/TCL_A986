

#ifndef SVGResource_h
#define SVGResource_h

#if ENABLE(SVG)
#include "PlatformString.h"
#include "RenderObject.h"
#include "StringHash.h"

#include <wtf/HashMap.h>
#include <wtf/HashSet.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    class AtomicString; 
    class Document;
    class SVGStyledElement;
    class TextStream;

    enum SVGResourceType {
        // Painting mode
        ClipperResourceType = 0,
        MarkerResourceType,
        ImageResourceType,
        FilterResourceType,
        MaskerResourceType,
        PaintServerResourceType,
        
        // For resource tracking we need to know how many types of resource there are
        _ResourceTypeCount
    };

    // The SVGResource file represent various graphics resources:
    // - Filter resource
    // - Clipper resource
    // - Masker resource
    // - Marker resource
    // - Pattern resource
    // - Linear/Radial gradient resource
    //
    // SVG creates/uses these resources.

    class SVGResource : public RefCounted<SVGResource> {
    public:
        virtual ~SVGResource();
      
        virtual void invalidate();

        void addClient(SVGStyledElement*);
        virtual SVGResourceType resourceType() const = 0;
        
        bool isPaintServer() const { return resourceType() == PaintServerResourceType; }
        bool isFilter() const { return resourceType() == FilterResourceType; }
        bool isClipper() const { return resourceType() == ClipperResourceType; }
        bool isMarker() const { return resourceType() == MarkerResourceType; }
        bool isMasker() const { return resourceType() == MaskerResourceType; }

        virtual TextStream& externalRepresentation(TextStream&) const;

        static void invalidateClients(HashSet<SVGStyledElement*>);
        static void removeClient(SVGStyledElement*);

    protected:
        SVGResource();

    private:
        HashSet<SVGStyledElement*> m_clients;
    };

    SVGResource* getResourceById(Document*, const AtomicString&, const RenderObject*);
    
    TextStream& operator<<(TextStream&, const SVGResource&);

} // namespace WebCore

#endif
#endif // SVGResource_h
