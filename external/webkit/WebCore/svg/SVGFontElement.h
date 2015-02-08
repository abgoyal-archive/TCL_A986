

#ifndef SVGFontElement_h
#define SVGFontElement_h

#if ENABLE(SVG_FONTS)
#include "SVGExternalResourcesRequired.h"
#include "SVGGlyphElement.h"
#include "SVGGlyphMap.h"
#include "SVGHKernElement.h"
#include "SVGStyledElement.h"

namespace WebCore {

    class SVGMissingGlyphElement;    
    class SVGFontElement : public SVGStyledElement
                         , public SVGExternalResourcesRequired {
    public:
        SVGFontElement(const QualifiedName&, Document*);
        virtual ~SVGFontElement();

        virtual void synchronizeProperty(const QualifiedName&);
        virtual bool rendererIsNeeded(RenderStyle*) { return false; }    

        void invalidateGlyphCache();

        void getGlyphIdentifiersForString(const String&, Vector<SVGGlyphIdentifier>&) const;

        bool getHorizontalKerningPairForStringsAndGlyphs(const String& u1, const String& g1, const String& u2, const String& g2, SVGHorizontalKerningPair& kerningPair) const;

        SVGMissingGlyphElement* firstMissingGlyphElement() const;

    private:
        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGFontElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)

        void ensureGlyphCache() const;

        typedef Vector<SVGHorizontalKerningPair> KerningPairVector;

        mutable KerningPairVector m_kerningPairs;
        mutable SVGGlyphMap m_glyphMap;
        mutable bool m_isGlyphCacheValid;
    };

} // namespace WebCore

#endif // ENABLE(SVG_FONTS)
#endif
