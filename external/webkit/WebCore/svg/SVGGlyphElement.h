

#ifndef SVGGlyphElement_h
#define SVGGlyphElement_h

#if ENABLE(SVG_FONTS)
#include "SVGStyledElement.h"

#include <limits>
#include "Path.h"

namespace WebCore {

    class AtomicString;
    class SVGFontData;

    // Describe a SVG <glyph> element
    struct SVGGlyphIdentifier {
        enum Orientation {
            Vertical,
            Horizontal,
            Both
        };

        // SVG Font depends on exactly this order.
        enum ArabicForm {
            None = 0,
            Isolated,
            Terminal,
            Initial,
            Medial
        };

        SVGGlyphIdentifier()
            : isValid(false)
            , orientation(Both)
            , arabicForm(None)
            , priority(0)
            , nameLength(0)
            , horizontalAdvanceX(0.0f)
            , verticalOriginX(0.0f)
            , verticalOriginY(0.0f)
            , verticalAdvanceY(0.0f)
        {
        }

        // Used to mark our float properties as "to be inherited from SVGFontData"
        static float inheritedValue()
        {
            static float s_inheritedValue = std::numeric_limits<float>::infinity();
            return s_inheritedValue;
        }

        bool operator==(const SVGGlyphIdentifier& other) const
        {
            return isValid == other.isValid &&
                   orientation == other.orientation &&
                   arabicForm == other.arabicForm &&
                   glyphName == other.glyphName &&
                   horizontalAdvanceX == other.horizontalAdvanceX &&
                   verticalOriginX == other.verticalOriginX &&
                   verticalOriginY == other.verticalOriginY &&
                   verticalAdvanceY == other.verticalAdvanceY &&
                   pathData.debugString() == other.pathData.debugString() &&
                   languages == other.languages;
        }

        bool isValid : 1;

        unsigned orientation : 2; // Orientation
        unsigned arabicForm : 3;  // ArabicForm
        int priority;
        size_t nameLength;
        String glyphName;

        float horizontalAdvanceX;
        float verticalOriginX;
        float verticalOriginY;
        float verticalAdvanceY;

        Path pathData;
        Vector<String> languages;
    };

    class SVGGlyphElement : public SVGStyledElement {
    public:
        SVGGlyphElement(const QualifiedName&, Document*);
        virtual ~SVGGlyphElement();

        virtual void parseMappedAttribute(MappedAttribute*);

        virtual void insertedIntoDocument();
        virtual void removedFromDocument();

        virtual bool rendererIsNeeded(RenderStyle*) { return false; }

        SVGGlyphIdentifier buildGlyphIdentifier() const;

        // Helper function used by SVGFont
        static void inheritUnspecifiedAttributes(SVGGlyphIdentifier&, const SVGFontData*);
        static String querySVGFontLanguage(const SVGElement*);

        // Helper function shared between SVGGlyphElement & SVGMissingGlyphElement
        static SVGGlyphIdentifier buildGenericGlyphIdentifier(const SVGElement*);
    private:
        void invalidateGlyphCache();
    };

} // namespace WebCore

#endif // ENABLE(SVG_FONTS)
#endif
