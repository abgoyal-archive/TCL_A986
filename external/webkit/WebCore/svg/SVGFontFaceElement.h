

#ifndef SVGFontFaceElement_h
#define SVGFontFaceElement_h

#if ENABLE(SVG_FONTS)
#include "SVGElement.h"

namespace WebCore {

    class CSSFontFaceRule;
    class CSSMutableStyleDeclaration;
    class SVGFontElement;

    class SVGFontFaceElement : public SVGElement {
    public:
        SVGFontFaceElement(const QualifiedName&, Document*);
        virtual ~SVGFontFaceElement();

        virtual void parseMappedAttribute(MappedAttribute*);

        virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);
        virtual void insertedIntoDocument();
        virtual void removedFromDocument();

        unsigned unitsPerEm() const;
        int xHeight() const;
        float horizontalOriginX() const;
        float horizontalOriginY() const;
        float horizontalAdvanceX() const;
        float verticalOriginX() const;
        float verticalOriginY() const;
        float verticalAdvanceY() const;
        int ascent() const;
        int descent() const;
        String fontFamily() const;

        SVGFontElement* associatedFontElement() const { return m_fontElement; }
        void rebuildFontFace();
        void removeFromMappedElementSheet();

    private:
        RefPtr<CSSFontFaceRule> m_fontFaceRule;
        RefPtr<CSSMutableStyleDeclaration> m_styleDeclaration;

        SVGFontElement* m_fontElement;
    };

} // namespace WebCore

#endif // ENABLE(SVG_FONTS)
#endif

// vim:ts=4:noet
