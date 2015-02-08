

#ifndef SVGInlineTextBox_h
#define SVGInlineTextBox_h

#if ENABLE(SVG)
#include "InlineTextBox.h"

namespace WebCore {

    class SVGRootInlineBox;

    struct SVGChar;
    struct SVGTextDecorationInfo;

    enum SVGTextPaintSubphase {
        SVGTextPaintSubphaseBackground,
        SVGTextPaintSubphaseGlyphFill,
        SVGTextPaintSubphaseGlyphFillSelection,
        SVGTextPaintSubphaseGlyphStroke,
        SVGTextPaintSubphaseGlyphStrokeSelection,
        SVGTextPaintSubphaseForeground
    };

    struct SVGTextPaintInfo {
        SVGTextPaintInfo() : activePaintServer(0), subphase(SVGTextPaintSubphaseBackground) {}

        SVGPaintServer* activePaintServer;
        SVGTextPaintSubphase subphase;
    };

    class SVGInlineTextBox : public InlineTextBox {
    public:
        SVGInlineTextBox(RenderObject* obj);

        virtual int virtualHeight() const { return m_height; }
        void setHeight(int h) { m_height = h; }

        virtual int selectionTop();
        virtual int selectionHeight();

        virtual int offsetForPosition(int x, bool includePartialGlyphs = true) const;
        virtual int positionForOffset(int offset) const;

        virtual bool nodeAtPoint(const HitTestRequest&, HitTestResult&, int x, int y, int tx, int ty);
        virtual IntRect selectionRect(int absx, int absy, int startPos, int endPos);

        // SVGs custom paint text method
        void paintCharacters(RenderObject::PaintInfo&, int tx, int ty, const SVGChar&, const UChar* chars, int length, SVGTextPaintInfo&);

        // SVGs custom paint selection method
        void paintSelection(int boxStartOffset, const SVGChar&, const UChar*, int length, GraphicsContext*, RenderStyle*, const Font&);

        // SVGs custom paint decoration method
        void paintDecoration(ETextDecoration, GraphicsContext*, int tx, int ty, int width, const SVGChar&, const SVGTextDecorationInfo&);
 
        SVGRootInlineBox* svgRootInlineBox() const;

        // Helper functions shared with SVGRootInlineBox     
        float calculateGlyphWidth(RenderStyle* style, int offset, int extraCharsAvailable, int& charsConsumed, String& glyphName) const;
        float calculateGlyphHeight(RenderStyle*, int offset, int extraCharsAvailable) const;

        FloatRect calculateGlyphBoundaries(RenderStyle*, int offset, const SVGChar&) const;
        SVGChar* closestCharacterToPosition(int x, int y, int& offset) const;

    private:
        friend class RenderSVGInlineText;
        bool svgCharacterHitsPosition(int x, int y, int& offset) const;
        bool chunkSelectionStartEnd(const UChar* chunk, int chunkLength, int& selectionStart, int& selectionEnd);
        
        int m_height;
    };

} // namespace WebCore

#endif
#endif // SVGInlineTextBox_h
