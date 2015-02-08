

#ifndef SVGRootInlineBox_h
#define SVGRootInlineBox_h

#if ENABLE(SVG)
#include "RootInlineBox.h"
#include "SVGCharacterLayoutInfo.h"
#include "SVGRenderSupport.h"

namespace WebCore {

class InlineTextBox;
class RenderSVGRoot;
class SVGInlineTextBox;

struct LastGlyphInfo {
    LastGlyphInfo() : isValid(false) { }

    String unicode;
    String glyphName;
    bool isValid;
};

class SVGRootInlineBox : public RootInlineBox, protected SVGRenderBase {
public:
    SVGRootInlineBox(RenderObject* obj)
        : RootInlineBox(obj)
        , m_height(0)
    {
    }
    virtual const SVGRenderBase* toSVGRenderBase() const { return this; }

    virtual bool isSVGRootInlineBox() { return true; }

    virtual int virtualHeight() const { return m_height; }
    void setHeight(int h) { m_height = h; }
    
    virtual void paint(RenderObject::PaintInfo&, int tx, int ty);

    virtual int placeBoxesHorizontally(int x, int& leftPosition, int& rightPosition, bool& needsWordSpacing);
    virtual int verticallyAlignBoxes(int heightOfBlock);

    virtual void computePerCharacterLayoutInformation();

    virtual FloatRect objectBoundingBox() const { return FloatRect(); }
    virtual FloatRect repaintRectInLocalCoordinates() const { return FloatRect(); }

    // Used by SVGInlineTextBox
    const Vector<SVGTextChunk>& svgTextChunks() const;

    void walkTextChunks(SVGTextChunkWalkerBase*, const SVGInlineTextBox* textBox = 0);

private:
    friend struct SVGRootInlineBoxPaintWalker;

    void layoutInlineBoxes();
    void layoutInlineBoxes(InlineFlowBox* start, Vector<SVGChar>::iterator& it, int& minX, int& maxX, int& minY, int& maxY);

    void buildLayoutInformation(InlineFlowBox* start, SVGCharacterLayoutInfo&);
    void buildLayoutInformationForTextBox(SVGCharacterLayoutInfo&, InlineTextBox*, LastGlyphInfo&);

    void buildTextChunks(Vector<SVGChar>&, Vector<SVGTextChunk>&, InlineFlowBox* start);
    void buildTextChunks(Vector<SVGChar>&, InlineFlowBox* start, SVGTextChunkLayoutInfo&);
    void layoutTextChunks();

    SVGTextDecorationInfo retrievePaintServersForTextDecoration(RenderObject* start);

private:
    int m_height;
    Vector<SVGChar> m_svgChars;
    Vector<SVGTextChunk> m_svgTextChunks;
};

// Shared with SVGRenderTreeAsText / SVGInlineTextBox
TextRun svgTextRunForInlineTextBox(const UChar*, int len, RenderStyle* style, const InlineTextBox* textBox, float xPos);
FloatPoint topLeftPositionOfCharacterRange(Vector<SVGChar>::iterator start, Vector<SVGChar>::iterator end);
float cummulatedWidthOfInlineBoxCharacterRange(SVGInlineBoxCharacterRange& range);
float cummulatedHeightOfInlineBoxCharacterRange(SVGInlineBoxCharacterRange& range);

RenderSVGRoot* findSVGRootObject(RenderObject* start);

} // namespace WebCore

#endif // ENABLE(SVG)

#endif // SVGRootInlineBox_h
