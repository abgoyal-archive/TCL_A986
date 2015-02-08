

#ifndef InlineTextBox_h
#define InlineTextBox_h

#include "InlineRunBox.h"
#include "RenderText.h" // so textRenderer() can be inline

namespace WebCore {

struct CompositionUnderline;

const unsigned short cNoTruncation = USHRT_MAX;
const unsigned short cFullTruncation = USHRT_MAX - 1;

// Helper functions shared by InlineTextBox / SVGRootInlineBox
void updateGraphicsContext(GraphicsContext*, const Color& fillColor, const Color& strokeColor, float strokeThickness, ColorSpace);
Color correctedTextColor(Color textColor, Color backgroundColor);

class InlineTextBox : public InlineRunBox {
public:
    InlineTextBox(RenderObject* obj)
        : InlineRunBox(obj)
        , m_start(0)
        , m_len(0)
        , m_truncation(cNoTruncation)
    {
    }

    InlineTextBox* nextTextBox() const { return static_cast<InlineTextBox*>(nextLineBox()); }
    InlineTextBox* prevTextBox() const { return static_cast<InlineTextBox*>(prevLineBox()); }

    unsigned start() const { return m_start; }
    unsigned end() const { return m_len ? m_start + m_len - 1 : m_start; }
    unsigned len() const { return m_len; }

    void setStart(unsigned start) { m_start = start; }
    void setLen(unsigned len) { m_len = len; }

    void offsetRun(int d) { m_start += d; }

    void setFallbackFonts(const HashSet<const SimpleFontData*>&);
    void takeFallbackFonts(Vector<const SimpleFontData*>&);

    unsigned short truncation() { return m_truncation; }

private:
    virtual int selectionTop();
    virtual int selectionHeight();

public:
    virtual IntRect selectionRect(int absx, int absy, int startPos, int endPos);
    bool isSelected(int startPos, int endPos) const;
    void selectionStartEnd(int& sPos, int& ePos);

private:
    virtual void paint(RenderObject::PaintInfo&, int tx, int ty);
    virtual bool nodeAtPoint(const HitTestRequest&, HitTestResult&, int x, int y, int tx, int ty);

public:
    RenderText* textRenderer() const;

private:
    virtual void deleteLine(RenderArena*);
    virtual void extractLine();
    virtual void attachLine();

public:
    virtual RenderObject::SelectionState selectionState();

private:
    virtual void clearTruncation() { m_truncation = cNoTruncation; }
    virtual int placeEllipsisBox(bool flowIsLTR, int visibleLeftEdge, int visibleRightEdge, int ellipsisWidth, bool& foundBox);

public:
    virtual bool isLineBreak() const;

    void setSpaceAdd(int add) { m_width -= m_toAdd; m_toAdd = add; m_width += m_toAdd; }

private:
    virtual bool isInlineTextBox() { return true; }    

public:
    virtual int caretMinOffset() const;
    virtual int caretMaxOffset() const;

private:
    virtual unsigned caretMaxRenderedOffset() const;

    int textPos() const;

public:
    virtual int offsetForPosition(int x, bool includePartialGlyphs = true) const;
    virtual int positionForOffset(int offset) const;

    bool containsCaretOffset(int offset) const; // false for offset after line break

private:
    int m_start;
    unsigned short m_len;

    unsigned short m_truncation; // Where to truncate when text overflow is applied.  We use special constants to
                      // denote no truncation (the whole run paints) and full truncation (nothing paints at all).

protected:
    void paintCompositionBackground(GraphicsContext*, int tx, int ty, RenderStyle*, const Font&, int startPos, int endPos);
    void paintDocumentMarkers(GraphicsContext*, int tx, int ty, RenderStyle*, const Font&, bool background);
    void paintCompositionUnderline(GraphicsContext*, int tx, int ty, const CompositionUnderline&);
#if PLATFORM(MAC)
    void paintCustomHighlight(int tx, int ty, const AtomicString& type);
#endif

private:
    void paintDecoration(GraphicsContext*, int tx, int ty, int decoration, ShadowData*);
    void paintSelection(GraphicsContext*, int tx, int ty, RenderStyle*, const Font&);
    void paintSpellingOrGrammarMarker(GraphicsContext*, int tx, int ty, const DocumentMarker&, RenderStyle*, const Font&, bool grammar);
    void paintTextMatchMarker(GraphicsContext*, int tx, int ty, const DocumentMarker&, RenderStyle*, const Font&);
    void computeRectForReplacementMarker(int tx, int ty, const DocumentMarker&, RenderStyle*, const Font&);
};

inline RenderText* InlineTextBox::textRenderer() const
{
    return toRenderText(renderer());
}

} // namespace WebCore

#endif // InlineTextBox_h
