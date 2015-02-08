

#ifndef PrintContext_h
#define PrintContext_h

#include <wtf/Vector.h>

namespace WebCore {

class Element;
class Frame;
class FloatRect;
class FloatSize;
class GraphicsContext;
class IntRect;

class PrintContext {
public:
    PrintContext(Frame*);
    ~PrintContext();

    int pageCount() const;
    const IntRect& pageRect(int pageNumber) const;
    const Vector<IntRect>& pageRects() const { return m_pageRects; }

    void computePageRects(const FloatRect& printRect, float headerHeight, float footerHeight, float userScaleFactor, float& outPageHeight);

    // TODO: eliminate width param
    void begin(float width);

    // TODO: eliminate width param
    void spoolPage(GraphicsContext& ctx, int pageNumber, float width);

    void end();

    // Used by layout tests.
    static int pageNumberForElement(Element*, const FloatSize& pageSizeInPixels);
    static int numberOfPages(Frame*, const FloatSize& pageSizeInPixels);

protected:
    void computePageRectsWithPageSize(const FloatSize& pageSizeInPixels, float userScaleFactor);

    Frame* m_frame;
    Vector<IntRect> m_pageRects;
};

}

#endif
