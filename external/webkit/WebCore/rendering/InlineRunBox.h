

#ifndef InlineRunBox_h
#define InlineRunBox_h

#include "InlineBox.h"

namespace WebCore {

class InlineRunBox : public InlineBox {
public:
    InlineRunBox(RenderObject* obj)
        : InlineBox(obj)
        , m_prevLine(0)
        , m_nextLine(0)
    {
    }

    InlineRunBox* prevLineBox() const { return m_prevLine; }
    InlineRunBox* nextLineBox() const { return m_nextLine; }
    void setNextLineBox(InlineRunBox* n) { m_nextLine = n; }
    void setPreviousLineBox(InlineRunBox* p) { m_prevLine = p; }

    virtual void paintBoxDecorations(RenderObject::PaintInfo&, int /*tx*/, int /*ty*/) { }
    virtual void paintTextDecorations(RenderObject::PaintInfo&, int /*tx*/, int /*ty*/, bool /*paintedChildren*/ = false) { }
    
protected:
    InlineRunBox* m_prevLine;  // The previous box that also uses our RenderObject
    InlineRunBox* m_nextLine;  // The next box that also uses our RenderObject
};

} // namespace WebCore

#endif // InlineRunBox_h
