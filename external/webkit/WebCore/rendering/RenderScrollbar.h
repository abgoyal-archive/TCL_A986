

#ifndef RenderScrollbar_h
#define RenderScrollbar_h

#include "RenderStyleConstants.h"
#include "Scrollbar.h"
#include <wtf/HashMap.h>

namespace WebCore {

class RenderBox;
class RenderScrollbarPart;
class RenderStyle;

class RenderScrollbar : public Scrollbar {
protected:
    RenderScrollbar(ScrollbarClient*, ScrollbarOrientation, RenderBox*);

public:
    friend class Scrollbar;
    static PassRefPtr<Scrollbar> createCustomScrollbar(ScrollbarClient*, ScrollbarOrientation, RenderBox*);
    virtual ~RenderScrollbar();

    static ScrollbarPart partForStyleResolve();
    static RenderScrollbar* scrollbarForStyleResolve();

    RenderBox* owningRenderer() const { return m_owner; }

    void paintPart(GraphicsContext*, ScrollbarPart, const IntRect&);

    IntRect buttonRect(ScrollbarPart);
    IntRect trackRect(int startLength, int endLength);
    IntRect trackPieceRectWithMargins(ScrollbarPart, const IntRect&);

    int minimumThumbLength();

private:
    virtual void setParent(ScrollView*);
    virtual void setEnabled(bool);

    virtual void paint(GraphicsContext*, const IntRect& damageRect);

    virtual void setHoveredPart(ScrollbarPart);
    virtual void setPressedPart(ScrollbarPart);

    virtual void styleChanged();

    virtual bool isCustomScrollbar() const { return true; }

    void updateScrollbarParts(bool destroy = false);

    PassRefPtr<RenderStyle> getScrollbarPseudoStyle(ScrollbarPart, PseudoId);
    void updateScrollbarPart(ScrollbarPart, bool destroy = false);

    RenderBox* m_owner;
    HashMap<unsigned, RenderScrollbarPart*> m_parts;
};

inline RenderScrollbar* toRenderScrollbar(Scrollbar* scrollbar)
{
    ASSERT(!scrollbar || scrollbar->isCustomScrollbar());
    return static_cast<RenderScrollbar*>(scrollbar);
}

// This will catch anyone doing an unnecessary cast.
void toRenderScrollbar(const RenderScrollbar*);

} // namespace WebCore

#endif // RenderScrollbar_h
