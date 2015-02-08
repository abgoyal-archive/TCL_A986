

#ifndef RenderListMarker_h
#define RenderListMarker_h

#include "RenderBox.h"

namespace WebCore {

class RenderListItem;

String listMarkerText(EListStyleType, int value);

// Used to render the list item's marker.
// The RenderListMarker always has to be a child of a RenderListItem.
class RenderListMarker : public RenderBox {
public:
    RenderListMarker(RenderListItem*);
    virtual ~RenderListMarker();

    virtual void calcPrefWidths();

    const String& text() const { return m_text; }

    bool isInside() const;

private:
    virtual const char* renderName() const { return "RenderListMarker"; }

    virtual bool isListMarker() const { return true; }

    virtual void paint(PaintInfo&, int tx, int ty);

    virtual void layout();

    virtual void imageChanged(WrappedImagePtr, const IntRect* = 0);

    virtual InlineBox* createInlineBox();

    virtual int lineHeight(bool firstLine, bool isRootLineBox = false) const;
    virtual int baselinePosition(bool firstLine, bool isRootLineBox = false) const;

    bool isImage() const;
    bool isText() const { return !isImage(); }

    virtual void setSelectionState(SelectionState);
    virtual IntRect selectionRectForRepaint(RenderBoxModelObject* repaintContainer, bool clipToVisibleContent = true);
    virtual bool canBeSelectionLeaf() const { return true; }

    void updateMargins();

    virtual void styleWillChange(StyleDifference, const RenderStyle* newStyle);
    virtual void styleDidChange(StyleDifference, const RenderStyle* oldStyle);

    IntRect getRelativeMarkerRect();
    IntRect localSelectionRect();

    String m_text;
    RefPtr<StyleImage> m_image;
    RenderListItem* m_listItem;
};

inline RenderListMarker* toRenderListMarker(RenderObject* object)
{
    ASSERT(!object || object->isListMarker());
    return static_cast<RenderListMarker*>(object);
}

inline const RenderListMarker* toRenderListMarker(const RenderObject* object)
{
    ASSERT(!object || object->isListMarker());
    return static_cast<const RenderListMarker*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderListMarker(const RenderListMarker*);

} // namespace WebCore

#endif // RenderListMarker_h
