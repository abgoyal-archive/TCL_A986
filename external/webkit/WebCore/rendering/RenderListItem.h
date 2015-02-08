

#ifndef RenderListItem_h
#define RenderListItem_h

#include "RenderBlock.h"

namespace WebCore {

class RenderListMarker;

class RenderListItem : public RenderBlock {
public:
    RenderListItem(Node*);

    int value() const { if (!m_isValueUpToDate) updateValueNow(); return m_value; }
    void updateValue();

    bool hasExplicitValue() const { return m_hasExplicitValue; }
    int explicitValue() const { return m_explicitValue; }
    void setExplicitValue(int value);
    void clearExplicitValue();

    void setNotInList(bool notInList) { m_notInList = notInList; }
    bool notInList() const { return m_notInList; }

    const String& markerText() const;

private:
    virtual const char* renderName() const { return "RenderListItem"; }

    virtual bool isListItem() const { return true; }
    
    virtual void destroy();

    virtual bool isEmpty() const;
    virtual void paint(PaintInfo&, int tx, int ty);

    virtual void layout();
    virtual void calcPrefWidths();

    virtual void positionListMarker();

    virtual void styleDidChange(StyleDifference, const RenderStyle* oldStyle);

    void updateMarkerLocation();
    inline int calcValue() const;
    void updateValueNow() const;
    void explicitValueChanged();

    RenderListMarker* m_marker;
    int m_explicitValue;
    mutable int m_value;

    bool m_hasExplicitValue : 1;
    mutable bool m_isValueUpToDate : 1;
    bool m_notInList : 1;
};

inline RenderListItem* toRenderListItem(RenderObject* object)
{
    ASSERT(!object || object->isListItem());
    return static_cast<RenderListItem*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderListItem(const RenderListItem*);

} // namespace WebCore

#endif // RenderListItem_h
