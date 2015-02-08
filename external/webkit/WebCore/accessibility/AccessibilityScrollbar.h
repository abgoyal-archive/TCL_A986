
 
#ifndef AccessibilityScrollbar_h
#define AccessibilityScrollbar_h

#include "AccessibilityObject.h"

namespace WebCore {

class Scrollbar;

class AccessibilityScrollbar : public AccessibilityObject {
public:
    static PassRefPtr<AccessibilityScrollbar> create();

    void setScrollbar(Scrollbar* scrollbar) { m_scrollbar = scrollbar; }

    virtual AccessibilityRole roleValue() const { return ScrollBarRole; }

    virtual float valueForRange() const;

private:
    AccessibilityScrollbar();

    virtual bool accessibilityIsIgnored() const { return false; }

    // These should never be reached since the AccessibilityScrollbar is not part of
    // the accessibility tree.
    virtual IntSize size() const { ASSERT_NOT_REACHED(); return IntSize(); }
    virtual IntRect elementRect() const { ASSERT_NOT_REACHED(); return IntRect(); }
    virtual AccessibilityObject* parentObject() const { ASSERT_NOT_REACHED(); return 0; }

    Scrollbar* m_scrollbar;
};

} // namespace WebCore

#endif // AccessibilityScrollbar_h
