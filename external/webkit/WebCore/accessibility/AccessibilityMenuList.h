

#ifndef AccessibilityMenuList_h
#define AccessibilityMenuList_h

#include "AccessibilityObject.h"
#include "AccessibilityRenderObject.h"

namespace WebCore {

class AccessibilityMenuList;
class AccessibilityMenuListPopup;
class HTMLOptionElement;

class AccessibilityMenuList : public AccessibilityRenderObject {
public:
    static PassRefPtr<AccessibilityMenuList> create(RenderObject* renderer) { return adoptRef(new AccessibilityMenuList(renderer)); }

    virtual bool isCollapsed() const;
    virtual bool press() const;

private:
    AccessibilityMenuList(RenderObject*);

    virtual bool isMenuList() const { return true; }
    virtual AccessibilityRole roleValue() const { return PopUpButtonRole; }
    virtual bool accessibilityIsIgnored() const { return false; }
    virtual bool canSetFocusAttribute() const { return true; }

    virtual void addChildren();
    virtual void childrenChanged();
};

} // namespace WebCore

#endif // AccessibilityMenuList_h
