

#ifndef AccessibilityMenuListPopup_h
#define AccessibilityMenuListPopup_h

#include "AccessibilityObject.h"

namespace WebCore {

class AccessibilityMenuList;
class AccessibilityMenuListOption;
class HTMLElement;

class AccessibilityMenuListPopup : public AccessibilityObject {
public:
    static PassRefPtr<AccessibilityMenuListPopup> create() { return adoptRef(new AccessibilityMenuListPopup); }

    void setMenuList(AccessibilityMenuList*);

    virtual bool isEnabled() const;
    virtual bool isOffScreen() const;

private:
    AccessibilityMenuListPopup();

    virtual bool isMenuListPopup() const { return true; }

    virtual IntRect elementRect() const { return IntRect(); }
    virtual IntSize size() const { return IntSize(); }
    virtual AccessibilityRole roleValue() const { return MenuListPopupRole; }

    virtual bool isVisible() const;
    virtual AccessibilityObject* parentObject() const;
    virtual bool press() const;
    virtual void addChildren();
    virtual void childrenChanged();

    AccessibilityMenuListOption* menuListOptionAccessibilityObject(HTMLElement*) const;

    AccessibilityMenuList* m_menuList;
};

} // namespace WebCore

#endif // AccessibilityMenuListPopup_h
