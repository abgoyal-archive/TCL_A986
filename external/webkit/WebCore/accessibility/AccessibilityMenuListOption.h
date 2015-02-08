

#ifndef AccessibilityMenuListOption_h
#define AccessibilityMenuListOption_h

#include "AccessibilityObject.h"

namespace WebCore {

class AccessibilityMenuListPopup;
class HTMLElement;

class AccessibilityMenuListOption : public AccessibilityObject {
public:
    static PassRefPtr<AccessibilityMenuListOption> create() { return adoptRef(new AccessibilityMenuListOption); }

    void setElement(HTMLElement*);
    void setParent(AccessibilityMenuListPopup* popup) { m_popup = popup; }

private:
    AccessibilityMenuListOption();

    virtual bool isMenuListOption() const { return true; }

    virtual AccessibilityRole roleValue() const { return MenuListOptionRole; }
    virtual bool canHaveChildren() const { return false; }
    virtual IntSize size() const { return elementRect().size(); }

    virtual Element* actionElement() const;
    virtual AccessibilityObject* parentObject() const;
    virtual bool isEnabled() const;
    virtual bool isVisible() const;
    virtual bool isOffScreen() const;
    virtual bool isSelected() const;
    virtual String nameForMSAA() const;
    virtual void setSelected(bool);
    virtual bool canSetSelectedAttribute() const;
    virtual IntRect elementRect() const;

    RefPtr<HTMLElement> m_element;
    AccessibilityMenuListPopup* m_popup;
};

} // namespace WebCore

#endif // AccessibilityMenuListOption_h
