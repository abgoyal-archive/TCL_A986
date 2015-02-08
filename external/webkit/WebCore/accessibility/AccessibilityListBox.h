

#ifndef AccessibilityListBox_h
#define AccessibilityListBox_h

#include "AccessibilityObject.h"
#include "AccessibilityRenderObject.h"

namespace WebCore {
    
class AccessibilityListBox : public AccessibilityRenderObject {

private:
    AccessibilityListBox(RenderObject*);
public:
    static PassRefPtr<AccessibilityListBox> create(RenderObject*);
    virtual ~AccessibilityListBox();
    
    virtual AccessibilityObject* doAccessibilityHitTest(const IntPoint&) const;
    virtual bool isListBox() const { return true; }
    
    virtual bool canSetFocusAttribute() const { return true; }
    virtual bool canSetSelectedChildrenAttribute() const;
    void setSelectedChildren(AccessibilityChildrenVector&);
    virtual AccessibilityRole roleValue() const { return ListBoxRole; }
    
    virtual bool accessibilityIsIgnored() const { return false; }
    
    virtual void selectedChildren(AccessibilityChildrenVector&);
    virtual void visibleChildren(AccessibilityChildrenVector&);
    
    virtual void addChildren();

private:    
    AccessibilityObject* listBoxOptionAccessibilityObject(HTMLElement*) const;
};
    
} // namespace WebCore

#endif // AccessibilityListBox_h
