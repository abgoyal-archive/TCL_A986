

#ifndef AccessibilityListBoxOption_h
#define AccessibilityListBoxOption_h

#include "AccessibilityObject.h"
#include "HTMLElement.h"

namespace WebCore {

class AccessibilityListBox;
class Element;
class HTMLElement;
class HTMLSelectElement;
class String;
    
class AccessibilityListBoxOption : public AccessibilityObject {

private:
    AccessibilityListBoxOption();
public:
    static PassRefPtr<AccessibilityListBoxOption> create();
    virtual ~AccessibilityListBoxOption();
    
    void setHTMLElement(HTMLElement* element) { m_optionElement = element; }
    
    virtual AccessibilityRole roleValue() const { return ListBoxOptionRole; }
    virtual bool accessibilityIsIgnored() const { return false; }
    virtual bool isSelected() const;
    virtual bool isEnabled() const;
    virtual String stringValue() const;
    virtual Element* actionElement() const;
    
    virtual void setSelected(bool);
    virtual bool canSetSelectedAttribute() const;

    virtual IntRect elementRect() const;
    virtual IntSize size() const;
    virtual AccessibilityObject* parentObject() const;
    bool isListBoxOption() const { return true; }
    
private:
    HTMLElement* m_optionElement;
    
    virtual bool canHaveChildren() const { return false; }
    HTMLSelectElement* listBoxOptionParentNode() const;
    int listBoxOptionIndex() const;
    IntRect listBoxOptionRect() const;
    AccessibilityObject* listBoxOptionAccessibilityObject(HTMLElement* element) const;
};
    
} // namespace WebCore 

#endif // AccessibilityListBoxOption_h
