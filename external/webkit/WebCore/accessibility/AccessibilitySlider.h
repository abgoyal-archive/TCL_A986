

#ifndef AccessibilitySlider_h
#define AccessibilitySlider_h

#include "AccessibilityRenderObject.h"

namespace WebCore {

class HTMLInputElement;

class AccessibilitySlider : public AccessibilityRenderObject {
    
public:
    static PassRefPtr<AccessibilitySlider> create(RenderObject*);
    virtual ~AccessibilitySlider() { }

    virtual AccessibilityRole roleValue() const { return SliderRole; }
    virtual bool accessibilityIsIgnored() const { return false; }

    virtual bool isSlider() const { return true; }

    virtual const AccessibilityChildrenVector& children();
    virtual void addChildren();

    virtual bool canSetValueAttribute() const { return true; }
    const AtomicString& getAttribute(const QualifiedName& attribute) const;

    virtual void setValue(const String&);
    virtual float valueForRange() const;
    virtual float maxValueForRange() const;
    virtual float minValueForRange() const;
    virtual AccessibilityOrientation orientation() const;

protected:
    AccessibilitySlider(RenderObject*);

private:
    HTMLInputElement* element() const;
};

class AccessibilitySliderThumb : public AccessibilityObject {
    
public:
    static PassRefPtr<AccessibilitySliderThumb> create();
    virtual ~AccessibilitySliderThumb() { }

    virtual AccessibilityRole roleValue() const { return SliderThumbRole; }
    virtual bool accessibilityIsIgnored() const { return false; }

    void setParentObject(AccessibilitySlider* slider) { m_parentSlider = slider; }
    virtual AccessibilityObject* parentObject() const { return m_parentSlider; }

    virtual IntSize size() const;
    virtual IntRect elementRect() const;

private:
    AccessibilitySliderThumb();

    AccessibilitySlider* m_parentSlider;
};


} // namespace WebCore

#endif // AccessibilitySlider_h
