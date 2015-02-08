


#ifndef AccessibilityMediaControls_h
#define AccessibilityMediaControls_h

#if ENABLE(VIDEO)

#include "AccessibilitySlider.h"
#include "MediaControlElements.h"

namespace WebCore {

class AccessibilityMediaControl : public AccessibilityRenderObject {

public:
    static PassRefPtr<AccessibilityObject> create(RenderObject*);
    virtual ~AccessibilityMediaControl() { }

    virtual AccessibilityRole roleValue() const;
    virtual bool accessibilityIsIgnored() const;

    virtual String title() const;
    virtual String accessibilityDescription() const;
    virtual String helpText() const;

protected:
    AccessibilityMediaControl(RenderObject*);
    MediaControlElementType controlType() const;
    String controlTypeName() const;
};


class AccessibilityMediaTimeline : public AccessibilitySlider {

public:
    static PassRefPtr<AccessibilityObject> create(RenderObject*);
    virtual ~AccessibilityMediaTimeline() { }

    virtual bool isMediaTimeline() const { return true; }

    virtual String helpText() const;
    virtual String valueDescription() const;
    const AtomicString& getAttribute(const QualifiedName& attribute) const;

private:
    AccessibilityMediaTimeline(RenderObject*);
};


class AccessibilityMediaControlsContainer : public AccessibilityMediaControl {

public:
    static PassRefPtr<AccessibilityObject> create(RenderObject*);
    virtual ~AccessibilityMediaControlsContainer() { }

    virtual AccessibilityRole roleValue() const { return ToolbarRole; }
    virtual bool accessibilityIsIgnored() const { return false; }

    virtual String helpText() const;
    virtual String accessibilityDescription() const;

private:
    AccessibilityMediaControlsContainer(RenderObject*);
    bool controllingVideoElement() const;
    const String elementTypeName() const;
};


class AccessibilityMediaTimeDisplay : public AccessibilityMediaControl {

public:
    static PassRefPtr<AccessibilityObject> create(RenderObject*);
    virtual ~AccessibilityMediaTimeDisplay() { }

    virtual AccessibilityRole roleValue() const { return StaticTextRole; }
    virtual bool accessibilityIsIgnored() const;

    virtual String stringValue() const;
    virtual String accessibilityDescription() const;

private:
    AccessibilityMediaTimeDisplay(RenderObject*);
};


} // namespace WebCore

#endif // ENABLE(VIDEO)

#endif // AccessibilityMediaControls_h
