

#ifndef WebAccessibilityObject_h
#define WebAccessibilityObject_h

#include "WebAccessibilityRole.h"
#include "WebCommon.h"

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class AccessibilityObject; }
namespace WTF { template <typename T> class PassRefPtr; }
#endif

namespace WebKit {

class WebAccessibilityObjectPrivate;
class WebString;
struct WebPoint;
struct WebRect;

// A container for passing around a reference to AccessibilityObject.
class WebAccessibilityObject {
public:
    ~WebAccessibilityObject() { reset(); }

    WebAccessibilityObject() : m_private(0) { }
    WebAccessibilityObject(const WebAccessibilityObject& o) : m_private(0) { assign(o); }
    WebAccessibilityObject& operator=(const WebAccessibilityObject& o)
    {
        assign(o);
        return *this;
    }

    WEBKIT_API void reset();
    WEBKIT_API void assign(const WebAccessibilityObject&);

    bool isNull() const { return !m_private; }

    WebString accessibilityDescription() const;
    WebString actionVerb() const;
    bool canSetFocusAttribute() const;
    bool canSetValueAttribute() const;

    unsigned childCount() const;

    WebAccessibilityObject childAt(unsigned) const;
    WebAccessibilityObject firstChild() const;
    WebAccessibilityObject focusedChild() const;
    WebAccessibilityObject lastChild() const;
    WebAccessibilityObject nextSibling() const;
    WebAccessibilityObject parentObject() const;
    WebAccessibilityObject previousSibling() const;

    bool isAnchor() const;
    bool isChecked() const;
    bool isFocused() const;
    bool isEnabled() const;
    bool isHovered() const;
    bool isIndeterminate() const;
    bool isMultiSelectable() const;
    bool isOffScreen() const;
    bool isPasswordField() const;
    bool isPressed() const;
    bool isReadOnly() const;
    bool isVisited() const;

    WebRect boundingBoxRect() const;
    WebString helpText() const;
    WebAccessibilityObject hitTest(const WebPoint&) const;
    WebString keyboardShortcut() const;
    bool performDefaultAction() const;
    WebAccessibilityRole roleValue() const;
    WebString stringValue() const;
    WebString title() const;

#if WEBKIT_IMPLEMENTATION
    WebAccessibilityObject(const WTF::PassRefPtr<WebCore::AccessibilityObject>&);
    WebAccessibilityObject& operator=(const WTF::PassRefPtr<WebCore::AccessibilityObject>&);
    operator WTF::PassRefPtr<WebCore::AccessibilityObject>() const;
#endif

private:
    void assign(WebAccessibilityObjectPrivate*);
    WebAccessibilityObjectPrivate* m_private;
};

} // namespace WebKit

#endif
