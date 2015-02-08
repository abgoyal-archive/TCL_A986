

#ifndef AccessibilityUIElement_h
#define AccessibilityUIElement_h

#include <JavaScriptCore/JSObjectRef.h>
#include <wtf/Platform.h>
#include <wtf/Vector.h>

#if PLATFORM(MAC)
#ifdef __OBJC__
typedef id PlatformUIElement;
#else
typedef struct objc_object* PlatformUIElement;
#endif
#elif PLATFORM(WIN)
#undef _WINSOCKAPI_
#define _WINSOCKAPI_ // Prevent inclusion of winsock.h in windows.h

#include <oleacc.h>
#include <WebCore/COMPtr.h>

typedef COMPtr<IAccessible> PlatformUIElement;
#elif PLATFORM(GTK)
#include <atk/atk.h>
typedef AtkObject* PlatformUIElement;
#else
typedef void* PlatformUIElement;
#endif

class AccessibilityUIElement {
public:
    AccessibilityUIElement(PlatformUIElement);
    AccessibilityUIElement(const AccessibilityUIElement&);
    ~AccessibilityUIElement();

    PlatformUIElement platformUIElement() { return m_element; }

    static JSObjectRef makeJSAccessibilityUIElement(JSContextRef, const AccessibilityUIElement&);

    bool isEqual(AccessibilityUIElement* otherElement) { return platformUIElement() == otherElement->platformUIElement(); }
    
    void getLinkedUIElements(Vector<AccessibilityUIElement>&);
    void getDocumentLinks(Vector<AccessibilityUIElement>&);
    void getChildren(Vector<AccessibilityUIElement>&);
    void getChildrenWithRange(Vector<AccessibilityUIElement>&, unsigned location, unsigned length);
    
    AccessibilityUIElement elementAtPoint(int x, int y);
    AccessibilityUIElement getChildAtIndex(unsigned);
    unsigned indexOfChild(AccessibilityUIElement*);
    int childrenCount();
    AccessibilityUIElement titleUIElement();
    AccessibilityUIElement parentElement();

    void takeFocus();
    void takeSelection();
    void addSelection();
    void removeSelection();

    // Methods - platform-independent implementations
    JSStringRef allAttributes();
    JSStringRef attributesOfLinkedUIElements();
    JSStringRef attributesOfDocumentLinks();
    JSStringRef attributesOfChildren();
    JSStringRef parameterizedAttributeNames();
    void increment();
    void decrement();
    void showMenu();

    // Attributes - platform-independent implementations
    JSStringRef stringAttributeValue(JSStringRef attribute);
    bool boolAttributeValue(JSStringRef attribute);
    bool isAttributeSupported(JSStringRef attribute);
    bool isAttributeSettable(JSStringRef attribute);
    bool isActionSupported(JSStringRef action);
    JSStringRef role();
    JSStringRef subrole();
    JSStringRef roleDescription();
    JSStringRef title();
    JSStringRef description();
    JSStringRef language();
    JSStringRef stringValue();
    JSStringRef accessibilityValue() const;
    JSStringRef orientation() const;
    double x();
    double y();
    double width();
    double height();
    double intValue() const;
    double minValue();
    double maxValue();
    JSStringRef valueDescription();
    int insertionPointLineNumber();
    JSStringRef selectedTextRange();
    bool isEnabled();
    bool isRequired() const;
    bool isSelected() const;
    bool isSelectable() const;
    bool isMultiSelectable() const;
    bool isExpanded() const;
    bool isChecked() const;
    bool isVisible() const;
    bool isOffScreen() const;
    bool isCollapsed() const;
    bool hasPopup() const;
    int hierarchicalLevel() const;
    double clickPointX();
    double clickPointY();
    JSStringRef documentEncoding();
    JSStringRef documentURI();
    JSStringRef url();

    // Table-specific attributes
    JSStringRef attributesOfColumnHeaders();
    JSStringRef attributesOfRowHeaders();
    JSStringRef attributesOfColumns();
    JSStringRef attributesOfRows();
    JSStringRef attributesOfVisibleCells();
    JSStringRef attributesOfHeader();
    int indexInTable();
    JSStringRef rowIndexRange();
    JSStringRef columnIndexRange();
    
    // Tree/Outline specific attributes
    AccessibilityUIElement selectedRowAtIndex(unsigned);
    AccessibilityUIElement disclosedByRow();
    AccessibilityUIElement disclosedRowAtIndex(unsigned);

    // ARIA specific
    AccessibilityUIElement ariaOwnsElementAtIndex(unsigned);
    AccessibilityUIElement ariaFlowToElementAtIndex(unsigned);

    // ARIA Drag and Drop
    bool ariaIsGrabbed() const;
    // A space concatentated string of all the drop effects.
    JSStringRef ariaDropEffects() const;
    
    // Parameterized attributes
    int lineForIndex(int);
    JSStringRef boundsForRange(unsigned location, unsigned length);
    void setSelectedTextRange(unsigned location, unsigned length);
    JSStringRef stringForRange(unsigned location, unsigned length);
    
    // Table-specific
    AccessibilityUIElement cellForColumnAndRow(unsigned column, unsigned row);

    // Notifications
    // Function callback should take one argument, the name of the notification.
    bool addNotificationListener(JSObjectRef functionCallback);
    
private:
    static JSClassRef getJSClass();

    PlatformUIElement m_element;
    JSObjectRef m_notificationFunctionCallback;
};

#endif // AccessibilityUIElement_h
