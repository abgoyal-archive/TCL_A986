

#ifndef BindingSecurity_h
#define BindingSecurity_h

#include "BindingSecurityBase.h"
#include "CSSHelper.h"
#include "Element.h"
#include "GenericBinding.h"
#include "HTMLFrameElementBase.h"

namespace WebCore {

class DOMWindow;
class Frame;
class Node;

// Security functions shared by various language bindings.
template <class Binding>
class BindingSecurity : public BindingSecurityBase {
public:
    // Check if the active execution context can access the target frame.
    static bool canAccessFrame(State<Binding>*, Frame*, bool reportError);

    // Check if it is safe to access the given node from the
    // current security context.
    static bool checkNodeSecurity(State<Binding>*, Node* target);

    static bool allowSettingFrameSrcToJavascriptUrl(State<Binding>*, HTMLFrameElementBase*, String value);
    static bool allowSettingSrcToJavascriptURL(State<Binding>*, Element*, String name, String value);

private:
    explicit BindingSecurity() {}
    ~BindingSecurity();

    // Check if the current DOMWindow's security context can access the target
    // DOMWindow.  This function does not report errors, so most callers should
    // use canAccessFrame instead.
    static bool canAccessWindow(State<Binding>*, DOMWindow* target);
};

// Implementations of templated methods must be in this file.

template <class Binding>
bool BindingSecurity<Binding>::canAccessWindow(State<Binding>* state,
                                               DOMWindow* targetWindow)
{
    DOMWindow* activeWindow = state->getActiveWindow();
    return canAccess(activeWindow, targetWindow);
}

template <class Binding>
bool BindingSecurity<Binding>::canAccessFrame(State<Binding>* state,
                                              Frame* target,
                                              bool reportError)
{
    // The subject is detached from a frame, deny accesses.
    if (!target)
        return false;

    if (!canAccessWindow(state, getDOMWindow(target))) {
        if (reportError)
            state->immediatelyReportUnsafeAccessTo(target);
        return false;
    }
    return true;
}

template <class Binding>
bool BindingSecurity<Binding>::checkNodeSecurity(State<Binding>* state, Node* node)
{
    if (!node)
        return false;

    Frame* target = getFrame(node);

    if (!target)
        return false;

    return canAccessFrame(state, target, true);
}

template <class Binding>
bool BindingSecurity<Binding>::allowSettingFrameSrcToJavascriptUrl(State<Binding>* state, HTMLFrameElementBase* frame, String value)
{
    if (protocolIsJavaScript(deprecatedParseURL(value))) {
        Node* contentDoc = frame->contentDocument();
        if (contentDoc && !checkNodeSecurity(state, contentDoc))
            return false;
    }
    return true;
}

template <class Binding>
bool BindingSecurity<Binding>::allowSettingSrcToJavascriptURL(State<Binding>* state, Element* element, String name, String value)
{
    if ((element->hasTagName(HTMLNames::iframeTag) || element->hasTagName(HTMLNames::frameTag)) && equalIgnoringCase(name, "src"))
        return allowSettingFrameSrcToJavascriptUrl(state, static_cast<HTMLFrameElementBase*>(element), value);
    return true;
}

}

#endif // BindingSecurity_h
