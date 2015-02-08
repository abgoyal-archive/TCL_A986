

#include "config.h"
#include "BindingSecurityBase.h"

#include "DOMWindow.h"
#include "Frame.h"
#include "SecurityOrigin.h"

namespace WebCore {

DOMWindow* BindingSecurityBase::getDOMWindow(Frame* frame)
{
    return frame->domWindow();
}

Frame* BindingSecurityBase::getFrame(Node* node)
{
    return node->document()->frame();
}

// Same origin policy implementation:
//
// Same origin policy prevents JS code from domain A from accessing JS & DOM
// objects in a different domain B. There are exceptions and several objects
// are accessible by cross-domain code. For example, the window.frames object
// is accessible by code from a different domain, but window.document is not.
//
// The JS binding code sets security check callbacks on a function template,
// and accessing instances of the template calls the callback function.
// The callback function enforces the same origin policy.
//
// Callback functions are expensive. Binding code should use a security token
// string to do fast access checks for the common case where source and target
// are in the same domain. A security token is a string object that represents
// the protocol/url/port of a domain.
//
// There are special cases where security token matching is not enough.
// For example, JS can set its domain to a super domain by calling
// document.setDomain(...). In these cases, the binding code can reset
// a context's security token to its global object so that the fast access
// check will always fail.

// Helper to check if the current execution context can access a target frame.
// First it checks same domain policy using the lexical context.
//
// This is equivalent to KJS::Window::allowsAccessFrom(ExecState*).
bool BindingSecurityBase::canAccess(DOMWindow* activeWindow,
                                    DOMWindow* targetWindow)
{
    ASSERT(targetWindow);

    String message;

    if (activeWindow == targetWindow)
        return true;

    if (!activeWindow)
        return false;

    const SecurityOrigin* activeSecurityOrigin = activeWindow->securityOrigin();
    const SecurityOrigin* targetSecurityOrigin = targetWindow->securityOrigin();

    // We have seen crashes were the security origin of the target has not been
    // initialized. Defend against that.
    if (!targetSecurityOrigin)
        return false;

    if (activeSecurityOrigin->canAccess(targetSecurityOrigin))
        return true;

    // Allow access to a "about:blank" page if the dynamic context is a
    // detached context of the same frame as the blank page.
    if (targetSecurityOrigin->isEmpty() && activeWindow->frame() == targetWindow->frame())
        return true;

    return false;
}

} // namespace WebCore
