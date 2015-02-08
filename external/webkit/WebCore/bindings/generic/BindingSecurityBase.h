

#ifndef BindingSecurityBase_h
#define BindingSecurityBase_h

namespace WebCore {

class DOMWindow;
class Frame;
class Node;

// Helper functions for BindingSecurity that depend on WebCore classes, and
// thus should not be implemented in BindingSecurity.h, which contains template
// method definitions.
class BindingSecurityBase {
protected:
    static DOMWindow* getDOMWindow(Frame*);
    static Frame* getFrame(Node*);
    static bool canAccess(DOMWindow* active, DOMWindow* target);
};

}

#endif // BindingSecurityBase_h
