

#ifndef FocusController_h
#define FocusController_h

#include "FocusDirection.h"
#include <wtf/Forward.h>
#include <wtf/Noncopyable.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class Frame;
class KeyboardEvent;
class Node;
class Page;

class FocusController : public Noncopyable {
public:
    FocusController(Page*);

    void setFocusedFrame(PassRefPtr<Frame>);
    Frame* focusedFrame() const { return m_focusedFrame.get(); }
    Frame* focusedOrMainFrame();

    bool setInitialFocus(FocusDirection, KeyboardEvent*);
    bool advanceFocus(FocusDirection, KeyboardEvent*, bool initialFocus = false);
        
    bool setFocusedNode(Node*, PassRefPtr<Frame>);

    void setActive(bool);
    bool isActive() const { return m_isActive; }

    void setFocused(bool);
    bool isFocused() const { return m_isFocused; }

private:
    Page* m_page;
    RefPtr<Frame> m_focusedFrame;
    bool m_isActive;
    bool m_isFocused;
    bool m_isChangingFocusedFrame;
};

} // namespace WebCore
    
#endif // FocusController_h
