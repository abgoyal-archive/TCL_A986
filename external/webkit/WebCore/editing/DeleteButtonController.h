

#ifndef DeleteButtonController_h
#define DeleteButtonController_h

#include "DeleteButton.h"

namespace WebCore {

class DeleteButton;
class Frame;
class HTMLElement;
class RenderObject;
class VisibleSelection;

class DeleteButtonController : public Noncopyable {
public:
    DeleteButtonController(Frame*);

    static const char* const containerElementIdentifier;

    HTMLElement* target() const { return m_target.get(); }
    HTMLElement* containerElement() const { return m_containerElement.get(); }

    void respondToChangedSelection(const VisibleSelection& oldSelection);

    void show(HTMLElement*);
    void hide();

    bool enabled() const { return (m_disableStack == 0); }
    void enable();
    void disable();

    void deleteTarget();

private:
    static const char* const buttonElementIdentifier;
    static const char* const outlineElementIdentifier;

    void createDeletionUI();

    Frame* m_frame;
    RefPtr<HTMLElement> m_target;
    RefPtr<HTMLElement> m_containerElement;
    RefPtr<HTMLElement> m_outlineElement;
    RefPtr<DeleteButton> m_buttonElement;
    bool m_wasStaticPositioned;
    bool m_wasAutoZIndex;
    unsigned m_disableStack;
};

} // namespace WebCore

#endif
