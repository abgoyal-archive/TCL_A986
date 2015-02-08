

#ifndef EditCommand_h
#define EditCommand_h

#include "EditAction.h"
#include "Element.h"
#include "VisibleSelection.h"

namespace WebCore {

class CompositeEditCommand;
class CSSMutableStyleDeclaration;

class EditCommand : public RefCounted<EditCommand> {
public:
    virtual ~EditCommand();

    void setParent(CompositeEditCommand*);

    void apply();
    void unapply();
    void reapply();

    virtual EditAction editingAction() const;

    const VisibleSelection& startingSelection() const { return m_startingSelection; }
    const VisibleSelection& endingSelection() const { return m_endingSelection; }

    Element* startingRootEditableElement() const { return m_startingRootEditableElement.get(); }
    Element* endingRootEditableElement() const { return m_endingRootEditableElement.get(); }
    
    virtual bool isInsertTextCommand() const;
    virtual bool isTypingCommand() const;
    
    virtual bool preservesTypingStyle() const;

protected:
    EditCommand(Document*);

    Document* document() const { return m_document.get(); }

    void setStartingSelection(const VisibleSelection&);
    void setEndingSelection(const VisibleSelection&);

    void updateLayout() const;

private:
    virtual void doApply() = 0;
    virtual void doUnapply() = 0;
    virtual void doReapply(); // calls doApply()

    RefPtr<Document> m_document;
    VisibleSelection m_startingSelection;
    VisibleSelection m_endingSelection;
    RefPtr<Element> m_startingRootEditableElement;
    RefPtr<Element> m_endingRootEditableElement;
    CompositeEditCommand* m_parent;

    friend void applyCommand(PassRefPtr<EditCommand>);
};

class SimpleEditCommand : public EditCommand {
protected:
    SimpleEditCommand(Document* document) : EditCommand(document) { }
};

void applyCommand(PassRefPtr<EditCommand>);

} // namespace WebCore

#endif // EditCommand_h
