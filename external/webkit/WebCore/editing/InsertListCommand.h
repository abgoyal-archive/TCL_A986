

#ifndef InsertListCommand_h
#define InsertListCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class HTMLElement;

class InsertListCommand : public CompositeEditCommand {
public:
    enum Type { OrderedList, UnorderedList };

    static PassRefPtr<InsertListCommand> create(Document* document, Type listType)
    {
        return adoptRef(new InsertListCommand(document, listType));
    }

    static PassRefPtr<HTMLElement> insertList(Document*, Type);
    
    virtual bool preservesTypingStyle() const { return true; }

private:
    InsertListCommand(Document*, Type);

    virtual void doApply();
    virtual EditAction editingAction() const { return EditActionInsertList; }

    HTMLElement* fixOrphanedListChild(Node*);
    bool modifyRange();
    RefPtr<HTMLElement> m_listElement;
    Type m_type;
    bool m_forceCreateList;
};

} // namespace WebCore

#endif // InsertListCommand_h
