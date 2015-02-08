

#ifndef InsertTextCommand_h
#define InsertTextCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class InsertTextCommand : public CompositeEditCommand {
public:
    static PassRefPtr<InsertTextCommand> create(Document* document)
    {
        return adoptRef(new InsertTextCommand(document));
    }

    void input(const String& text, bool selectInsertedText = false);

private:
    InsertTextCommand(Document*);

    void deleteCharacter();
    unsigned charactersAdded() const { return m_charactersAdded; }
    
    virtual void doApply();
    virtual bool isInsertTextCommand() const;

    Position prepareForTextInsertion(const Position&);
    Position insertTab(const Position&);
    
    bool performTrivialReplace(const String&, bool selectInsertedText);

    unsigned m_charactersAdded;
};

} // namespace WebCore

#endif // InsertTextCommand_h
