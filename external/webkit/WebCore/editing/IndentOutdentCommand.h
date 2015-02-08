

#ifndef IndentOutdentCommand_h
#define IndentOutdentCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class IndentOutdentCommand : public CompositeEditCommand {
public:
    enum EIndentType { Indent, Outdent };
    static PassRefPtr<IndentOutdentCommand> create(Document* document, EIndentType type, int marginInPixels = 0)
    {
        return adoptRef(new IndentOutdentCommand(document, type, marginInPixels));
    }
    
    virtual bool preservesTypingStyle() const { return true; }

private:
    IndentOutdentCommand(Document*, EIndentType, int marginInPixels);

    virtual void doApply();
    virtual EditAction editingAction() const { return m_typeOfAction == Indent ? EditActionIndent : EditActionOutdent; }

    void indentRegion(const VisiblePosition&, const VisiblePosition&);
    void outdentRegion(const VisiblePosition&, const VisiblePosition&);
    void outdentParagraph();
    bool tryIndentingAsListItem(const VisiblePosition&);
    void indentIntoBlockquote(const VisiblePosition&, const VisiblePosition&, RefPtr<Element>&);

    EIndentType m_typeOfAction;
    int m_marginInPixels;
};

} // namespace WebCore

#endif // IndentOutdentCommand_h
