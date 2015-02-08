

#ifndef TypingCommand_h
#define TypingCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class TypingCommand : public CompositeEditCommand {
public:
    enum ETypingCommand { 
        DeleteSelection,
        DeleteKey, 
        ForwardDeleteKey, 
        InsertText, 
        InsertLineBreak, 
        InsertParagraphSeparator,
        InsertParagraphSeparatorInQuotedContent
    };

    static void deleteSelection(Document*, bool smartDelete = false);
    static void deleteKeyPressed(Document*, bool smartDelete = false, TextGranularity = CharacterGranularity, bool killRing = false);
    static void forwardDeleteKeyPressed(Document*, bool smartDelete = false, TextGranularity = CharacterGranularity, bool killRing = false);
    static void insertText(Document*, const String&, bool selectInsertedText = false, bool insertedTextIsComposition = false);
    static void insertText(Document*, const String&, const VisibleSelection&, bool selectInsertedText = false, bool insertedTextIsComposition = false);
    static void insertLineBreak(Document*);
    static void insertParagraphSeparator(Document*);
    static void insertParagraphSeparatorInQuotedContent(Document*);
    static bool isOpenForMoreTypingCommand(const EditCommand*);
    static void closeTyping(EditCommand*);
    
    bool isOpenForMoreTyping() const { return m_openForMoreTyping; }
    void closeTyping() { m_openForMoreTyping = false; }

    void insertText(const String &text, bool selectInsertedText);
    void insertTextRunWithoutNewlines(const String &text, bool selectInsertedText);
    void insertLineBreak();
    void insertParagraphSeparatorInQuotedContent();
    void insertParagraphSeparator();
    void deleteKeyPressed(TextGranularity, bool killRing);
    void forwardDeleteKeyPressed(TextGranularity, bool killRing);
    void deleteSelection(bool smartDelete);

private:
    static PassRefPtr<TypingCommand> create(Document* document, ETypingCommand command, const String& text = "", bool selectInsertedText = false, TextGranularity granularity = CharacterGranularity, bool killRing = false)
    {
        return adoptRef(new TypingCommand(document, command, text, selectInsertedText, granularity, killRing));
    }

    TypingCommand(Document*, ETypingCommand, const String& text, bool selectInsertedText, TextGranularity, bool killRing);

    bool smartDelete() const { return m_smartDelete; }
    void setSmartDelete(bool smartDelete) { m_smartDelete = smartDelete; }
    
    virtual void doApply();
    virtual EditAction editingAction() const;
    virtual bool isTypingCommand() const;
    virtual bool preservesTypingStyle() const { return m_preservesTypingStyle; }

    void updatePreservesTypingStyle(ETypingCommand);
    void markMisspellingsAfterTyping();
    void typingAddedToOpenCommand(ETypingCommand);
    bool makeEditableRootEmpty();
    
    ETypingCommand m_commandType;
    String m_textToInsert;
    bool m_openForMoreTyping;
    bool m_selectInsertedText;
    bool m_smartDelete;
    TextGranularity m_granularity;
    bool m_killRing;
    bool m_preservesTypingStyle;
    
    // Undoing a series of backward deletes will restore a selection around all of the
    // characters that were deleted, but only if the typing command being undone
    // was opened with a backward delete.
    bool m_openedByBackwardDelete;
};

} // namespace WebCore

#endif // TypingCommand_h
