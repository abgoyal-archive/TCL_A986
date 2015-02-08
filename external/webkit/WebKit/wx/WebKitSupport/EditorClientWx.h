

#ifndef EditorClientWx_h
#define EditorClientWx_h

#include "EditorClient.h"
#include "Page.h"

#include "WebView.h"
#include "WebFrame.h"

namespace WebCore {

class EditorClientWx : public EditorClient {
friend class ::wxWebView;
friend class ::wxWebFrame;

public:
    virtual ~EditorClientWx();
    void setPage(Page*);
    virtual void pageDestroyed();

    virtual bool shouldDeleteRange(Range*);
    virtual bool shouldShowDeleteInterface(HTMLElement*);
    virtual bool smartInsertDeleteEnabled();
    virtual bool isSelectTrailingWhitespaceEnabled();
    virtual bool isContinuousSpellCheckingEnabled();
    virtual void toggleContinuousSpellChecking();
    virtual bool isGrammarCheckingEnabled();
    virtual void toggleGrammarChecking();
    virtual int spellCheckerDocumentTag();

    virtual bool selectWordBeforeMenuEvent();
    virtual bool isEditable();

    virtual bool shouldBeginEditing(Range*);
    virtual bool shouldEndEditing(Range*);
    virtual bool shouldInsertNode(Node*, Range*,
                                  EditorInsertAction);
    virtual bool shouldInsertText(const String&, Range*,
                                  EditorInsertAction);
    virtual bool shouldApplyStyle(CSSStyleDeclaration*,
                                  Range*);
    virtual bool shouldMoveRangeAfterDelete(Range*, Range*);
    virtual bool shouldChangeSelectedRange(Range* fromRange, Range* toRange, 
                                EAffinity, bool stillSelecting);

    virtual void didBeginEditing();
    virtual void respondToChangedContents();
    virtual void respondToChangedSelection();
    virtual void didEndEditing();
    virtual void didWriteSelectionToPasteboard();
    virtual void didSetSelectionTypesForPasteboard();

    virtual void registerCommandForUndo(PassRefPtr<EditCommand>);
    virtual void registerCommandForRedo(PassRefPtr<EditCommand>);
    virtual void clearUndoRedoOperations();

    virtual bool canUndo() const;
    virtual bool canRedo() const;

    virtual void undo();
    virtual void redo();
    
    virtual const char* interpretKeyEvent(const KeyboardEvent*);
    virtual bool handleEditingKeyboardEvent(KeyboardEvent*);
    virtual void handleKeyboardEvent(KeyboardEvent*);
    virtual void handleInputMethodKeydown(KeyboardEvent*);
    
    virtual void textFieldDidBeginEditing(Element*);
    virtual void textFieldDidEndEditing(Element*);
    virtual void textDidChangeInTextField(Element*);
    virtual bool doTextFieldCommandFromEvent(Element*, KeyboardEvent*);
    virtual void textWillBeDeletedInTextField(Element*);
    virtual void textDidChangeInTextArea(Element*);

    virtual void ignoreWordInSpellDocument(const String&);
    virtual void learnWord(const String&);
    virtual void checkSpellingOfString(const UChar*, int length, int* misspellingLocation, int* misspellingLength);
    virtual void checkGrammarOfString(const UChar*, int length, Vector<GrammarDetail>&, int* badGrammarLocation, int* badGrammarLength);
    virtual void updateSpellingUIWithGrammarString(const String&, const GrammarDetail& detail);
    virtual void updateSpellingUIWithMisspelledWord(const String&);
    virtual void showSpellingUI(bool show);
    virtual bool spellingUIIsShowing();
    virtual void getGuessesForWord(const String&, Vector<String>& guesses);
    virtual String getAutoCorrectSuggestionForMisspelledWord(const WebCore::String&);
    
    virtual void setInputMethodState(bool enabled);

private:
    Page* m_page;
};

}

#endif // EditorClientWx_h
