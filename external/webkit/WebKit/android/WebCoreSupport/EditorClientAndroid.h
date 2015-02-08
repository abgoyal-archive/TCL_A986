

#ifndef EditorClientAndroid_h
#define EditorClientAndroid_h

#include "EditorClient.h"
#include "Page.h"

using namespace WebCore;

namespace android {

class EditorClientAndroid : public EditorClient {
public:
    EditorClientAndroid() {
        m_shouldChangeSelectedRange = true;
        m_uiGeneratedSelectionChange = false;
    }
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
    
    virtual bool isEditable();
    
    virtual bool shouldBeginEditing(Range*);
    virtual bool shouldEndEditing(Range*);
    virtual bool shouldInsertNode(Node*, Range*, EditorInsertAction);
    virtual bool shouldInsertText(const String&, Range*, EditorInsertAction);
    virtual bool shouldChangeSelectedRange(Range* fromRange, Range* toRange, EAffinity, bool stillSelecting);

    virtual bool shouldApplyStyle(CSSStyleDeclaration*, Range*);
//  virtual bool shouldChangeTypingStyle(CSSStyleDeclaration* fromStyle, CSSStyleDeclaration* toStyle);
//  virtual bool doCommandBySelector(SEL selector);
    virtual bool shouldMoveRangeAfterDelete(Range*, Range*);

    virtual void didBeginEditing();
    virtual void respondToChangedContents();
    virtual void respondToChangedSelection();
    virtual void didEndEditing();
    virtual void didWriteSelectionToPasteboard();
    virtual void didSetSelectionTypesForPasteboard();
//  virtual void didChangeTypingStyle:(NSNotification *)notification;
//  virtual void didChangeSelection:(NSNotification *)notification;
//  virtual NSUndoManager* undoManager:(WebView *)webView;

    virtual void registerCommandForUndo(PassRefPtr<EditCommand>);
    virtual void registerCommandForRedo(PassRefPtr<EditCommand>);
    virtual void clearUndoRedoOperations();
    
    virtual bool canUndo() const;
    virtual bool canRedo() const;
    
    virtual void undo();
    virtual void redo();

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
    virtual String getAutoCorrectSuggestionForMisspelledWord(const String& misspelledWorld);
    virtual void checkGrammarOfString(const UChar*, int length, WTF::Vector<GrammarDetail>&, int* badGrammarLocation, int* badGrammarLength);
    virtual void updateSpellingUIWithGrammarString(const String&, const GrammarDetail& detail);
    virtual void updateSpellingUIWithMisspelledWord(const String&);
    virtual void showSpellingUI(bool show);
    virtual bool spellingUIIsShowing();
    virtual void getGuessesForWord(const String&, WTF::Vector<String>& guesses);
    virtual void setInputMethodState(bool);

    // Android specific:
    void setPage(Page* page) { m_page = page; }
    void setShouldChangeSelectedRange(bool shouldChangeSelectedRange) { m_shouldChangeSelectedRange = shouldChangeSelectedRange; }
    void setUiGeneratedSelectionChange(bool uiGenerated) { m_uiGeneratedSelectionChange = uiGenerated; }
private:
    Page* m_page;
    bool  m_shouldChangeSelectedRange;
    bool  m_uiGeneratedSelectionChange;
};

}

#endif
