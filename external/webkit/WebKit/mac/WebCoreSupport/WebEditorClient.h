

#import <WebCore/EditorClient.h>
#import <wtf/RetainPtr.h>
#import <wtf/Forward.h>
#import <wtf/Vector.h>

@class WebView;
@class WebEditorUndoTarget;

class WebEditorClient : public WebCore::EditorClient {
public:
    WebEditorClient(WebView *);
    
    virtual void pageDestroyed();

    virtual bool isGrammarCheckingEnabled();
    virtual void toggleGrammarChecking();
    virtual bool isContinuousSpellCheckingEnabled();
    virtual void toggleContinuousSpellChecking();
    virtual int spellCheckerDocumentTag();

    virtual bool smartInsertDeleteEnabled();
    virtual bool isSelectTrailingWhitespaceEnabled();
    virtual bool isEditable();

    virtual bool shouldDeleteRange(WebCore::Range*);    
    virtual bool shouldShowDeleteInterface(WebCore::HTMLElement*);

    virtual bool shouldBeginEditing(WebCore::Range*);
    virtual bool shouldEndEditing(WebCore::Range*);
    virtual bool shouldInsertNode(WebCore::Node*, WebCore::Range*, WebCore::EditorInsertAction);
    virtual bool shouldInsertText(const WebCore::String&, WebCore::Range*, WebCore::EditorInsertAction);
    virtual bool shouldChangeSelectedRange(WebCore::Range* fromRange, WebCore::Range* toRange, WebCore::EAffinity, bool stillSelecting);

    virtual bool shouldApplyStyle(WebCore::CSSStyleDeclaration*, WebCore::Range*);
    
    virtual bool shouldMoveRangeAfterDelete(WebCore::Range* range, WebCore::Range* rangeToBeReplaced); 

    virtual void didBeginEditing();
    virtual void didEndEditing();
    virtual void didWriteSelectionToPasteboard();
    virtual void didSetSelectionTypesForPasteboard();

    virtual NSString* userVisibleString(NSURL*);
#ifdef BUILDING_ON_TIGER
    virtual NSArray* pasteboardTypesForSelection(WebCore::Frame*);
#endif
    
#if !defined(BUILDING_ON_TIGER) && !defined(BUILDING_ON_LEOPARD)
    virtual void uppercaseWord();
    virtual void lowercaseWord();
    virtual void capitalizeWord();
    virtual void showSubstitutionsPanel(bool show);
    virtual bool substitutionsPanelIsShowing();
    virtual void toggleSmartInsertDelete();
    virtual bool isAutomaticQuoteSubstitutionEnabled();
    virtual void toggleAutomaticQuoteSubstitution();
    virtual bool isAutomaticLinkDetectionEnabled();
    virtual void toggleAutomaticLinkDetection();
    virtual bool isAutomaticDashSubstitutionEnabled();
    virtual void toggleAutomaticDashSubstitution();
    virtual bool isAutomaticTextReplacementEnabled();
    virtual void toggleAutomaticTextReplacement();
    virtual bool isAutomaticSpellingCorrectionEnabled();
    virtual void toggleAutomaticSpellingCorrection();
#endif

    virtual void respondToChangedContents();
    virtual void respondToChangedSelection();

    virtual void registerCommandForUndo(PassRefPtr<WebCore::EditCommand>);
    virtual void registerCommandForRedo(PassRefPtr<WebCore::EditCommand>);
    virtual void clearUndoRedoOperations();
    
    virtual bool canUndo() const;
    virtual bool canRedo() const;
    
    virtual void undo();
    virtual void redo();
    
    virtual void handleKeyboardEvent(WebCore::KeyboardEvent*);
    virtual void handleInputMethodKeydown(WebCore::KeyboardEvent*);

    virtual void textFieldDidBeginEditing(WebCore::Element*);
    virtual void textFieldDidEndEditing(WebCore::Element*);
    virtual void textDidChangeInTextField(WebCore::Element*);
    virtual bool doTextFieldCommandFromEvent(WebCore::Element*, WebCore::KeyboardEvent*);
    virtual void textWillBeDeletedInTextField(WebCore::Element*);
    virtual void textDidChangeInTextArea(WebCore::Element*);
    
    virtual void ignoreWordInSpellDocument(const WebCore::String&);
    virtual void learnWord(const WebCore::String&);
    virtual void checkSpellingOfString(const UChar*, int length, int* misspellingLocation, int* misspellingLength);
    virtual WebCore::String getAutoCorrectSuggestionForMisspelledWord(const WebCore::String&);
    virtual void checkGrammarOfString(const UChar*, int length, WTF::Vector<WebCore::GrammarDetail>&, int* badGrammarLocation, int* badGrammarLength);
    virtual void checkTextOfParagraph(const UChar* text, int length, uint64_t checkingTypes, WTF::Vector<WebCore::TextCheckingResult>& results);
    virtual void updateSpellingUIWithGrammarString(const WebCore::String&, const WebCore::GrammarDetail&);
    virtual void updateSpellingUIWithMisspelledWord(const WebCore::String&);
    virtual void showSpellingUI(bool show);
    virtual bool spellingUIIsShowing();
    virtual void getGuessesForWord(const WebCore::String&, WTF::Vector<WebCore::String>& guesses);
    virtual void setInputMethodState(bool enabled);
private:
    void registerCommandForUndoOrRedo(PassRefPtr<WebCore::EditCommand>, bool isRedo);
    WebEditorClient();
    
    WebView *m_webView;
    RetainPtr<WebEditorUndoTarget> m_undoTarget;
    
    bool m_haveUndoRedoOperations;
};
