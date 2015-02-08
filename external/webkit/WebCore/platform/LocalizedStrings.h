

#ifndef LocalizedStrings_h
#define LocalizedStrings_h

namespace WebCore {

    class String;
    class IntSize;
    
    String inputElementAltText();
    String resetButtonDefaultLabel();
    String searchableIndexIntroduction();
    String submitButtonDefaultLabel();
    String fileButtonChooseFileLabel();
    String fileButtonNoFileSelectedLabel();
    String copyImageUnknownFileLabel();
#if ENABLE(CONTEXT_MENUS)
    String contextMenuItemTagOpenLinkInNewWindow();
    String contextMenuItemTagDownloadLinkToDisk();
    String contextMenuItemTagCopyLinkToClipboard();
    String contextMenuItemTagOpenImageInNewWindow();
    String contextMenuItemTagDownloadImageToDisk();
    String contextMenuItemTagCopyImageToClipboard();
    String contextMenuItemTagOpenFrameInNewWindow();
    String contextMenuItemTagCopy();
    String contextMenuItemTagGoBack();
    String contextMenuItemTagGoForward();
    String contextMenuItemTagStop();
    String contextMenuItemTagReload();
    String contextMenuItemTagCut();
    String contextMenuItemTagPaste();
#if PLATFORM(GTK)
    String contextMenuItemTagDelete();
    String contextMenuItemTagSelectAll();
    String contextMenuItemTagInputMethods();
    String contextMenuItemTagUnicode();
#endif
    String contextMenuItemTagNoGuessesFound();
    String contextMenuItemTagIgnoreSpelling();
    String contextMenuItemTagLearnSpelling();
    String contextMenuItemTagSearchWeb();
    String contextMenuItemTagLookUpInDictionary();
    String contextMenuItemTagOpenLink();
    String contextMenuItemTagIgnoreGrammar();
    String contextMenuItemTagSpellingMenu();
    String contextMenuItemTagShowSpellingPanel(bool show);
    String contextMenuItemTagCheckSpelling();
    String contextMenuItemTagCheckSpellingWhileTyping();
    String contextMenuItemTagCheckGrammarWithSpelling();
    String contextMenuItemTagFontMenu();
    String contextMenuItemTagBold();
    String contextMenuItemTagItalic();
    String contextMenuItemTagUnderline();
    String contextMenuItemTagOutline();
    String contextMenuItemTagWritingDirectionMenu();
    String contextMenuItemTagTextDirectionMenu();
    String contextMenuItemTagDefaultDirection();
    String contextMenuItemTagLeftToRight();
    String contextMenuItemTagRightToLeft();
#if PLATFORM(MAC)
    String contextMenuItemTagSearchInSpotlight();
    String contextMenuItemTagShowFonts();
    String contextMenuItemTagStyles();
    String contextMenuItemTagShowColors();
    String contextMenuItemTagSpeechMenu();
    String contextMenuItemTagStartSpeaking();
    String contextMenuItemTagStopSpeaking();
    String contextMenuItemTagCorrectSpellingAutomatically();
    String contextMenuItemTagSubstitutionsMenu();
    String contextMenuItemTagShowSubstitutions(bool show);
    String contextMenuItemTagSmartCopyPaste();
    String contextMenuItemTagSmartQuotes();
    String contextMenuItemTagSmartDashes();
    String contextMenuItemTagSmartLinks();
    String contextMenuItemTagTextReplacement();
    String contextMenuItemTagTransformationsMenu();
    String contextMenuItemTagMakeUpperCase();
    String contextMenuItemTagMakeLowerCase();
    String contextMenuItemTagCapitalize();
    String contextMenuItemTagChangeBack(const String& replacedString);
#endif
    String contextMenuItemTagInspectElement();
#endif // ENABLE(CONTEXT_MENUS)

    String searchMenuNoRecentSearchesText();
    String searchMenuRecentSearchesText();
    String searchMenuClearRecentSearchesText();

    String AXWebAreaText();
    String AXLinkText();
    String AXListMarkerText();
    String AXImageMapText();
    String AXHeadingText();
    String AXDefinitionListTermText();
    String AXDefinitionListDefinitionText();
    String AXARIAContentGroupText(const String& ariaType);
    
    String AXButtonActionVerb();
    String AXRadioButtonActionVerb();
    String AXTextFieldActionVerb();
    String AXCheckedCheckBoxActionVerb();
    String AXUncheckedCheckBoxActionVerb();
    String AXMenuListActionVerb();
    String AXMenuListPopupActionVerb();
    String AXLinkActionVerb();

    String multipleFileUploadText(unsigned numberOfFiles);
    String unknownFileSizeText();

#if PLATFORM(WIN)
    String uploadFileText();
    String allFilesText();
#endif

    String imageTitle(const String& filename, const IntSize& size);

    String mediaElementLoadingStateText();
    String mediaElementLiveBroadcastStateText();
    String localizedMediaControlElementString(const String&);
    String localizedMediaControlElementHelpText(const String&);
    String localizedMediaTimeDescription(float);

    String validationMessageValueMissingText();
    String validationMessageTypeMismatchText();
    String validationMessagePatternMismatchText();
    String validationMessageTooLongText();
    String validationMessageRangeUnderflowText();
    String validationMessageRangeOverflowText();
    String validationMessageStepMismatchText();

}

#endif
