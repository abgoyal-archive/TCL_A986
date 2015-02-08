

#ifndef EditAction_h
#define EditAction_h

namespace WebCore {
    typedef enum {
        EditActionUnspecified,
        EditActionSetColor,
        EditActionSetBackgroundColor,
        EditActionTurnOffKerning,
        EditActionTightenKerning,
        EditActionLoosenKerning,
        EditActionUseStandardKerning,
        EditActionTurnOffLigatures,
        EditActionUseStandardLigatures,
        EditActionUseAllLigatures,
        EditActionRaiseBaseline,
        EditActionLowerBaseline,
        EditActionSetTraditionalCharacterShape,
        EditActionSetFont,
        EditActionChangeAttributes,
        EditActionAlignLeft,
        EditActionAlignRight,
        EditActionCenter,
        EditActionJustify,
        EditActionSetWritingDirection,
        EditActionSubscript,
        EditActionSuperscript,
        EditActionUnderline,
        EditActionOutline,
        EditActionUnscript,
        EditActionDrag,
        EditActionCut,
        EditActionPaste,
        EditActionPasteFont,
        EditActionPasteRuler,
        EditActionTyping,
        EditActionCreateLink,
        EditActionUnlink,
        EditActionFormatBlock,
        EditActionInsertList,
        EditActionIndent,
        EditActionOutdent
    } EditAction;    
}

#endif
