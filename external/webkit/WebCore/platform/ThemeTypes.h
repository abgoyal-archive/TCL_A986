

#ifndef ThemeTypes_h
#define ThemeTypes_h

namespace WebCore {

enum ControlState {
    HoverState = 1,
    PressedState = 1 << 1,
    FocusState = 1 << 2,
    EnabledState = 1 << 3,
    CheckedState = 1 << 4,
    ReadOnlyState = 1 << 5,
    DefaultState = 1 << 6,
    WindowInactiveState = 1 << 7,
    IndeterminateState = 1 << 8,
    SpinUpState = 1 << 9, // Sub-state for HoverState and PressedState.
    AllStates = 0xffffffff
};

typedef unsigned ControlStates;

// Must follow CSSValueKeywords.in order
enum ControlPart {
    NoControlPart, CheckboxPart, RadioPart, PushButtonPart, SquareButtonPart, ButtonPart,
    ButtonBevelPart, DefaultButtonPart, InnerSpinButtonPart, ListButtonPart, ListboxPart, ListItemPart,
    MediaFullscreenButtonPart, MediaMuteButtonPart, MediaPlayButtonPart, MediaSeekBackButtonPart, 
    MediaSeekForwardButtonPart, MediaRewindButtonPart, MediaReturnToRealtimeButtonPart, MediaToggleClosedCaptionsButtonPart,
    MediaSliderPart, MediaSliderThumbPart, MediaVolumeSliderContainerPart, MediaVolumeSliderPart, MediaVolumeSliderThumbPart,
    MediaControlsBackgroundPart, MediaCurrentTimePart, MediaTimeRemainingPart,
    MenulistPart, MenulistButtonPart, MenulistTextPart, MenulistTextFieldPart, OuterSpinButtonPart,
    SliderHorizontalPart, SliderVerticalPart, SliderThumbHorizontalPart,
    SliderThumbVerticalPart, CaretPart, SearchFieldPart, SearchFieldDecorationPart,
    SearchFieldResultsDecorationPart, SearchFieldResultsButtonPart,
    SearchFieldCancelButtonPart, TextFieldPart, TextAreaPart, CapsLockIndicatorPart
};

enum SelectionPart {
    SelectionBackground, SelectionForeground
};

enum ThemeFont {
    CaptionFont, IconFont, MenuFont, MessageBoxFont, SmallCaptionFont, StatusBarFont, MiniControlFont, SmallControlFont, ControlFont 
};

enum ThemeColor {
    ActiveBorderColor, ActiveCaptionColor, AppWorkspaceColor, BackgroundColor, ButtonFaceColor, ButtonHighlightColor, ButtonShadowColor,
    ButtonTextColor, CaptionTextColor, GrayTextColor, HighlightColor, HighlightTextColor, InactiveBorderColor, InactiveCaptionColor,
    InactiveCaptionTextColor, InfoBackgroundColor, InfoTextColor, MatchColor, MenuTextColor, ScrollbarColor, ThreeDDarkDhasowColor,
    ThreeDFaceColor, ThreeDHighlightColor, ThreeDLightShadowColor, ThreeDShadowCLor, WindowColor, WindowFrameColor, WindowTextColor,
    FocusRingColor
};

}
#endif
