

#ifndef GraphicsTypes_h
#define GraphicsTypes_h

namespace WebCore {

    class String;

    // Note: These constants exactly match the NSCompositeOperator constants of
    // AppKit on Mac OS X Tiger. If these ever change, we'll need to change the
    // Mac OS X Tiger platform code to map one to the other.
    enum CompositeOperator {
        CompositeClear,
        CompositeCopy,
        CompositeSourceOver,
        CompositeSourceIn,
        CompositeSourceOut,
        CompositeSourceAtop,
        CompositeDestinationOver,
        CompositeDestinationIn,
        CompositeDestinationOut,
        CompositeDestinationAtop,
        CompositeXOR,
        CompositePlusDarker,
        CompositeHighlight,
        CompositePlusLighter
    };

    // FIXME: Currently these constants have to match the values used in the SVG
    // DOM API. That's a mistake. We need to make cut that dependency.
    enum GradientSpreadMethod {
        SpreadMethodPad = 1,
        SpreadMethodReflect = 2,
        SpreadMethodRepeat = 3
    };

    enum LineCap { ButtCap, RoundCap, SquareCap };

    enum LineJoin { MiterJoin, RoundJoin, BevelJoin };

    enum HorizontalAlignment { AlignLeft, AlignRight, AlignHCenter };

    enum TextBaseline { AlphabeticTextBaseline, TopTextBaseline, MiddleTextBaseline, BottomTextBaseline, IdeographicTextBaseline, HangingTextBaseline };
    
    enum TextAlign { StartTextAlign, EndTextAlign, LeftTextAlign, CenterTextAlign, RightTextAlign };

    String compositeOperatorName(CompositeOperator);
    bool parseCompositeOperator(const String&, CompositeOperator&);

    String lineCapName(LineCap);
    bool parseLineCap(const String&, LineCap&);

    String lineJoinName(LineJoin);
    bool parseLineJoin(const String&, LineJoin&);

    String textAlignName(TextAlign);
    bool parseTextAlign(const String&, TextAlign&);
    
    String textBaselineName(TextBaseline);
    bool parseTextBaseline(const String&, TextBaseline&);

} // namespace WebCore

#endif
