

namespace WebCore {

    class Color;
    class Font;
    class GraphicsContext;
    class IntPoint;
    class String;

    void WebCoreDrawTextAtPoint(GraphicsContext&, const String&, const IntPoint&, const Font&, const Color&, int underlinedIndex = -1);
    void WebCoreDrawDoubledTextAtPoint(GraphicsContext&, const String&, const IntPoint&, const Font&, const Color& topColor, const Color& bottomColor, int underlinedIndex = -1);
    float WebCoreTextFloatWidth(const String&, const Font&);

    void WebCoreSetShouldUseFontSmoothing(bool);
    bool WebCoreShouldUseFontSmoothing();

    void WebCoreSetAlwaysUsesComplexTextCodePath(bool);
    bool WebCoreAlwaysUsesComplexTextCodePath();

} // namespace WebCore
