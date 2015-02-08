

#ifndef RenderTreeAsText_h
#define RenderTreeAsText_h

namespace WebCore {

class Element;
class Frame;
class RenderObject;
class String;
class TextStream;

enum RenderAsTextBehaviorFlags {
    RenderAsTextBehaviorNormal = 0,
    RenderAsTextShowAllLayers = 1 << 0, // Dump all layers, not just those that would paint.
    RenderAsTextShowLayerNesting = 1 << 1, // Annotate the layer lists.
    RenderAsTextShowCompositedLayers = 1 << 2 // Show which layers are composited.
};
typedef unsigned RenderAsTextBehavior;

String externalRepresentation(Frame*, RenderAsTextBehavior = RenderAsTextBehaviorNormal);
void write(TextStream&, const RenderObject&, int indent = 0);

// Helper function shared with SVGRenderTreeAsText
String quoteAndEscapeNonPrintables(const String&);

String counterValueForElement(Element*);

} // namespace WebCore

#endif // RenderTreeAsText_h
