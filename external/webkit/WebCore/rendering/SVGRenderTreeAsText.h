

#ifndef SVGRenderTreeAsText_h
#define SVGRenderTreeAsText_h

#if ENABLE(SVG)

#include "TextStream.h"

namespace WebCore {

    class Color;
    class FloatPoint;
    class FloatRect;
    class FloatSize;
    class IntPoint;
    class IntRect;
    class Node;
    class RenderBlock;
    class RenderImage;
    class RenderObject;
    class RenderPath;
    class RenderSVGRoot;
    class RenderText;
    class AffineTransform;

// functions used by the main RenderTreeAsText code
void write(TextStream&, const RenderPath&, int indent);
void write(TextStream&, const RenderSVGRoot&, int indent);
void writeSVGContainer(TextStream&, const RenderObject&, int indent);
void writeSVGImage(TextStream&, const RenderImage&, int indent);
void writeSVGInlineText(TextStream&, const RenderText&, int indent);
void writeSVGText(TextStream&, const RenderBlock&, int indent);

void writeRenderResources(TextStream&, Node* parent);

// helper operators defined used in various classes to dump the render tree.
TextStream& operator<<(TextStream&, const AffineTransform&);
TextStream& operator<<(TextStream&, const IntRect&);
TextStream& operator<<(TextStream&, const Color&);
TextStream& operator<<(TextStream&, const IntPoint&);
TextStream& operator<<(TextStream&, const FloatSize&);
TextStream& operator<<(TextStream&, const FloatRect&);
TextStream& operator<<(TextStream&, const FloatPoint&);

// helper operators specific to dumping the render tree. these are used in various classes to dump the render tree
// these could be defined in separate namespace to avoid matching these generic signatures unintentionally.

template<typename Item>
TextStream& operator<<(TextStream& ts, const Vector<Item*>& v)
{
    ts << "[";

    for (unsigned i = 0; i < v.size(); i++) {
        ts << *v[i];
        if (i < v.size() - 1)
            ts << ", ";
    }

    ts << "]";
    return ts;
}

template<typename Item>
TextStream& operator<<(TextStream& ts, const Vector<Item>& v)
{
    ts << "[";

    for (unsigned i = 0; i < v.size(); i++) {
        ts << v[i];
        if (i < v.size() - 1)
            ts << ", ";
    }

    ts << "]";
    return ts;
}

template<typename Pointer>
TextStream& operator<<(TextStream& ts, Pointer* t)
{
    ts << reinterpret_cast<intptr_t>(t);
    return ts;
}

} // namespace WebCore

#endif // ENABLE(SVG)

#endif // SVGRenderTreeAsText_h
