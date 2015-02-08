

#ifndef SVGGlyphMap_h
#define SVGGlyphMap_h

#if ENABLE(SVG_FONTS)
#include "SVGGlyphElement.h"


namespace WebCore {

    struct GlyphMapNode;

    typedef HashMap<UChar, RefPtr<GlyphMapNode> > GlyphMapLayer;

    struct GlyphMapNode : public RefCounted<GlyphMapNode> {
    private:
        GlyphMapNode() { }
    public:
        static PassRefPtr<GlyphMapNode> create() { return adoptRef(new GlyphMapNode); }

        Vector<SVGGlyphIdentifier> glyphs;

        GlyphMapLayer children;
    };

    class SVGGlyphMap {

    public:
        SVGGlyphMap() : m_currentPriority(0) { }

        void add(const String& string, const SVGGlyphIdentifier& glyph) 
        {
            size_t len = string.length();
            GlyphMapLayer* currentLayer = &m_rootLayer;

            RefPtr<GlyphMapNode> node;
            for (size_t i = 0; i < len; i++) {
                UChar curChar = string[i];
                node = currentLayer->get(curChar);
                if (!node) {
                    node = GlyphMapNode::create();
                    currentLayer->set(curChar, node);
                }
                currentLayer = &node->children;
            }

            if (node) {
                node->glyphs.append(glyph);
                node->glyphs.last().priority = m_currentPriority++;
                node->glyphs.last().nameLength = len;
                node->glyphs.last().isValid = true;
            }
        }

        static inline bool compareGlyphPriority(const SVGGlyphIdentifier& first, const SVGGlyphIdentifier& second)
        {
            return first.priority < second.priority;
        }

        void get(const String& string, Vector<SVGGlyphIdentifier>& glyphs)
        {
            GlyphMapLayer* currentLayer = &m_rootLayer;

            for (size_t i = 0; i < string.length(); i++) {
                UChar curChar = string[i];
                RefPtr<GlyphMapNode> node = currentLayer->get(curChar);
                if (!node)
                    break;
                glyphs.append(node->glyphs);
                currentLayer = &node->children;
            }
            std::sort(glyphs.begin(), glyphs.end(), compareGlyphPriority);
        }

        void clear() 
        { 
            m_rootLayer.clear(); 
            m_currentPriority = 0;
        }

    private:
        GlyphMapLayer m_rootLayer;
        int m_currentPriority;
    };

}

#endif // ENABLE(SVG_FONTS)


#endif //SVGGlyphMap_h
