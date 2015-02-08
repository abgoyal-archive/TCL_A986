

#ifndef TextRun_h
#define TextRun_h

#include "PlatformString.h"

namespace WebCore {

class RenderObject;
class SVGPaintServer;

class TextRun {
public:
    TextRun(const UChar* c, int len, bool allowTabs = false, int xpos = 0, int padding = 0, bool rtl = false, bool directionalOverride = false,
              bool applyRunRounding = true, bool applyWordRounding = true)
        : m_characters(c)
        , m_len(len)
        , m_xpos(xpos)
        , m_padding(padding)
        , m_allowTabs(allowTabs)
        , m_rtl(rtl)
        , m_directionalOverride(directionalOverride)
        , m_applyRunRounding(applyRunRounding)
        , m_applyWordRounding(applyWordRounding)
        , m_disableSpacing(false)
#if ENABLE(SVG_FONTS)
        , m_referencingRenderObject(0)
        , m_activePaintServer(0)
#endif
    {
    }

    TextRun(const String& s, bool allowTabs = false, int xpos = 0, int padding = 0, bool rtl = false, bool directionalOverride = false,
              bool applyRunRounding = true, bool applyWordRounding = true)
        : m_characters(s.characters())
        , m_len(s.length())
        , m_xpos(xpos)
        , m_padding(padding)
        , m_allowTabs(allowTabs)
        , m_rtl(rtl)
        , m_directionalOverride(directionalOverride)
        , m_applyRunRounding(applyRunRounding)
        , m_applyWordRounding(applyWordRounding)
        , m_disableSpacing(false)
#if ENABLE(SVG_FONTS)
        , m_referencingRenderObject(0)
        , m_activePaintServer(0)
#endif
    {
    }

    UChar operator[](int i) const { return m_characters[i]; }
    const UChar* data(int i) const { return &m_characters[i]; }

    const UChar* characters() const { return m_characters; }
    int length() const { return m_len; }

    void setText(const UChar* c, int len) { m_characters = c; m_len = len; }

    bool allowTabs() const { return m_allowTabs; }
    int xPos() const { return m_xpos; }
    int padding() const { return m_padding; }
    bool rtl() const { return m_rtl; }
    bool ltr() const { return !m_rtl; }
    bool directionalOverride() const { return m_directionalOverride; }
    bool applyRunRounding() const { return m_applyRunRounding; }
    bool applyWordRounding() const { return m_applyWordRounding; }
    bool spacingDisabled() const { return m_disableSpacing; }

    void disableSpacing() { m_disableSpacing = true; }
    void disableRoundingHacks() { m_applyRunRounding = m_applyWordRounding = false; }
    void setRTL(bool b) { m_rtl = b; }
    void setDirectionalOverride(bool override) { m_directionalOverride = override; }

#if ENABLE(SVG_FONTS)
    RenderObject* referencingRenderObject() const { return m_referencingRenderObject; }
    void setReferencingRenderObject(RenderObject* object) { m_referencingRenderObject = object; }

    SVGPaintServer* activePaintServer() const { return m_activePaintServer; }
    void setActivePaintServer(SVGPaintServer* object) { m_activePaintServer = object; }
#endif

private:
    const UChar* m_characters;
    int m_len;

    int m_xpos;
    int m_padding;
    bool m_allowTabs;
    bool m_rtl;
    bool m_directionalOverride;
    bool m_applyRunRounding;
    bool m_applyWordRounding;
    bool m_disableSpacing;

#if ENABLE(SVG_FONTS)
    RenderObject* m_referencingRenderObject;
    SVGPaintServer* m_activePaintServer;
#endif
};

}

#endif
