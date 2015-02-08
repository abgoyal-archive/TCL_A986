

#ifndef HTMLFrameOwnerElement_h
#define HTMLFrameOwnerElement_h

#include "FrameLoaderTypes.h"
#include "HTMLElement.h"

namespace WebCore {

class DOMWindow;
class Frame;

#if ENABLE(SVG)
class SVGDocument;
#endif

class HTMLFrameOwnerElement : public HTMLElement {
public:
    virtual ~HTMLFrameOwnerElement();

    Frame* contentFrame() const { return m_contentFrame; }
    DOMWindow* contentWindow() const;
    Document* contentDocument() const;

#if ENABLE(SVG)
    SVGDocument* getSVGDocument(ExceptionCode&) const;
#endif

    virtual ScrollbarMode scrollingMode() const { return ScrollbarAuto; }

    SandboxFlags sandboxFlags() const { return m_sandboxFlags; }

protected:
    HTMLFrameOwnerElement(const QualifiedName& tagName, Document*);

    void setSandboxFlags(SandboxFlags);

    virtual void willRemove();

private:
    friend class Frame;

    virtual bool isFrameOwnerElement() const { return true; }
    virtual bool isKeyboardFocusable(KeyboardEvent*) const { return m_contentFrame; }

    Frame* m_contentFrame;
    SandboxFlags m_sandboxFlags;
};

} // namespace WebCore

#endif // HTMLFrameOwnerElement_h
