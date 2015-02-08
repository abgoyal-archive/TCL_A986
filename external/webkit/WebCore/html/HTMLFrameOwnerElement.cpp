

#include "config.h"
#include "HTMLFrameOwnerElement.h"

#include "DOMWindow.h"
#include "Frame.h"
#include "FrameLoader.h"

#if ENABLE(SVG)
#include "ExceptionCode.h"
#include "SVGDocument.h"
#endif

namespace WebCore {

HTMLFrameOwnerElement::HTMLFrameOwnerElement(const QualifiedName& tagName, Document* document)
    : HTMLElement(tagName, document, CreateElement)
    , m_contentFrame(0)
    , m_sandboxFlags(SandboxNone)
{
}

void HTMLFrameOwnerElement::willRemove()
{
    if (Frame* frame = contentFrame()) {
        frame->disconnectOwnerElement();
        frame->loader()->frameDetached();
    }

    HTMLElement::willRemove();
}

HTMLFrameOwnerElement::~HTMLFrameOwnerElement()
{
    if (m_contentFrame)
        m_contentFrame->disconnectOwnerElement();
}

Document* HTMLFrameOwnerElement::contentDocument() const
{
    return m_contentFrame ? m_contentFrame->document() : 0;
}

DOMWindow* HTMLFrameOwnerElement::contentWindow() const
{
    return m_contentFrame ? m_contentFrame->domWindow() : 0;
}

void HTMLFrameOwnerElement::setSandboxFlags(SandboxFlags flags)
{
    if (m_sandboxFlags == flags)
        return;

    m_sandboxFlags = flags;

    if (Frame* frame = contentFrame())
        frame->loader()->ownerElementSandboxFlagsChanged();
}

#if ENABLE(SVG)
SVGDocument* HTMLFrameOwnerElement::getSVGDocument(ExceptionCode& ec) const
{
    Document* doc = contentDocument();
    if (doc && doc->isSVGDocument())
        return static_cast<SVGDocument*>(doc);
    // Spec: http://www.w3.org/TR/SVG/struct.html#InterfaceGetSVGDocument
    ec = NOT_SUPPORTED_ERR;
    return 0;
}
#endif

} // namespace WebCore
