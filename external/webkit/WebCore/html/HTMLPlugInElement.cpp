

#include "config.h"
#include "HTMLPlugInElement.h"

#include "CSSPropertyNames.h"
#include "Document.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "FrameTree.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"
#include "Page.h"
#include "RenderWidget.h"
#include "ScriptController.h"
#include "Settings.h"
#include "Widget.h"

#if ENABLE(NETSCAPE_PLUGIN_API)
#include "npruntime_impl.h"
#endif

namespace WebCore {

using namespace HTMLNames;

HTMLPlugInElement::HTMLPlugInElement(const QualifiedName& tagName, Document* doc)
    : HTMLFrameOwnerElement(tagName, doc)
#if ENABLE(NETSCAPE_PLUGIN_API)
    , m_NPObject(0)
#endif
{
}

HTMLPlugInElement::~HTMLPlugInElement()
{
    ASSERT(!m_instance); // cleared in detach()

#if ENABLE(NETSCAPE_PLUGIN_API)
    if (m_NPObject) {
        _NPN_ReleaseObject(m_NPObject);
        m_NPObject = 0;
    }
#endif
}

void HTMLPlugInElement::detach()
{
    m_instance.clear();
    HTMLFrameOwnerElement::detach();
}

PassScriptInstance HTMLPlugInElement::getInstance() const
{
    Frame* frame = document()->frame();
    if (!frame)
        return 0;

    // If the host dynamically turns off JavaScript (or Java) we will still return
    // the cached allocated Bindings::Instance.  Not supporting this edge-case is OK.
    if (m_instance)
        return m_instance;

    RenderWidget* renderWidget = renderWidgetForJSBindings();
    if (renderWidget && renderWidget->widget())
        m_instance = frame->script()->createScriptInstanceForWidget(renderWidget->widget());

    return m_instance;
}

String HTMLPlugInElement::height() const
{
    return getAttribute(heightAttr);
}

void HTMLPlugInElement::setHeight(const String& value)
{
    setAttribute(heightAttr, value);
}

String HTMLPlugInElement::width() const
{
    return getAttribute(widthAttr);
}

void HTMLPlugInElement::setWidth(const String& value)
{
    setAttribute(widthAttr, value);
}

bool HTMLPlugInElement::mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const
{
    if (attrName == widthAttr ||
        attrName == heightAttr ||
        attrName == vspaceAttr ||
        attrName == hspaceAttr) {
            result = eUniversal;
            return false;
    }
    
    if (attrName == alignAttr) {
        result = eReplaced; // Share with <img> since the alignment behavior is the same.
        return false;
    }
    
    return HTMLFrameOwnerElement::mapToEntry(attrName, result);
}

void HTMLPlugInElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == widthAttr)
        addCSSLength(attr, CSSPropertyWidth, attr->value());
    else if (attr->name() == heightAttr)
        addCSSLength(attr, CSSPropertyHeight, attr->value());
    else if (attr->name() == vspaceAttr) {
        addCSSLength(attr, CSSPropertyMarginTop, attr->value());
        addCSSLength(attr, CSSPropertyMarginBottom, attr->value());
    } else if (attr->name() == hspaceAttr) {
        addCSSLength(attr, CSSPropertyMarginLeft, attr->value());
        addCSSLength(attr, CSSPropertyMarginRight, attr->value());
    } else if (attr->name() == alignAttr)
        addHTMLAlignment(attr);
    else
        HTMLFrameOwnerElement::parseMappedAttribute(attr);
}

bool HTMLPlugInElement::checkDTD(const Node* newChild)
{
    return newChild->hasTagName(paramTag) || HTMLFrameOwnerElement::checkDTD(newChild);
}

void HTMLPlugInElement::defaultEventHandler(Event* event)
{
    RenderObject* r = renderer();
    if (!r || !r->isWidget())
        return;
    Widget* widget = toRenderWidget(r)->widget();
    if (!widget)
        return;
    widget->handleEvent(event);
}

#if ENABLE(NETSCAPE_PLUGIN_API)

NPObject* HTMLPlugInElement::getNPObject()
{
    ASSERT(document()->frame());
    if (!m_NPObject)
        m_NPObject = document()->frame()->script()->createScriptObjectForPluginElement(this);
    return m_NPObject;
}

#endif /* ENABLE(NETSCAPE_PLUGIN_API) */

void HTMLPlugInElement::updateWidgetCallback(Node* n)
{
    static_cast<HTMLPlugInElement*>(n)->updateWidget();
}

#if PLATFORM(ANDROID)
bool HTMLPlugInElement::supportsFocus() const
{
    return true;
}
#endif

}
