

#ifndef HTMLPlugInElement_h
#define HTMLPlugInElement_h

#include "HTMLFrameOwnerElement.h"
#include "ScriptInstance.h"

#if ENABLE(NETSCAPE_PLUGIN_API)
struct NPObject;
#endif

namespace WebCore {

class RenderWidget;

class HTMLPlugInElement : public HTMLFrameOwnerElement {
public:
    virtual ~HTMLPlugInElement();

    String height() const;
    void setHeight(const String&);
    
    String width() const;
    void setWidth(const String&);

    PassScriptInstance getInstance() const;

#if ENABLE(NETSCAPE_PLUGIN_API)
    NPObject* getNPObject();
#endif

protected:
    HTMLPlugInElement(const QualifiedName& tagName, Document*);

    virtual void detach();

    static void updateWidgetCallback(Node*);

    virtual bool mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const;
    virtual void parseMappedAttribute(MappedAttribute*);

#if PLATFORM(ANDROID)
    // in Android, plugin has a focused mode where it accepts all the touch events.
    // so need to claim that plugin element supports focus instead of using the default.
    virtual bool supportsFocus() const;
#endif

private:
    virtual void defaultEventHandler(Event*);

    virtual RenderWidget* renderWidgetForJSBindings() const = 0;

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual bool checkDTD(const Node* newChild);

    virtual void updateWidget() { }

protected:
    AtomicString m_name;

private:
    mutable ScriptInstance m_instance;
#if ENABLE(NETSCAPE_PLUGIN_API)
    NPObject* m_NPObject;
#endif
};

} // namespace WebCore

#endif // HTMLPlugInElement_h
