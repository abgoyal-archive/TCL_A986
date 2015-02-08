

#ifndef HTMLObjectElement_h
#define HTMLObjectElement_h

#include "HTMLPlugInImageElement.h"

namespace WebCore {

class HTMLObjectElement : public HTMLPlugInImageElement {
public:
    static PassRefPtr<HTMLObjectElement> create(const QualifiedName&, Document*, bool createdByParser);

    void setNeedWidgetUpdate(bool needWidgetUpdate) { m_needWidgetUpdate = needWidgetUpdate; }

    void renderFallbackContent();

    bool declare() const;
    void setDeclare(bool);

    int hspace() const;
    void setHspace(int);

    int vspace() const;
    void setVspace(int);

    bool isDocNamedItem() const { return m_docNamedItem; }

    const String& classId() const { return m_classId; }

    bool containsJavaApplet() const;

private:
    HTMLObjectElement(const QualifiedName&, Document*, bool createdByParser);

    virtual int tagPriority() const { return 5; }

    virtual void parseMappedAttribute(MappedAttribute*);

    virtual void attach();
    virtual bool canLazyAttach() { return false; }
    virtual bool rendererIsNeeded(RenderStyle*);
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
    virtual void finishParsingChildren();
    virtual void detach();
    virtual void insertedIntoDocument();
    virtual void removedFromDocument();
    
    virtual void recalcStyle(StyleChange);
    virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);

    virtual bool isURLAttribute(Attribute*) const;
    virtual const QualifiedName& imageSourceAttributeName() const;

    virtual void updateWidget();

    virtual RenderWidget* renderWidgetForJSBindings() const;

    virtual void addSubresourceAttributeURLs(ListHashSet<KURL>&) const;

    void updateDocNamedItem();

    AtomicString m_id;
    String m_classId;
    bool m_docNamedItem : 1;
    bool m_needWidgetUpdate : 1;
    bool m_useFallbackContent : 1;
};

}

#endif
