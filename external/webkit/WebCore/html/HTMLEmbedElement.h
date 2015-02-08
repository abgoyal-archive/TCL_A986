

#ifndef HTMLEmbedElement_h
#define HTMLEmbedElement_h

#include "HTMLPlugInImageElement.h"

namespace WebCore {

class HTMLEmbedElement : public HTMLPlugInImageElement {
public:
    static PassRefPtr<HTMLEmbedElement> create(const QualifiedName&, Document*);

    void setNeedWidgetUpdate(bool needWidgetUpdate) { m_needWidgetUpdate = needWidgetUpdate; }

private:
    HTMLEmbedElement(const QualifiedName&, Document*);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusForbidden; }
    virtual int tagPriority() const { return 0; }

    virtual bool mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    virtual void attach();
    virtual bool canLazyAttach() { return false; }
    virtual bool rendererIsNeeded(RenderStyle*);
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
    virtual void insertedIntoDocument();
    virtual void removedFromDocument();
    virtual void attributeChanged(Attribute*, bool preserveDecls = false);
    
    virtual bool isURLAttribute(Attribute*) const;
    virtual const QualifiedName& imageSourceAttributeName() const;

    virtual void updateWidget();

    virtual RenderWidget* renderWidgetForJSBindings() const;

    virtual void addSubresourceAttributeURLs(ListHashSet<KURL>&) const;

    bool m_needWidgetUpdate;
};

}

#endif
