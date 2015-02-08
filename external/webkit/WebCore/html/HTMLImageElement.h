

#ifndef HTMLImageElement_h
#define HTMLImageElement_h

#include "GraphicsTypes.h"
#include "HTMLElement.h"
#include "HTMLImageLoader.h"

namespace WebCore {

class HTMLFormElement;

class HTMLImageElement : public HTMLElement {
    friend class HTMLFormElement;
public:
    HTMLImageElement(const QualifiedName&, Document*, HTMLFormElement* = 0);
    ~HTMLImageElement();

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusForbidden; }
    virtual int tagPriority() const { return 0; }

    virtual bool mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    virtual void attach();
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);

    virtual bool canStartSelection() const { return false; }

    int width(bool ignorePendingStylesheets = false) const;
    int height(bool ignorePendingStylesheets = false) const;

    int naturalWidth() const;
    int naturalHeight() const;
    
    bool isServerMap() const { return ismap && usemap.isEmpty(); }

    String altText() const;

    virtual bool isURLAttribute(Attribute*) const;

    CompositeOperator compositeOperator() const { return m_compositeOperator; }

    CachedImage* cachedImage() const { return m_imageLoader.image(); }
    void setCachedImage(CachedImage* i) { m_imageLoader.setImage(i); };

    void setLoadManually(bool loadManually) { m_imageLoader.setLoadManually(loadManually); }

    const AtomicString& alt() const;

    virtual bool draggable() const;

    void setHeight(int);

    int hspace() const;
    void setHspace(int);

    bool isMap() const;
    void setIsMap(bool);

    KURL longDesc() const;
    void setLongDesc(const String&);

    KURL lowsrc() const;
    void setLowsrc(const String&);

    KURL src() const;
    void setSrc(const String&);

    int vspace() const;
    void setVspace(int);

    void setWidth(int);

    int x() const;
    int y() const;

    bool complete() const;

    bool haveFiredLoadEvent() const { return m_imageLoader.haveFiredLoadEvent(); }

    virtual void addSubresourceAttributeURLs(ListHashSet<KURL>&) const;

private:
    virtual void insertedIntoDocument();
    virtual void removedFromDocument();
    virtual void insertedIntoTree(bool deep);
    virtual void removedFromTree(bool deep);

    HTMLImageLoader m_imageLoader;
    String usemap;
    bool ismap;
    HTMLFormElement* m_form;
    AtomicString m_name;
    AtomicString m_id;
    CompositeOperator m_compositeOperator;
};

} //namespace

#endif
