

#ifndef HTMLBodyElement_h
#define HTMLBodyElement_h

#include "HTMLElement.h"
#include "Document.h"

namespace WebCore {

class HTMLBodyElement : public HTMLElement {
public:
    HTMLBodyElement(const QualifiedName&, Document*);
    virtual ~HTMLBodyElement();

    String aLink() const;
    void setALink(const String&);
    String bgColor() const;
    void setBgColor(const String&);
    String link() const;
    void setLink(const String&);
    String text() const;
    void setText(const String&);
    String vLink() const;
    void setVLink(const String&);

    // Declared virtual in Element
    DEFINE_WINDOW_ATTRIBUTE_EVENT_LISTENER(blur);
    DEFINE_WINDOW_ATTRIBUTE_EVENT_LISTENER(error);
    DEFINE_WINDOW_ATTRIBUTE_EVENT_LISTENER(focus);
    DEFINE_WINDOW_ATTRIBUTE_EVENT_LISTENER(load);

    DEFINE_WINDOW_ATTRIBUTE_EVENT_LISTENER(beforeunload);
    DEFINE_WINDOW_ATTRIBUTE_EVENT_LISTENER(hashchange);
    DEFINE_WINDOW_ATTRIBUTE_EVENT_LISTENER(message);
    DEFINE_WINDOW_ATTRIBUTE_EVENT_LISTENER(offline);
    DEFINE_WINDOW_ATTRIBUTE_EVENT_LISTENER(online);
    DEFINE_WINDOW_ATTRIBUTE_EVENT_LISTENER(popstate);
    DEFINE_WINDOW_ATTRIBUTE_EVENT_LISTENER(resize);
    DEFINE_WINDOW_ATTRIBUTE_EVENT_LISTENER(storage);
    DEFINE_WINDOW_ATTRIBUTE_EVENT_LISTENER(unload);

#if ENABLE(ORIENTATION_EVENTS)
    DEFINE_WINDOW_ATTRIBUTE_EVENT_LISTENER(orientationchange);
#endif

private:
    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 10; }
    
    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    virtual void insertedIntoDocument();

    void createLinkDecl();
    
    virtual bool isURLAttribute(Attribute*) const;

    virtual int scrollLeft() const;
    virtual void setScrollLeft(int scrollLeft);
    
    virtual int scrollTop() const;
    virtual void setScrollTop(int scrollTop);
    
    virtual int scrollHeight() const;
    virtual int scrollWidth() const;
    
    virtual void addSubresourceAttributeURLs(ListHashSet<KURL>&) const;
    
    virtual void didMoveToNewOwnerDocument();

    RefPtr<CSSMutableStyleDeclaration> m_linkDecl;
};

} //namespace

#endif
