
#ifndef HTMLStyleElement_h
#define HTMLStyleElement_h

#include "CSSStyleSheet.h"
#include "HTMLElement.h"
#include "StyleElement.h"

namespace WebCore {

class HTMLStyleElement : public HTMLElement, public StyleElement {
public:
    HTMLStyleElement(const QualifiedName&, Document*, bool createdByParser);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 1; }
    virtual bool checkDTD(const Node* newChild) { return newChild->isTextNode(); }

    // overload from HTMLElement
    virtual void parseMappedAttribute(MappedAttribute*);
    virtual void insertedIntoDocument();
    virtual void removedFromDocument();
    virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);

    virtual void finishParsingChildren();

    virtual bool isLoading() const;
    virtual bool sheetLoaded();

    bool disabled() const;
    void setDisabled(bool);

    virtual const AtomicString& media() const;
    void setMedia(const AtomicString&);

    virtual const AtomicString& type() const;
    void setType(const AtomicString&);

    StyleSheet* sheet();

    virtual void setLoading(bool loading) { m_loading = loading; }

    virtual void addSubresourceAttributeURLs(ListHashSet<KURL>&) const;

protected:
    String m_media;
    bool m_loading;
    bool m_createdByParser;
};

} //namespace

#endif
