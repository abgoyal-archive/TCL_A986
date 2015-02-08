

#ifndef HTMLScriptElement_h
#define HTMLScriptElement_h

#include "ScriptElement.h"
#include "HTMLElement.h"

namespace WebCore {

class HTMLScriptElement : public HTMLElement
                        , public ScriptElement {
public:
    HTMLScriptElement(const QualifiedName&, Document*, bool createdByParser);
    ~HTMLScriptElement();

    virtual bool shouldExecuteAsJavaScript() const;
    virtual String scriptContent() const;

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 1; }
    virtual bool checkDTD(const Node* newChild) { return newChild->isTextNode(); }

    virtual void parseMappedAttribute(MappedAttribute*);
    virtual void insertedIntoDocument();
    virtual void removedFromDocument();
    virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);

    virtual bool isURLAttribute(Attribute*) const;
    virtual void finishParsingChildren();

    String text() const;
    void setText(const String&);

    String htmlFor() const;
    void setHtmlFor(const String&);

    String event() const;
    void setEvent(const String&);

    String charset() const;
    void setCharset(const String&);

    bool defer() const;
    void setDefer(bool);

    KURL src() const;
    void setSrc(const String&);

    String type() const;
    void setType(const String&);

    virtual String scriptCharset() const;
    
    virtual void addSubresourceAttributeURLs(ListHashSet<KURL>&) const;

    bool haveFiredLoadEvent() const { return m_data.haveFiredLoadEvent(); }

protected:
    virtual String sourceAttributeValue() const;
    virtual String charsetAttributeValue() const;
    virtual String typeAttributeValue() const;
    virtual String languageAttributeValue() const;
    virtual String forAttributeValue() const;

    virtual void dispatchLoadEvent();
    virtual void dispatchErrorEvent();

private:
    ScriptElementData m_data;
};

} //namespace

#endif
