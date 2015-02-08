
#ifndef HTMLMetaElement_h
#define HTMLMetaElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLMetaElement : public HTMLElement {
public:
    HTMLMetaElement(const QualifiedName&, Document*);
    ~HTMLMetaElement();

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusForbidden; }
    virtual int tagPriority() const { return 0; }

    virtual void parseMappedAttribute(MappedAttribute*);
    virtual void insertedIntoDocument();

    void process();

    String content() const;
    void setContent(const String&);

    String httpEquiv() const;
    void setHttpEquiv(const String&);

    String name() const;
    void setName(const String&);

    String scheme() const;
    void setScheme(const String&);

protected:
    String m_equiv;
    String m_content;
};

} //namespace

#endif
