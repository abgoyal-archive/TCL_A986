

#ifndef HTMLHtmlElement_h
#define HTMLHtmlElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLHtmlElement : public HTMLElement {
public:
    HTMLHtmlElement(const QualifiedName&, Document*);
    ~HTMLHtmlElement();

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 11; }
    virtual bool checkDTD(const Node* newChild);

#if ENABLE(OFFLINE_WEB_APPLICATIONS)
    virtual void insertedIntoDocument();
#endif

    String version() const;
    void setVersion(const String&);
};

} //namespace

#endif
