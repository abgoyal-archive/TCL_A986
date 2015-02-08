

#ifndef HTMLHeadElement_h
#define HTMLHeadElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLHeadElement : public HTMLElement {
public:
    HTMLHeadElement(const QualifiedName&, Document*);
    ~HTMLHeadElement();

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusOptional; }
    virtual int tagPriority() const { return 10; }
    virtual bool childAllowed(Node* newChild);
    virtual bool checkDTD(const Node* newChild);

    String profile() const;
    void setProfile(const String&);
};

} //namespace

#endif
